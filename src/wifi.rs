use aligned::{A4, Aligned};
use embassy_futures::join::join;
use embassy_net::{Config, Stack, StackResources};
use embassy_net_driver_channel::{
    self as channel,
    driver::{HardwareAddress, LinkState},
};
use embassy_time::{Delay, Duration, Timer, with_timeout};
use esp_hal::{Async, gpio::Output, rng::Rng, sdmmc::Slot};
use esp_hosted::{
    RpcIdP as RpcId, RpcP as Rpc, RpcTypeP as RpcType,
    proto::{self, Rpc_::Payload},
};
use micropb::{MessageDecode, PbDecoder};
use sdio::{MmcError, sdio::SdioCard};
use static_cell::{ConstStaticCell, StaticCell};

const SSID: &str = env!("P4_WIFI_SSID");
#[cfg(has_wifi_password)]
const PASSWORD: &[u8] = include_bytes!("../.cargo/wifi-password");
#[cfg(not(has_wifi_password))]
const PASSWORD: &[u8] = b"";
const FUNCTION: u8 = 1;
const BLOCK: usize = 512;
const FRAME: usize = 1536;
const MTU: usize = 1514;
const TOKEN_REGISTER: u32 = 0x044;
const PACKET_LENGTH_REGISTER: u32 = 0x060;
const HOST_INTERRUPT_REGISTER: u32 = 0x08c;
const RX_MASK: u32 = 0x000f_ffff;
const TX_MASK: u32 = 0x0fff;
const FIFO_END: u32 = 0x1f800;
const HEADER: usize = 12;
const SERIAL_TLV: usize = 12;

type Card = SdioCard<Slot<'static, 1, Async>, Delay>;

#[derive(Debug)]
enum Error {
    Bus(MmcError),
    Timeout,
    Frame,
    Rpc(RpcId, i32),
    Credentials,
}

impl From<MmcError> for Error {
    fn from(error: MmcError) -> Self {
        Self::Bus(error)
    }
}

struct Hosted {
    card: Card,
    rx_bytes: u32,
    tx_buffers: u32,
    uid: u32,
    associated: bool,
}

struct Frames<'a>(&'a [u8]);

impl<'a> Iterator for Frames<'a> {
    type Item = (u8, &'a [u8]);

    fn next(&mut self) -> Option<Self::Item> {
        let header = self.0.get(..HEADER)?;
        let length = usize::from(u16::from_le_bytes([header[2], header[3]]));
        let offset = usize::from(u16::from_le_bytes([header[4], header[5]]));
        if offset < HEADER || length == 0 {
            return None;
        }
        let payload = self.0.get(offset..offset + length)?;
        self.0 = &self.0[offset + length..];
        Some((header[0] & 0x0f, payload))
    }
}

impl Hosted {
    async fn register(&mut self, address: u32) -> Result<u32, Error> {
        let mut data = Aligned::<A4, _>([0; 4]);
        self.card
            .cmd53_read_bytes(FUNCTION, true, address, &mut data)
            .await?;
        Ok(u32::from_le_bytes(*data))
    }

    async fn receive<'a>(
        &mut self,
        buffer: &'a mut Aligned<A4, [u8]>,
    ) -> Result<Frames<'a>, Error> {
        let counter = self.register(PACKET_LENGTH_REGISTER).await? & RX_MASK;
        let len = counter.wrapping_sub(self.rx_bytes) & RX_MASK;
        if len == 0 || counter == RX_MASK {
            return Ok(Frames(&[]));
        }
        let padded = (len as usize).next_multiple_of(4);
        if padded > buffer.len() {
            return Err(Error::Frame);
        }
        self.card
            .cmd53_read::<BLOCK>(FUNCTION, FIFO_END - len, &mut buffer[..padded])
            .await?;
        self.rx_bytes = counter;
        let data: &[u8] = buffer;
        Ok(Frames(&data[..len as usize]))
    }

    async fn send(&mut self, frame: &Aligned<A4, [u8]>, len: usize) -> Result<(), Error> {
        let wait_for_credit = async {
            loop {
                let counter = (self.register(TOKEN_REGISTER).await? >> 16) & TX_MASK;
                if counter.wrapping_sub(self.tx_buffers) & TX_MASK != 0 {
                    return Ok::<_, Error>(());
                }
                Timer::after_millis(2).await;
            }
        };
        with_timeout(Duration::from_millis(500), wait_for_credit)
            .await
            .map_err(|_| Error::Timeout)??;
        self.card
            .cmd53_write::<BLOCK>(
                FUNCTION,
                FIFO_END - len as u32,
                &frame[..len.next_multiple_of(4)],
            )
            .await?;
        self.tx_buffers = (self.tx_buffers + 1) & TX_MASK;
        Ok(())
    }

    async fn init(&mut self, rx: &mut Aligned<A4, [u8]>) -> Result<(), Error> {
        self.card.reacquire(20_000_000).await?;
        self.card.enable_functions(1 << FUNCTION).await?;
        self.card.set_block_size(FUNCTION, BLOCK as u16).await?;
        self.card
            .cmd52_write(FUNCTION, HOST_INTERRUPT_REGISTER, 1)
            .await?;
        loop {
            if self.receive(rx).await?.any(|(interface, payload)| {
                interface == 5
                    && payload.len() >= 2
                    && payload[0] == 0x22
                    && usize::from(payload[1]) <= payload.len() - 2
            }) {
                return Ok(());
            }
            Timer::after_millis(10).await;
        }
    }

    fn handle_event(&mut self, rpc: &Rpc) {
        match rpc.payload {
            Some(Payload::EventStaConnected(_)) => self.associated = true,
            Some(Payload::EventStaDisconnected(_)) => self.associated = false,
            _ => {}
        }
    }

    async fn rpc(
        &mut self,
        rx: &mut Aligned<A4, [u8]>,
        id: RpcId,
        payload: Payload,
    ) -> Result<Payload, Error> {
        self.uid = self.uid.wrapping_add(1);
        let uid = self.uid;
        let mut storage = Aligned::<A4, _>([0; BLOCK]);
        let frame: &mut [u8] = &mut *storage;
        let len = esp_hosted::setup_rpc_proto(
            frame,
            Rpc {
                msg_type: RpcType::Req,
                msg_id: id,
                uid,
                payload: Some(payload),
            },
        )
        .map_err(|_| Error::Frame)?;
        self.send(&storage, len).await?;
        let response = async {
            loop {
                let mut response = None;
                for (interface, data) in self.receive(rx).await? {
                    if let Some(rpc) = decode_rpc(interface, data) {
                        self.handle_event(&rpc);
                        if rpc.uid == uid
                            && rpc.msg_type == RpcType::Resp
                            && rpc.msg_id.0 == id.0 + 256
                        {
                            response = rpc.payload;
                        }
                    }
                }
                if let Some(payload) = response {
                    return Ok(payload);
                }
                Timer::after_millis(5).await;
            }
        };
        match with_timeout(Duration::from_secs(3), response).await {
            Ok(result) => result,
            Err(_) => Err(Error::Rpc(id, -1)),
        }
    }

    async fn configure(
        &mut self,
        rx: &mut Aligned<A4, [u8]>,
        password: &[u8],
    ) -> Result<[u8; 6], Error> {
        let mut init = proto::Rpc_Req_WifiInit::default();
        init.set_cfg(proto::wifi_init_config {
            static_rx_buf_num: 10,
            dynamic_rx_buf_num: 32,
            tx_buf_type: 1,
            dynamic_tx_buf_num: 32,
            ampdu_rx_enable: 1,
            ampdu_tx_enable: 1,
            rx_ba_win: 6,
            beacon_max_len: 752,
            mgmt_sbuf_num: 32,
            espnow_max_encrypt_num: 7,
            magic: 0x1f2f3f4f,
            ..Default::default()
        });
        self.command(rx, RpcId::ReqWifiInit, Payload::ReqWifiInit(init))
            .await?;
        self.command(
            rx,
            RpcId::ReqSetWifiMode,
            Payload::ReqSetWifiMode(proto::Rpc_Req_SetMode { mode: 1 }),
        )
        .await?;
        let mut station = proto::wifi_sta_config::default();
        station
            .ssid
            .extend_from_slice(SSID.as_bytes())
            .map_err(|_| Error::Credentials)?;
        station
            .password
            .extend_from_slice(password)
            .map_err(|_| Error::Credentials)?;
        station.set_threshold(proto::wifi_scan_threshold {
            rssi: -127,
            authmode: 3,
        });
        station.set_pmf_cfg(proto::wifi_pmf_config {
            capable: true,
            required: false,
        });
        let mut config = proto::Rpc_Req_WifiSetConfig::default();
        config.set_cfg(proto::wifi_config {
            u: Some(proto::wifi_config_::U::Sta(station)),
        });
        self.command(
            rx,
            RpcId::ReqWifiSetConfig,
            Payload::ReqWifiSetConfig(config),
        )
        .await?;
        self.command(
            rx,
            RpcId::ReqWifiStart,
            Payload::ReqWifiStart(Default::default()),
        )
        .await?;
        match self
            .rpc(
                rx,
                RpcId::ReqGetMacAddress,
                Payload::ReqGetMacAddress(proto::Rpc_Req_GetMacAddress { mode: 0 }),
            )
            .await?
        {
            Payload::RespGetMacAddress(reply) if reply.resp == 0 => {
                reply.mac.as_slice().try_into().map_err(|_| Error::Frame)
            }
            Payload::RespGetMacAddress(reply) => {
                Err(Error::Rpc(RpcId::ReqGetMacAddress, reply.resp))
            }
            _ => Err(Error::Frame),
        }
    }

    async fn command(
        &mut self,
        rx: &mut Aligned<A4, [u8]>,
        id: RpcId,
        payload: Payload,
    ) -> Result<(), Error> {
        let status = match self.rpc(rx, id, payload).await? {
            Payload::RespWifiInit(reply) => reply.resp,
            Payload::RespSetWifiMode(reply) => reply.resp,
            Payload::RespWifiSetConfig(reply) => reply.resp,
            Payload::RespWifiStart(reply) => reply.resp,
            Payload::RespWifiConnect(reply) => reply.resp,
            _ => return Err(Error::Frame),
        };
        if status == 0 {
            Ok(())
        } else {
            Err(Error::Rpc(id, status))
        }
    }

    async fn run_network(
        &mut self,
        rx: &mut Aligned<A4, [u8]>,
        runner: &mut channel::Runner<'_, MTU>,
    ) -> Result<(), Error> {
        let mut tx = Aligned::<A4, _>([0; FRAME]);
        let mut reconnect = embassy_time::Instant::now();
        loop {
            if !self.associated && embassy_time::Instant::now() >= reconnect {
                if let Err(error) = self
                    .command(
                        rx,
                        RpcId::ReqWifiConnect,
                        Payload::ReqWifiConnect(Default::default()),
                    )
                    .await
                {
                    if matches!(error, Error::Bus(_) | Error::Timeout | Error::Frame) {
                        return Err(error);
                    }
                    report(error);
                }
                reconnect = embassy_time::Instant::now() + Duration::from_secs(10);
            }
            for (interface, payload) in self.receive(rx).await? {
                if let Some(rpc) = decode_rpc(interface, payload) {
                    self.handle_event(&rpc);
                } else if interface == 1 && self.associated && payload.len() <= MTU {
                    if let Some(packet) = runner.try_rx_buf() {
                        packet[..payload.len()].copy_from_slice(payload);
                        runner.rx_done(payload.len());
                    }
                }
            }
            runner.set_link_state(if self.associated {
                LinkState::Up
            } else {
                LinkState::Down
            });
            if let Some(packet) = runner.try_tx_buf() {
                let result = if self.associated {
                    let frame: &mut [u8] = &mut *tx;
                    let len = HEADER + packet.len();
                    frame[..HEADER].fill(0);
                    frame[0] = 1;
                    frame[2..4].copy_from_slice(&(packet.len() as u16).to_le_bytes());
                    frame[4..6].copy_from_slice(&(HEADER as u16).to_le_bytes());
                    frame[HEADER..len].copy_from_slice(packet);
                    frame[len..len.next_multiple_of(4)].fill(0);
                    let checksum = frame[..len]
                        .iter()
                        .fold(0u16, |sum, &byte| sum.wrapping_add(u16::from(byte)));
                    frame[6..8].copy_from_slice(&checksum.to_le_bytes());
                    self.send(&tx, len).await
                } else {
                    Ok(())
                };
                runner.tx_done();
                result?;
            }
            Timer::after_millis(3).await;
        }
    }

    async fn restart(
        &mut self,
        rx: &mut Aligned<A4, [u8]>,
        reset: &mut Output<'_>,
        password: &[u8],
    ) -> [u8; 6] {
        loop {
            self.rx_bytes = 0;
            self.tx_buffers = 0;
            self.associated = false;
            reset.set_low();
            Timer::after_millis(20).await;
            reset.set_high();
            Timer::after_millis(500).await;
            let result = match with_timeout(Duration::from_secs(3), self.init(rx)).await {
                Ok(Ok(())) => {
                    esp_println::println!("ESP-Hosted initialization event received from C6");
                    self.configure(rx, password).await
                }
                Ok(Err(error)) => Err(error),
                Err(_) => Err(Error::Timeout),
            };
            match result {
                Ok(mac) => {
                    esp_println::println!("Connecting to Wi-Fi {SSID}");
                    return mac;
                }
                Err(error) => report(error),
            }
            Timer::after_secs(5).await;
        }
    }
}

fn decode_rpc(interface: u8, payload: &[u8]) -> Option<Rpc> {
    if interface != 3 || payload.len() < SERIAL_TLV {
        return None;
    }
    let tlv = &payload[..SERIAL_TLV];
    if tlv[0] != 1
        || tlv[1..3] != [6, 0]
        || tlv[9] != 2
        || (&tlv[3..9] != b"RPCRsp" && &tlv[3..9] != b"RPCEvt")
        || usize::from(u16::from_le_bytes([tlv[10], tlv[11]])) != payload.len() - SERIAL_TLV
    {
        return None;
    }
    let data = &payload[SERIAL_TLV..];
    let mut rpc = Rpc::default();
    rpc.decode(&mut PbDecoder::new(data), data.len()).ok()?;
    Some(rpc)
}

fn report(error: Error) {
    match error {
        Error::Bus(error) => esp_println::println!("ESP-Hosted SDIO error: {error:?}"),
        Error::Rpc(id, status) => {
            esp_println::println!("ESP-Hosted RPC {} failed ({status:#x})", id.0)
        }
        error => esp_println::println!("Wi-Fi startup or transport error: {error:?}"),
    }
}

async fn print_lease(stack: Stack<'_>) {
    loop {
        stack.wait_config_up().await;
        if let Some(config) = stack.config_v4() {
            esp_println::println!("Wi-Fi joined {SSID}, IPv4 {}", config.address);
        }
        stack.wait_config_down().await;
    }
}

#[embassy_executor::task]
pub async fn run(slot: Slot<'static, 1, Async>, mut reset: Output<'static>) {
    let password = PASSWORD.strip_suffix(b"\n").unwrap_or(PASSWORD);
    let password = password.strip_suffix(b"\r").unwrap_or(password);
    if SSID.is_empty() || SSID.len() > 32 {
        esp_println::println!("P4_WIFI_SSID must contain 1 to 32 bytes");
        return;
    }
    if !(8..=63).contains(&password.len())
        || !password.iter().all(|byte| matches!(byte, 0x20..=0x7e))
    {
        esp_println::println!("Wi-Fi requires a local WPA2 passphrase in .cargo/wifi-password");
        return;
    }

    // The coprocessor can queue multiple frames before the host polls it.
    static RX: ConstStaticCell<Aligned<A4, [u8; 16 * FRAME]>> =
        ConstStaticCell::new(Aligned([0; 16 * FRAME]));
    let rx: &mut Aligned<A4, [u8]> = RX.take();
    let mut hosted = Hosted {
        card: SdioCard::new_uninit(slot, Delay),
        rx_bytes: 0,
        tx_buffers: 0,
        uid: 0,
        associated: false,
    };
    let mac = hosted.restart(rx, &mut reset, password).await;

    static CHANNEL: StaticCell<channel::State<MTU, 4, 4>> = StaticCell::new();
    static RESOURCES: StaticCell<StackResources<3>> = StaticCell::new();
    let (mut transport, device) = channel::new(
        CHANNEL.init_with(channel::State::new),
        HardwareAddress::Ethernet(mac),
    );
    let rng = Rng::new();
    let seed = u64::from(rng.random()) << 32 | u64::from(rng.random());
    let (stack, mut network) = embassy_net::new(
        device,
        Config::dhcpv4(Default::default()),
        RESOURCES.init_with(StackResources::new),
        seed,
    );
    join(join(network.run(), print_lease(stack)), async {
        loop {
            if let Err(error) = hosted.run_network(rx, &mut transport).await {
                report(error);
            }
            transport.set_link_state(LinkState::Down);
            let mac = hosted.restart(rx, &mut reset, password).await;
            transport.set_hardware_address(HardwareAddress::Ethernet(mac));
        }
    })
    .await;
}
