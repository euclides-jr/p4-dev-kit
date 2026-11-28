//! Fixed-address transport bring-up for the Sendspin client.
#[path = "sendspin/clock.rs"]
mod clock;
#[path = "sendspin/noise.rs"]
mod noise;
#[path = "sendspin/pcm.rs"]
pub(crate) mod pcm;

use base64::{Engine, engine::general_purpose::STANDARD};
use core::fmt::Write;
use core::net::Ipv4Addr;
use embassy_executor::Spawner;
use embassy_net::{Stack, tcp::TcpSocket};
use embassy_time::{Duration, Timer, with_timeout};
use embedded_io_async::Write as _;
use sha1::{Digest, Sha1};

const PORT: u16 = 8927;
const GUID: &[u8] = b"258EAFA5-E914-47DA-95CA-C5AB0DC85B11";

pub fn start(spawner: Spawner, stack: Stack<'static>) {
    let Some(address) = option_env!("SENDSPIN_SERVER_IP").filter(|_| pcm::enabled()) else {
        return;
    };
    match address.parse() {
        Ok(address) => spawner.spawn(probe(stack, address).unwrap()),
        Err(_) => esp_println::println!("SENDSPIN_SERVER_IP must be an IPv4 address"),
    }
}

#[embassy_executor::task]
async fn probe(stack: Stack<'static>, address: Ipv4Addr) {
    let Some(buffers) = pcm::take_buffers() else {
        return;
    };
    let mut tx = [0; 512];
    stack.wait_config_up().await;
    let Some(identity) = noise::identity() else {
        esp_println::println!(
            "Set SENDSPIN_IDENTITY_KEY to a base64url-encoded 32-byte private key"
        );
        return;
    };
    // Retain Music Assistant's last volume across transport/session failures.
    let mut volume = 25u8;
    loop {
        stack.wait_config_up().await;
        {
            let mut socket = TcpSocket::new(stack, &mut *buffers.tcp_rx, &mut tx);
            socket.set_nagle_enabled(false);
            let mut nonce = [0; 16];
            esp_hal::rng::Rng::new().read(&mut nonce);
            let handshake = async {
                upgrade(&mut socket, address, nonce).await?;
                noise::handshake(&mut socket, &identity).await
            };
            match with_timeout(Duration::from_secs(10), handshake).await {
                Ok(Ok((transport, legacy))) => {
                    if let Err(reason) = noise::run_session(
                        &mut socket,
                        transport,
                        legacy,
                        buffers.frame,
                        buffers.plaintext,
                        &mut volume,
                    )
                    .await
                    {
                        esp_println::println!("Sendspin session ended: {}", reason);
                        pcm::reset();
                    }
                }
                Ok(Err(reason)) => esp_println::println!("Sendspin probe failed: {}", reason),
                Err(_) => esp_println::println!("Sendspin probe timed out"),
            }
        }
        Timer::after_secs(5).await;
    }
}

async fn upgrade(
    socket: &mut TcpSocket<'_>,
    address: Ipv4Addr,
    nonce: [u8; 16],
) -> Result<(), &'static str> {
    let mut key = [0; 24];
    STANDARD.encode_slice(nonce, &mut key).unwrap();
    let key = core::str::from_utf8(&key).unwrap();
    let mut request = Header {
        bytes: [0; 256],
        len: 0,
    };
    write!(
        request,
        "GET /sendspin HTTP/1.1\r\nHost: {}:{}\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Version: 13\r\nSec-WebSocket-Key: {}\r\n\r\n",
        address, PORT, key
    )
    .map_err(|_| "request too large")?;
    socket
        .connect((address, PORT))
        .await
        .map_err(|_| "TCP connect")?;
    socket
        .write_all(&request.bytes[..request.len])
        .await
        .map_err(|_| "request write")?;

    let mut headers = [0; 1024];
    let mut used = 0;
    let end = loop {
        if used == headers.len() {
            return Err("response headers too large");
        }
        let count = socket
            .read(&mut headers[used..used + 1])
            .await
            .map_err(|_| "response read")?;
        if count == 0 {
            return Err("server closed during upgrade");
        }
        used += count;
        if let Some(end) = headers[..used]
            .windows(4)
            .position(|bytes| bytes == b"\r\n\r\n")
        {
            break end;
        }
    };
    let mut hash = Sha1::new();
    hash.update(key);
    hash.update(GUID);
    let mut accept = [0; 28];
    STANDARD.encode_slice(hash.finalize(), &mut accept).unwrap();
    validate_headers(&headers[..end], &accept)
}

fn validate_headers(headers: &[u8], accept: &[u8]) -> Result<(), &'static str> {
    let headers = core::str::from_utf8(headers).map_err(|_| "invalid HTTP response")?;
    let mut lines = headers.split("\r\n");
    let mut status = lines.next().unwrap_or_default().split_ascii_whitespace();
    if status.next() != Some("HTTP/1.1") || status.next() != Some("101") {
        return Err("server refused WebSocket upgrade");
    }
    let mut websocket = false;
    let mut connection = false;
    let mut accepted = false;
    for (name, value) in lines.filter_map(|line| line.split_once(':')) {
        let value = value.trim();
        if name.eq_ignore_ascii_case("upgrade") {
            websocket |= value.eq_ignore_ascii_case("websocket");
        } else if name.eq_ignore_ascii_case("connection") {
            connection |= value
                .split(',')
                .any(|token| token.trim().eq_ignore_ascii_case("upgrade"));
        } else if name.eq_ignore_ascii_case("sec-websocket-accept") {
            accepted |= value.as_bytes() == accept;
        }
    }
    if websocket && connection && accepted {
        Ok(())
    } else {
        Err("invalid WebSocket upgrade headers")
    }
}

struct Header {
    bytes: [u8; 256],
    len: usize,
}
impl core::fmt::Write for Header {
    fn write_str(&mut self, text: &str) -> core::fmt::Result {
        let end = self.len + text.len();
        self.bytes
            .get_mut(self.len..end)
            .ok_or(core::fmt::Error)?
            .copy_from_slice(text.as_bytes());
        self.len = end;
        Ok(())
    }
}
