extern crate alloc;

use self::alloc::boxed::Box;
use base64::{Engine, engine::general_purpose::URL_SAFE_NO_PAD};
use core::fmt::Write;
use embassy_futures::select::{Either, select};
use embassy_net::tcp::TcpSocket;
use embassy_time::{Duration, Instant, Timer, with_timeout};
use embedded_io_async::{Read as _, Write as _};
use esp_hal::rng::Rng;
use serde::Deserialize;
use snow::{
    Builder, Keypair, TransportState,
    params::{CipherChoice, DHChoice, HashChoice},
    resolvers::{CryptoResolver, DefaultResolver},
    types::{Cipher, Dh, Hash, Random},
};

use super::pcm;

const SUITE: &str = "25519_ChaChaPoly_SHA256";
const SENTINEL_ID: &str = "GFsV9tLaSQm9HcFWpKsgYQOr7wFTvNUtkmFwuVz3zoo";
const SENTINEL: [u8; 32] = [
    0x1b, 0x5e, 0x24, 0xdb, 0xc1, 0xae, 0xd9, 0x5f, 0xc2, 0xa5, 0xa3, 0x38, 0xa9, 0x0c, 0x05, 0xdf,
    0x44, 0xbd, 0x10, 0xf5, 0xec, 0x1f, 0x4c, 0xd6, 0x6c, 0xbf, 0x86, 0x27, 0x27, 0x67, 0xb9, 0xd3,
];

struct WifiEntropy;

impl Random for WifiEntropy {
    fn try_fill_bytes(&mut self, dest: &mut [u8]) -> Result<(), snow::Error> {
        // This resolver is only used after Wi-Fi is up, with the RF entropy source active.
        Rng::new().read(dest);
        Ok(())
    }
}

impl CryptoResolver for WifiEntropy {
    fn resolve_rng(&self) -> Option<Box<dyn Random>> {
        Some(Box::new(Self))
    }

    fn resolve_dh(&self, choice: &DHChoice) -> Option<Box<dyn Dh>> {
        DefaultResolver.resolve_dh(choice)
    }

    fn resolve_hash(&self, choice: &HashChoice) -> Option<Box<dyn Hash>> {
        DefaultResolver.resolve_hash(choice)
    }

    fn resolve_cipher(&self, choice: &CipherChoice) -> Option<Box<dyn Cipher>> {
        DefaultResolver.resolve_cipher(choice)
    }
}

fn builder<'a>() -> Builder<'a> {
    Builder::with_resolver(
        "Noise_KKpsk2_25519_ChaChaPoly_SHA256".parse().unwrap(),
        Box::new(WifiEntropy),
    )
}

pub(super) fn identity() -> Option<Keypair> {
    let key = option_env!("SENDSPIN_IDENTITY_KEY")?;
    let mut private = [0; 32];
    if URL_SAFE_NO_PAD.decode_slice(key, &mut private).ok()? != private.len()
        || private.iter().all(|&byte| byte == 0)
    {
        return None;
    }
    let mut dh = DefaultResolver.resolve_dh(&DHChoice::Curve25519)?;
    dh.set(&private);
    Some(Keypair {
        private: private.to_vec(),
        public: dh.pubkey().to_vec(),
    })
}

#[derive(Deserialize)]
struct Envelope<'a, T> {
    #[serde(rename = "type")]
    kind: &'a str,
    payload: T,
}

#[derive(Deserialize)]
struct ServerInit<'a> {
    server_id: &'a str,
    version: u32,
}

#[derive(Deserialize)]
struct Handshake<'a> {
    data: &'a str,
}

#[derive(Deserialize)]
struct Psk<'a> {
    psk_id: &'a str,
    psk_category: Option<&'a str>,
}

#[derive(Deserialize)]
struct MessageType<'a> {
    #[serde(rename = "type")]
    kind: &'a str,
}

#[derive(Deserialize)]
struct Activation<'a> {
    #[serde(borrow, default)]
    active_roles: Option<self::alloc::vec::Vec<&'a str>>,
}

pub(super) async fn handshake(
    socket: &mut TcpSocket<'_>,
    identity: &Keypair,
) -> Result<(TransportState, bool), &'static str> {
    let mut client_id = [0; 43];
    URL_SAFE_NO_PAD
        .encode_slice(&identity.public, &mut client_id)
        .unwrap();
    let client_id = core::str::from_utf8(&client_id).unwrap();
    esp_println::println!("Sendspin client ID: {}", client_id);

    let mut message = super::Header {
        bytes: [0; 256],
        len: 0,
    };
    write!(message, "{{\"type\":\"client/init\",\"payload\":{{\"client_id\":\"{client_id}\",\"version\":1,\"suite\":\"{SUITE}\"}}}}")
        .map_err(|_| "client/init size")?;
    let mut prologue = [0; 768];
    let init_len = message.len;
    prologue[..init_len].copy_from_slice(&message.bytes[..init_len]);
    send_frame(socket, 1, &mut message.bytes[..init_len]).await?;
    let len = receive(socket, 1, &mut prologue[init_len..]).await?;
    let prologue = &prologue[..init_len + len];
    let init: Envelope<'_, ServerInit<'_>> =
        serde_json::from_slice(&prologue[init_len..]).map_err(|_| "invalid server/init JSON")?;
    if init.kind != "server/init" || init.payload.version != 1 {
        return Err("unexpected server/init");
    }
    let mut remote_key = [0; 32];
    if init.payload.server_id.len() != 43
        || URL_SAFE_NO_PAD
            .decode_slice(init.payload.server_id, &mut remote_key)
            .ok()
            != Some(32)
    {
        return Err("invalid server public key");
    }

    let mut noise = builder()
        .local_private_key(&identity.private)
        .and_then(|builder| builder.remote_public_key(&remote_key))
        .and_then(|builder| builder.prologue(prologue))
        .and_then(|builder| builder.psk(2, &SENTINEL))
        .and_then(Builder::build_responder)
        .map_err(|_| "Noise setup")?;
    let mut frame = [0; 1024];
    let len = receive(socket, 1, &mut frame).await?;
    let handshake: Envelope<'_, Handshake<'_>> =
        serde_json::from_slice(&frame[..len]).map_err(|_| "invalid noise/handshake JSON")?;
    if handshake.kind != "noise/handshake" {
        return Err("expected Noise handshake message");
    }
    let mut ciphertext = [0; 256];
    let len = URL_SAFE_NO_PAD
        .decode_slice(handshake.payload.data, &mut ciphertext)
        .map_err(|_| "invalid Noise handshake encoding")?;
    let mut plaintext = [0; 1024];
    let len = noise
        .read_message(&ciphertext[..len], &mut plaintext)
        .map_err(|_| "Noise handshake verification")?;
    let psk: Psk<'_> =
        serde_json::from_slice(&plaintext[..len]).map_err(|_| "invalid PSK descriptor JSON")?;
    // Earlier servers omit the category. Accept that form only for the public Sentinel.
    let legacy_pair_methods = psk.psk_category.is_none();
    let valid = match psk.psk_category {
        None | Some("sn") => psk.psk_id == SENTINEL_ID,
        Some("lt" | "pr") => psk.psk_id.len() == 43,
        Some(_) => false,
    };
    if !valid {
        return Err("invalid PSK descriptor");
    }

    let len = noise
        .write_message(b"{}", &mut ciphertext)
        .map_err(|_| "Noise handshake reply")?;
    let mut encoded = [0; 128];
    let len = URL_SAFE_NO_PAD
        .encode_slice(&ciphertext[..len], &mut encoded)
        .map_err(|_| "Noise reply encoding")?;
    let encoded = core::str::from_utf8(&encoded[..len]).unwrap();
    message.len = 0;
    write!(
        message,
        "{{\"type\":\"noise/handshake\",\"payload\":{{\"data\":\"{encoded}\"}}}}"
    )
    .map_err(|_| "Noise reply size")?;
    send_frame(socket, 1, &mut message.bytes[..message.len]).await?;

    let mut transport = noise
        .into_transport_mode()
        .map_err(|_| "Noise transport setup")?;
    let len = receive(socket, 2, &mut frame).await?;
    let len = transport
        .read_message(&frame[..len], &mut plaintext)
        .map_err(|_| "Noise transport verification")?;
    let hello: MessageType<'_> = decode_json(&plaintext[..len])?;
    if hello.kind != "server/hello" {
        return Err("expected encrypted server/hello");
    }
    esp_println::println!("Sendspin Noise handshake verified");

    use crate::audio::SAMPLE_RATE;
    // Music Assistant 2.10.5 uses the list form and omits the PSK category.
    let pair_methods = if legacy_pair_methods {
        serde_json::json!([{ "method": "pairing_psk" }])
    } else {
        serde_json::json!({ "pairing_psk": {} })
    };
    let mut hello = serde_json::json!({
        "type": "client/hello",
        "payload": {
            "name": "ESP32-P4",
            "supported_roles": ["player@v1"],
            "player@v1_support": {
                "supported_formats": [{
                    "codec": "pcm",
                    "channels": 2,
                    "sample_rate": SAMPLE_RATE,
                    "bit_depth": 16
                }],
                "buffer_capacity": pcm::CAPACITY
            },
            "supported_pair_methods": pair_methods,
            "unpaired_access": { "enabled": true }
        }
    });
    if legacy_pair_methods {
        hello["payload"]["player@v1_support"]["supported_commands"] =
            serde_json::json!(["volume", "mute"]);
    }
    send_json(socket, &mut transport, &hello, &mut plaintext, &mut frame).await?;

    Ok((transport, legacy_pair_methods))
}

pub(super) async fn run_session(
    socket: &mut TcpSocket<'_>,
    mut transport: TransportState,
    legacy: bool,
    frame: &mut [u8],
    plaintext: &mut [u8],
    saved_volume: &mut u8,
) -> Result<(), &'static str> {
    let mut ready = false;
    let mut state_sent = false;
    let mut streaming = false;
    let mut chunks = 0u32;
    let mut volume = *saved_volume;
    let mut muted = false;
    let mut last_active = None;
    let mut next_probe = Instant::now();
    let mut outstanding = None;
    let mut clock = super::clock::ClockFilter::default();
    let mut burst_start = next_probe;
    let mut burst_sent = 0u8;
    let mut best: Option<(super::clock::Measurement, i64)> = None;
    // Publish unavailable before the server can start playback. Clock
    // synchronization will publish available once timestamps are usable.
    send_player_state(
        socket,
        &mut transport,
        legacy,
        false,
        volume,
        muted,
        plaintext,
        frame,
    )
    .await?;
    loop {
        if last_active == Some(true)
            && let Either::First(()) = select(Timer::at(next_probe), socket.wait_read_ready()).await
        {
            if !socket.may_recv() {
                return Err("server closed TCP");
            }
            if burst_sent == 8 {
                if let Some((sample, arrived)) = best.take() {
                    clock.update(sample, arrived);
                    let synchronized = clock.client_time(arrived + sample.offset).is_some();
                    if !state_sent || (!ready && synchronized) {
                        send_player_state(
                            socket,
                            &mut transport,
                            legacy,
                            synchronized,
                            volume,
                            muted,
                            plaintext,
                            frame,
                        )
                        .await?;
                        state_sent = true;
                        ready = synchronized;
                        if ready {
                            esp_println::println!(
                                "Sendspin player ready (clock sample uncertainty {} us)",
                                sample.uncertainty
                            );
                        }
                    }
                }
                let period = if clock.samples < 10 { 1 } else { 8 };
                next_probe = burst_start + Duration::from_secs(period);
                burst_sent = 0;
                outstanding = None;
                continue;
            }
            if burst_sent == 0 {
                burst_start = Instant::now();
            }
            let sent = Instant::now().as_micros() as i64;
            send_json(
                socket,
                &mut transport,
                &serde_json::json!({
                    "type": "client/time",
                    "payload": { "client_transmitted": sent }
                }),
                plaintext,
                frame,
            )
            .await?;
            outstanding = Some(sent);
            burst_sent += 1;
            next_probe = Instant::now() + Duration::from_millis(250);
            continue;
        }

        // Race the timer against read readiness only. Cancelling a partial frame
        // read would discard bytes and desynchronize the WebSocket stream.
        let len = with_timeout(Duration::from_secs(30), receive(socket, 2, frame))
            .await
            .map_err(|_| "Sendspin receive timed out")??;
        let len = transport
            .read_message(&frame[..len], plaintext)
            .map_err(|_| "decrypt Sendspin message")?;
        let arrived = Instant::now().as_micros() as i64;
        if let [4, timestamp @ ..] = &plaintext[..len] {
            let header = if legacy { 8 } else { 12 };
            if streaming && timestamp.len() >= header {
                let timestamp_us = i64::from_be_bytes(timestamp[..8].try_into().unwrap());
                if let Some(local) = clock.client_time(timestamp_us) {
                    let payload = &mut plaintext[header + 1..len];
                    if chunks == 0 {
                        esp_println::println!(
                            "Sendspin received first PCM chunk ({} bytes, {} us until playback)",
                            payload.len(),
                            local.saturating_sub(arrived)
                        );
                    }
                    chunks = chunks.saturating_add(1);
                    // Little-endian signed 16-bit PCM, with a conservative default volume.
                    let gain = if muted {
                        0
                    } else {
                        u32::from(volume).pow(2) as i32
                    };
                    for sample in payload.chunks_exact_mut(2) {
                        let value = i32::from(i16::from_le_bytes([sample[0], sample[1]]));
                        sample.copy_from_slice(&((value * gain / 10_000) as i16).to_le_bytes());
                    }
                    if !pcm::push(payload, local) {
                        return Err("PCM queue full or invalid PCM chunk");
                    }
                }
            }
            embassy_futures::yield_now().await;
            continue;
        }
        let Some((&0, json)) = plaintext[..len].split_first() else {
            continue;
        };
        let message: MessageType<'_> =
            serde_json::from_slice(json).map_err(|_| "invalid encrypted Sendspin JSON")?;
        match message.kind {
            "server/activate" => {
                let activation: Envelope<'_, Activation<'_>> = decode_json(&plaintext[..len])?;
                let active = activation
                    .payload
                    .active_roles
                    .map_or(last_active.unwrap_or(false), |roles| {
                        roles.contains(&"player@v1")
                    });
                if last_active != Some(active) {
                    last_active = Some(active);
                    ready = false;
                    state_sent = false;
                    streaming = false;
                    pcm::reset();
                    outstanding = None;
                    clock = super::clock::ClockFilter::default();
                    burst_sent = 0;
                    best = None;
                    next_probe = Instant::now();
                    if active {
                        esp_println::println!(
                            "Sendspin server activated the 96 kHz stereo PCM player role"
                        );
                    } else {
                        esp_println::println!("Sendspin connected, waiting for player activation");
                    }
                }
            }
            "server/time" => {
                let reply: Envelope<'_, super::clock::TimeReply> = decode_json(&plaintext[..len])?;
                if let Some(sent) = outstanding
                    && let Some(sample) = reply.payload.measure(sent, arrived)
                {
                    outstanding = None;
                    if best
                        .as_ref()
                        .is_none_or(|(prior, _)| sample.uncertainty < prior.uncertainty)
                    {
                        best = Some((sample, arrived));
                    }
                    next_probe = Instant::now();
                }
            }
            "stream/start" => {
                let start: Envelope<'_, StartPayload<'_>> = decode_json(&plaintext[..len])?;
                if let Some(player) = start.payload.player {
                    pcm::reset();
                    chunks = 0;
                    streaming = last_active == Some(true)
                        && player.codec == "pcm"
                        && player.channels == 2
                        && player.sample_rate == crate::audio::SAMPLE_RATE
                        && player.bit_depth == 16;
                    if streaming {
                        esp_println::println!("Sendspin received 96 kHz stereo PCM stream/start");
                    } else {
                        esp_println::println!("Sendspin rejected unsupported stream/start");
                    }
                }
            }
            "stream/clear" | "stream/end" => {
                let control: Envelope<'_, StreamRoles<'_>> = decode_json(&plaintext[..len])?;
                if control
                    .payload
                    .roles
                    .as_ref()
                    .is_none_or(|roles| roles.contains(&"player") || roles.contains(&"player@v1"))
                {
                    if message.kind == "stream/clear" {
                        pcm::reset();
                    } else {
                        streaming = false;
                        pcm::end();
                        esp_println::println!("Sendspin stream ended after {chunks} PCM chunks");
                    }
                }
            }
            "server/command" => {
                let command: Envelope<'_, CommandPayload<'_>> = decode_json(&plaintext[..len])?;
                if let Some(command) = command.payload.player {
                    match (command.command, command.volume, command.mute) {
                        ("volume", Some(next), _) if next <= 100 => {
                            volume = next;
                            // Save before acknowledging: a failed send must not
                            // discard a volume command already received.
                            *saved_volume = next;
                        }
                        ("mute", _, Some(next)) => muted = next,
                        _ => continue,
                    }
                    send_player_state(
                        socket,
                        &mut transport,
                        legacy,
                        ready,
                        volume,
                        muted,
                        plaintext,
                        frame,
                    )
                    .await?;
                }
            }
            _ => {}
        }
    }
}

#[derive(Deserialize)]
struct PcmFormat<'a> {
    codec: &'a str,
    channels: u8,
    sample_rate: u32,
    bit_depth: u8,
}

#[derive(Deserialize)]
struct StartPayload<'a> {
    #[serde(borrow)]
    player: Option<PcmFormat<'a>>,
}

#[derive(Deserialize)]
struct StreamRoles<'a> {
    #[serde(borrow)]
    roles: Option<self::alloc::vec::Vec<&'a str>>,
}

#[derive(Deserialize)]
struct PlayerCommand<'a> {
    command: &'a str,
    volume: Option<u8>,
    mute: Option<bool>,
}

#[derive(Deserialize)]
struct CommandPayload<'a> {
    #[serde(borrow)]
    player: Option<PlayerCommand<'a>>,
}

async fn send_player_state(
    socket: &mut TcpSocket<'_>,
    transport: &mut TransportState,
    legacy: bool,
    available: bool,
    volume: u8,
    muted: bool,
    plaintext: &mut [u8],
    frame: &mut [u8],
) -> Result<(), &'static str> {
    let mut player = serde_json::json!({
        "static_delay_ms": 0,
        "required_lead_time_ms": 900,
        "min_buffer_ms": 750,
        "volume": volume,
        "muted": muted,
    });
    if !legacy {
        player["supported_commands"] = serde_json::json!(["volume", "mute"]);
    }
    let message = serde_json::json!({
        "type": "client/state",
        "payload": { "available": available, "player": player },
    });
    send_json(socket, transport, &message, plaintext, frame).await
}

async fn send_json(
    socket: &mut TcpSocket<'_>,
    transport: &mut TransportState,
    message: &serde_json::Value,
    plaintext: &mut [u8],
    frame: &mut [u8],
) -> Result<(), &'static str> {
    let json = serde_json::to_vec(message).map_err(|_| "serialize Sendspin JSON")?;
    let len = json.len() + 1;
    plaintext[0] = 0;
    plaintext
        .get_mut(1..len)
        .ok_or("Sendspin JSON too large")?
        .copy_from_slice(&json);
    let len = transport
        .write_message(&plaintext[..len], frame)
        .map_err(|_| "encrypt Sendspin JSON")?;
    with_timeout(
        Duration::from_secs(5),
        send_frame(socket, 2, &mut frame[..len]),
    )
    .await
    .map_err(|_| "Sendspin send timed out")?
}

fn decode_json<'a, T: Deserialize<'a>>(plaintext: &'a [u8]) -> Result<T, &'static str> {
    let Some((&0, json)) = plaintext.split_first() else {
        return Err("expected encrypted JSON message");
    };
    serde_json::from_slice(json).map_err(|_| "invalid encrypted Sendspin JSON")
}

async fn send_frame(
    socket: &mut TcpSocket<'_>,
    opcode: u8,
    data: &mut [u8],
) -> Result<(), &'static str> {
    let mut header = [0; 8];
    header[0] = 0x80 | opcode;
    let mask_offset = if data.len() < 126 {
        header[1] = 0x80 | data.len() as u8;
        2
    } else {
        header[1] = 0xfe;
        header[2..4].copy_from_slice(&(data.len() as u16).to_be_bytes());
        4
    };
    let mask = &mut header[mask_offset..mask_offset + 4];
    Rng::new().read(mask);
    for (index, byte) in data.iter_mut().enumerate() {
        *byte ^= mask[index % 4];
    }
    socket
        .write_all(&header[..mask_offset + 4])
        .await
        .map_err(|_| "WebSocket frame write")?;
    socket
        .write_all(data)
        .await
        .map_err(|_| "WebSocket payload write")
}

async fn receive(
    socket: &mut TcpSocket<'_>,
    opcode: u8,
    data: &mut [u8],
) -> Result<usize, &'static str> {
    loop {
        let mut head = [0; 2];
        socket
            .read_exact(&mut head)
            .await
            .map_err(|_| "WebSocket frame read")?;
        if head[0] & 0xf0 != 0x80 || head[1] & 0x80 != 0 {
            return Err("unsupported WebSocket frame flags");
        }
        let kind = head[0] & 0x0f;
        if matches!(kind, 8..=10) {
            let mut control = [0; 125];
            let payload = control
                .get_mut(..usize::from(head[1]))
                .ok_or("invalid WebSocket control length")?;
            socket
                .read_exact(payload)
                .await
                .map_err(|_| "WebSocket control read")?;
            match kind {
                8 => {
                    if let [first, second, reason @ ..] = payload {
                        esp_println::println!(
                            "Sendspin WebSocket closed with code {} ({})",
                            u16::from_be_bytes([*first, *second]),
                            core::str::from_utf8(reason).unwrap_or("")
                        );
                    }
                    let _ = send_frame(socket, 8, payload).await;
                    return Err("server closed WebSocket");
                }
                9 => send_frame(socket, 10, payload).await?,
                _ => {}
            }
            continue;
        }
        if kind != opcode {
            esp_println::println!("Unexpected Sendspin WebSocket opcode {}", kind);
            return Err("unexpected WebSocket frame");
        }
        let len = match head[1] {
            126 => {
                socket
                    .read_exact(&mut head)
                    .await
                    .map_err(|_| "WebSocket length read")?;
                usize::from(u16::from_be_bytes(head))
            }
            127 => return Err("WebSocket handshake frame too large"),
            len => usize::from(len),
        };
        let data = data
            .get_mut(..len)
            .ok_or("WebSocket handshake frame too large")?;
        socket
            .read_exact(data)
            .await
            .map_err(|_| "WebSocket payload read")?;
        return Ok(len);
    }
}
