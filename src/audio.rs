use embassy_time::{Duration, Timer, with_timeout};
use es8311::{ClockConfig, Error, Es8311, Resolution};
use esp_hal::{
    Async, Blocking,
    delay::Delay,
    gpio::Output,
    i2c::master::{Error as I2cError, I2c},
    i2s::master::{Channels, DataFormat, I2sRx, I2sTx, TdmConfig},
    time::Rate,
};

const CODEC_ADDRESS: u8 = 0x18;
pub const SAMPLE_RATE: u32 = 96_000;
const MCLK_HZ: u32 = SAMPLE_RATE * 256;

pub fn i2s_config() -> TdmConfig {
    TdmConfig::new_tdm_philips()
        .with_sample_rate(Rate::from_hz(SAMPLE_RATE))
        .with_data_format(DataFormat::Data16Channel16)
        .with_channels(Channels::STEREO)
        .with_signal_loopback(true)
}

pub struct Audio {
    tx: I2sTx<'static, Async>,
    rx: I2sRx<'static, Async>,
    amplifier: Output<'static>,
}

impl Audio {
    pub fn new(
        bus: &mut I2c<'_, Blocking>,
        tx: I2sTx<'static, Async>,
        rx: I2sRx<'static, Async>,
        mut amplifier: Output<'static>,
    ) -> Result<Self, Error<I2cError>> {
        amplifier.set_low();
        // Waveshare starts I2S before codec initialization. Keep zero samples
        // and clocks present through reset and analogue output power-up.
        let mut startup = esp_hal::dma_tx_buffer!(32768).expect("codec startup DMA buffer");
        startup.as_mut_slice().fill(0);
        let clocks = match tx.write(startup) {
            Ok(transfer) => transfer,
            Err((error, _, _)) => panic!("codec startup clocks failed: {error:?}"),
        };
        let configured = Self::configure_codec(bus);
        let (tx, _) = clocks.stop();
        configured?;
        Ok(Self { tx, rx, amplifier })
    }

    fn configure_codec(bus: &mut I2c<'_, Blocking>) -> Result<(), Error<I2cError>> {
        let codec = Es8311::new(CODEC_ADDRESS);
        codec.init(
            bus,
            &ClockConfig {
                mclk_inverted: false,
                sclk_inverted: false,
                mclk_from_mclk_pin: true,
                mclk_frequency: MCLK_HZ,
                sample_frequency: SAMPLE_RATE,
            },
            Resolution::Bits16,
            Resolution::Bits16,
            &mut Delay::new(),
        )?;
        codec.microphone_config(bus, false)?;
        // Match the esp-player workaround. The crate's shifted gain writes ADC mute bits.
        bus.write(CODEC_ADDRESS, &[0x16, 4]).map_err(Error::I2c)?;
        // ES8311 DAC volume is -95.5 + register * 0.5 dB. Keep unity
        // gain here; Sendspin applies volume and mute to PCM samples.
        bus.write(CODEC_ADDRESS, &[0x32, 0xbf])
            .map_err(Error::I2c)?;
        codec.mute(bus, false)?;

        let mut id = [0];
        bus.write_read(CODEC_ADDRESS, &[0xfd], &mut id)
            .map_err(Error::I2c)?;
        esp_println::println!(
            "Audio: ES8311 ID {:02x}, {SAMPLE_RATE} Hz, 16-bit I2S",
            id[0],
        );
        // Allow the headphone output bias to settle while I2S clocks run.
        Delay::new().delay_millis(10);
        Ok(())
    }

    /// Exercise both DMA directions while the speaker is muted. Report only
    /// sample levels, then discard the short microphone buffer.
    pub async fn check_dma(self) -> Self {
        let Self { tx, rx, amplifier } = self;
        let mut tx_buffer = esp_hal::dma_tx_buffer!(8192).expect("fixed TX DMA buffer");
        tx_buffer.as_mut_slice().fill(0);
        let rx_buffer = esp_hal::dma_rx_buffer!(4096).expect("fixed RX DMA buffer");

        Timer::after_millis(150).await;
        let mut input = match rx.read(rx_buffer) {
            Ok(transfer) => transfer,
            Err((error, rx, _)) => {
                esp_println::println!("Audio RX could not start: {error:?}");
                return Self { tx, rx, amplifier };
            }
        };
        let mut output = match tx.write(tx_buffer) {
            Ok(transfer) => transfer,
            Err((error, tx, _)) => {
                esp_println::println!("Audio TX could not start: {error:?}");
                let (rx, _) = input.stop();
                return Self { tx, rx, amplifier };
            }
        };

        let timeout = Duration::from_millis(200);
        let rx_result = with_timeout(timeout, input.wait_for_done_async()).await;
        let (rx, input) = input.stop();
        let tx_result = with_timeout(timeout, output.wait_for_done_async()).await;
        let (tx, _) = output.stop();

        match rx_result {
            Ok(Ok(())) => {
                let count = input.number_of_received_bytes();
                let mut peak = [0; 2];
                for pcm in input.received_data().flat_map(|data| data.chunks_exact(4)) {
                    for (channel, sample) in peak.iter_mut().zip(pcm.chunks_exact(2)) {
                        *channel = (*channel)
                            .max(i16::from_le_bytes([sample[0], sample[1]]).unsigned_abs());
                    }
                }
                esp_println::println!(
                    "Audio RX: {count} PCM bytes, peak L/R {}/{}",
                    peak[0],
                    peak[1]
                );
            }
            error => esp_println::println!("Audio RX DMA check failed: {error:?}"),
        }
        match tx_result {
            Ok(Ok(())) => esp_println::println!("Audio TX: silent DMA transfer completed"),
            error => esp_println::println!("Audio TX DMA check failed: {error:?}"),
        }
        Self { tx, rx, amplifier }
    }
}

#[embassy_executor::task]
pub async fn playback(audio: Audio) {
    use crate::sendspin::pcm;
    // Keep about 10 ms per descriptor at 96 kHz (below the 4092-byte DMA limit).
    const CHUNK: usize = 3840;
    const SIZE: usize = CHUNK * 8;
    let Audio {
        mut tx,
        rx: _,
        mut amplifier,
    } = audio;
    let mut buffer = esp_hal::dma_tx_stream_buffer!(SIZE, CHUNK);
    loop {
        let Some(status) = pcm::status() else {
            Timer::after_millis(10).await;
            continue;
        };
        let now = embassy_time::Instant::now().as_micros() as i64;
        if status.len == 0 || status.first_time > now || (status.len < SIZE && !status.ended) {
            Timer::after_millis(2).await;
            continue;
        }
        pcm::discard_before(now);
        let Some(ready) = pcm::status() else { continue };
        if ready.len == 0 {
            Timer::after_millis(2).await;
            continue;
        }
        let (descriptors, data) = buffer.split();
        buffer = esp_hal::dma::DmaTxStreamBuf::new(descriptors, data).unwrap();
        let mut primed = 0;
        let mut peak = 0u16;
        buffer.push_with(|target| {
            let n = pcm::pop(target);
            primed += n;
            for sample in target[..n].chunks_exact(2) {
                peak = peak.max(i16::from_le_bytes([sample[0], sample[1]]).unsigned_abs());
            }
            target[n..].fill(0);
            target.len()
        });
        let mut transfer = match tx.write(buffer) {
            Ok(transfer) => transfer,
            Err((error, next_tx, next_buffer)) => {
                esp_println::println!("Sendspin DMA start failed: {error:?}");
                tx = next_tx;
                buffer = next_buffer;
                Timer::after_millis(10).await;
                continue;
            }
        };
        amplifier.set_high();
        esp_println::println!(
            "Sendspin playback started: {primed} PCM bytes, peak {peak}, amplifier enabled"
        );
        let mut last_refill = embassy_time::Instant::now();
        let mut max_refill_gap_us = 0;
        let mut min_buffered_bytes = SIZE;
        loop {
            let now = embassy_time::Instant::now();
            let refill_gap_us = (now - last_refill).as_micros();
            max_refill_gap_us = max_refill_gap_us.max(refill_gap_us);
            last_refill = now;
            let Some(status) = pcm::status() else { break };
            if transfer.is_done() || status.generation != ready.generation {
                esp_println::println!(
                    "Sendspin playback stopped: DMA done={}, generation changed={}, queued={}, refill gap={} us, max gap={} us, min buffered={} bytes",
                    transfer.is_done(),
                    status.generation != ready.generation,
                    status.len,
                    refill_gap_us,
                    max_refill_gap_us,
                    min_buffered_bytes
                );
                break;
            }
            let available_bytes = transfer.available_bytes();
            min_buffered_bytes = min_buffered_bytes.min(SIZE.saturating_sub(available_bytes));
            if status.ended && status.len == 0 && available_bytes >= SIZE - CHUNK {
                break;
            }
            let draining = available_bytes >= SIZE - 2 * CHUNK;
            transfer.push_with(|target| {
                let available = target.len().min(status.len);
                let count = if (status.ended || draining) && available != 0 {
                    available.next_multiple_of(CHUNK).min(target.len())
                } else {
                    available / CHUNK * CHUNK
                };
                if count != 0 {
                    let n = pcm::pop(&mut target[..count]);
                    target[n..count].fill(0);
                    count
                } else if !status.ended && draining {
                    // Keep DMA alive through a temporary network gap, as in
                    // esp-player. Publish only one descriptor of silence.
                    let silence = target.len().min(CHUNK);
                    target[..silence].fill(0);
                    silence
                } else {
                    0
                }
            });
            Timer::after_millis(2).await;
        }
        amplifier.set_low();
        (tx, buffer) = transfer.stop();
    }
}
