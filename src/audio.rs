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
const SAMPLE_RATE: u32 = 48_000;
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
        codec.volume_set(bus, 25, None)?;
        codec.mute(bus, true)?;

        let mut id = [0];
        bus.write_read(CODEC_ADDRESS, &[0xfd], &mut id)
            .map_err(Error::I2c)?;
        esp_println::println!(
            "Audio: ES8311 ID {:02x}, {SAMPLE_RATE} Hz, 16-bit I2S",
            id[0],
        );
        Ok(Self { tx, rx, amplifier })
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
