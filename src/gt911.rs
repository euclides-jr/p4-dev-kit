use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use embassy_time::Timer;
use esp_hal::{
    Async, Blocking,
    i2c::master::{Error, I2c},
};

pub struct Gt911(u8);

pub static EVENTS: Channel<CriticalSectionRawMutex, Sample, 16> = Channel::new();

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Sample {
    Unchanged,
    Released,
    Pressed { x: u16, y: u16 },
}

impl Gt911 {
    pub fn detect(i2c: &mut I2c<'_, Blocking>) -> Option<Self> {
        [0x5d, 0x14].into_iter().find_map(|address| {
            let mut id = [0; 4];
            i2c.write_read(address, &[0x81, 0x40], &mut id).ok()?;
            id.starts_with(b"911").then_some(Self(address))
        })
    }

    pub fn check_status(&self, i2c: &mut I2c<'_, Blocking>) -> Result<(), Error> {
        let mut before = [0];
        let mut after = [0];
        i2c.write_read(self.0, &[0x81, 0x4e], &mut before)?;
        i2c.write(self.0, &[0x81, 0x4e, 0])?;
        i2c.write_read(self.0, &[0x81, 0x4e], &mut after)?;
        esp_println::println!(
            "GT911 status before/after ACK: {:#04x} / {:#04x}",
            before[0],
            after[0]
        );
        Ok(())
    }

    async fn detect_async(i2c: &mut I2c<'_, Async>) -> Option<Self> {
        for address in [0x5d, 0x14] {
            let mut id = [0; 4];
            if i2c
                .write_read_async(address, &[0x81, 0x40], &mut id)
                .await
                .is_ok()
                && id.starts_with(b"911")
            {
                return Some(Self(address));
            }
        }
        None
    }

    async fn poll(&self, i2c: &mut I2c<'_, Async>) -> Result<Sample, Error> {
        let mut status = [0];
        i2c.write_read_async(self.0, &[0x81, 0x4e], &mut status)
            .await?;
        if status[0] & 0x80 == 0 {
            return Ok(Sample::Unchanged);
        }

        let mut points = [0; 5 * 8];
        let count = status[0] & 0x0f;
        let result = if (1..=5).contains(&count) {
            i2c.write_read_async(self.0, &[0x81, 0x4f], &mut points[..usize::from(count) * 8])
                .await
                .map(|()| Sample::Pressed {
                    x: u16::from_le_bytes([points[1], points[2]]),
                    y: u16::from_le_bytes([points[3], points[4]]),
                })
        } else {
            Ok(Sample::Released)
        };
        i2c.write_async(self.0, &[0x81, 0x4e, 0]).await?;
        result
    }
}

#[embassy_executor::task]
pub async fn run(mut touch: Option<Gt911>, i2c: I2c<'static, Blocking>, mut backlight_setup: bool) {
    let mut i2c = i2c.into_async();
    let mut backlight_ready = false;
    let mut last = Sample::Released;
    let mut previous_error = None;
    let mut retry_at = embassy_time::Instant::now();
    loop {
        if embassy_time::Instant::now() >= retry_at {
            if !backlight_ready {
                let result = async {
                    if !backlight_setup {
                        i2c.write_async(0x45, &[0x95, 0x11]).await?;
                        i2c.write_async(0x45, &[0x95, 0x17]).await?;
                        i2c.write_async(0x45, &[0x96, 0x00]).await?;
                        backlight_setup = true;
                        Timer::after_millis(100).await;
                    }
                    i2c.write_async(0x45, &[0x96, 0xff]).await
                }
                .await;
                match result {
                    Ok(()) => {
                        backlight_ready = true;
                        esp_println::println!("Backlight enabled");
                    }
                    Err(error) => {
                        esp_println::println!("Backlight enable failed: {error:?}");
                    }
                }
            }
            if touch.is_none() {
                touch = Gt911::detect_async(&mut i2c).await;
                if touch.is_some() {
                    esp_println::println!("GT911 touchscreen detected after retry");
                }
            }
            retry_at = embassy_time::Instant::now() + embassy_time::Duration::from_secs(1);
        }
        let Some(device) = &touch else {
            Timer::after_millis(100).await;
            continue;
        };
        match device.poll(&mut i2c).await {
            Ok(sample) => {
                previous_error = None;
                if sample != Sample::Unchanged && sample != last {
                    EVENTS.send(sample).await;
                    last = sample;
                }
            }
            Err(error) => {
                if previous_error != Some(error) {
                    esp_println::println!("GT911 polling failed: {error:?}");
                    previous_error = Some(error);
                }
                if last != Sample::Released {
                    EVENTS.send(Sample::Released).await;
                    last = Sample::Released;
                }
            }
        }
        Timer::after_millis(8).await;
    }
}
