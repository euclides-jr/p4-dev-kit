use esp_hal::{
    Blocking,
    i2c::master::{Error, I2c},
};

pub struct Gt911(u8);

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

    pub fn poll(&self, i2c: &mut I2c<'_, Blocking>) -> Result<Sample, Error> {
        let mut status = [0];
        i2c.write_read(self.0, &[0x81, 0x4e], &mut status)?;
        if status[0] & 0x80 == 0 {
            return Ok(Sample::Unchanged);
        }

        let mut point = [0; 5];
        let count = status[0] & 0x0f;
        let result = if (1..=5).contains(&count) {
            i2c.write_read(self.0, &[0x81, 0x4f], &mut point)
                .map(|()| Sample::Pressed {
                    x: u16::from_le_bytes([point[1], point[2]]),
                    y: u16::from_le_bytes([point[3], point[4]]),
                })
        } else {
            Ok(Sample::Released)
        };
        i2c.write(self.0, &[0x81, 0x4e, 0])?;
        result
    }
}
