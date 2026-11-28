use serde::Deserialize;

#[derive(Deserialize)]
pub(super) struct TimeReply {
    pub client_transmitted: i64,
    pub server_received: i64,
    pub server_transmitted: i64,
}

#[derive(Clone, Copy)]
pub(super) struct Measurement {
    pub offset: i64,
    pub uncertainty: i64,
}

impl TimeReply {
    pub fn measure(&self, sent: i64, received: i64) -> Option<Measurement> {
        if self.client_transmitted != sent || received <= sent {
            return None;
        }
        let server_elapsed = self.server_transmitted.checked_sub(self.server_received)?;
        let round_trip = (received - sent).checked_sub(server_elapsed)?;
        // Reject reversed timestamps and replies too slow to seed an audio clock.
        if server_elapsed < 0 || !(2..=100_000).contains(&round_trip) {
            return None;
        }
        let server_midpoint = self.server_received.checked_add(server_elapsed / 2)?;
        let client_midpoint = sent + (received - sent) / 2;
        Some(Measurement {
            offset: server_midpoint.checked_sub(client_midpoint)?,
            uncertainty: round_trip / 2,
        })
    }
}

/// Two-state offset and drift filter from the Sendspin synchronization spec.
/// Covariances are in microseconds squared, microseconds, and drift squared.
#[derive(Default)]
pub(super) struct ClockFilter {
    offset: f64,
    drift: f64,
    offset_variance: f64,
    covariance: f64,
    drift_variance: f64,
    last_update: i64,
    pub samples: u32,
}

impl ClockFilter {
    pub fn update(&mut self, sample: Measurement, received: i64) {
        let elapsed = (received - self.last_update) as f64;
        if elapsed <= 0.0 {
            return;
        }
        self.last_update = received;

        let measured = sample.offset as f64;
        let sigma = sample.uncertainty as f64 / 2.0;
        let variance = sigma * sigma;
        match self.samples {
            0 => {
                self.offset = measured;
                self.offset_variance = variance;
            }
            1 => {
                self.drift = (measured - self.offset) / elapsed;
                self.drift_variance = (self.offset_variance + variance) / (elapsed * elapsed);
                self.offset = measured;
                self.offset_variance = variance;
            }
            _ => {
                let prediction = self.offset + elapsed * self.drift;
                let residual = measured - prediction;
                let forget =
                    if self.samples >= 100 && residual.abs() > 3.0 * sample.uncertainty as f64 {
                        4.0
                    } else {
                        1.0
                    };
                let offset_variance = forget
                    * (self.offset_variance
                        + elapsed * (2.0 * self.covariance + elapsed * self.drift_variance));
                let covariance = forget * (self.covariance + elapsed * self.drift_variance);
                let drift_variance = forget * (self.drift_variance + elapsed * 1e-22);
                let innovation_variance = offset_variance + variance;
                let offset_gain = offset_variance / innovation_variance;
                let drift_gain = covariance / innovation_variance;
                self.offset = prediction + offset_gain * residual;
                self.drift += drift_gain * residual;
                self.offset_variance = offset_variance * (1.0 - offset_gain);
                self.covariance = covariance * (1.0 - offset_gain);
                self.drift_variance = (drift_variance - drift_gain * covariance).max(0.0);
            }
        }
        self.samples = self.samples.saturating_add(1);
    }

    /// Convert a server timestamp to local monotonic time after the filter settles.
    pub fn client_time(&self, server_time: i64) -> Option<i64> {
        if self.samples < 10 {
            return None;
        }
        let drift = if self.drift * self.drift > 4.0 * self.drift_variance {
            self.drift
        } else {
            0.0
        };
        let relative = server_time.checked_sub(self.last_update)? as f64 - self.offset;
        let local = self.last_update as f64 + relative / (1.0 + drift);
        local
            .is_finite()
            .then(|| (local + 0.5_f64.copysign(local)) as i64)
    }
}
