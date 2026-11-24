//! Application composition: connect camera events to the preview and display.
use crate::{gt911, perf, preview::Preview, slint_platform::Display, slint_ui::Ui, uvc};

pub struct App {
    ui: Ui,
    display: Display,
    preview: Option<Preview>,
    frames: u32,
    performance: perf::Sampler,
    counter: i32,
}

impl App {
    pub fn new(preview_enabled: bool) -> Result<Self, slint::platform::PlatformError> {
        let (display, screen) = Display::initialize()?;
        let ui = Ui::new(screen, preview_enabled);
        let counter = ui.count();
        Ok(Self {
            ui,
            display,
            preview: preview_enabled.then(Preview::new),
            frames: 0,
            performance: perf::Sampler::new(0),
            counter,
        })
    }

    pub fn update(&mut self) -> bool {
        // Bound input work so a burst cannot delay rendering indefinitely.
        for _ in 0..gt911::EVENTS.capacity() {
            let Ok(sample) = gt911::EVENTS.try_receive() else {
                break;
            };
            self.display.touch(sample);
        }
        let count = self.ui.count();
        if count != self.counter {
            self.counter = count;
            esp_println::println!("Slint touch counter: {count}");
        }
        self.performance.report(self.frames);
        // When no preview is attached, leave camera events for another consumer.
        match &mut self.preview {
            Some(preview) => preview.update(&mut self.ui, &self.display, uvc::take_frame()),
            None => false,
        }
    }

    pub async fn draw(&mut self, framebuffer: &mut [u8]) -> bool {
        let redrawn = self.display.draw(
            framebuffer,
            self.preview.as_mut().map(|preview| &mut preview.jpeg),
        );
        if redrawn {
            if let (Some(preview), Some((x, y))) =
                (&mut self.preview, self.ui.preview_overlay_position())
            {
                preview.draw_overlay(framebuffer, x, y).await;
            }
        }
        redrawn
    }

    pub fn presented(&mut self, camera_updated: bool) {
        if camera_updated {
            self.frames = self.frames.saturating_add(1);
            if matches!(self.frames, 1 | 5 | 30) || self.frames.is_multiple_of(300) {
                esp_println::println!("USB camera: rendered {} preview frame(s)", self.frames);
            }
        }
    }
}
