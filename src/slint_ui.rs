use alloc::rc::Rc;
use core::time::Duration;

use slint::{
    ComponentHandle, LogicalPosition, PhysicalSize,
    platform::{
        Platform, PlatformError, PointerEventButton, WindowAdapter, WindowEvent,
        software_renderer::{MinimalSoftwareWindow, RepaintBufferType, Rgb565Pixel},
    },
};

use crate::waveshare_5dsi_touch_a::{HEIGHT, WIDTH};

slint::include_modules!();

struct DisplayPlatform(Rc<MinimalSoftwareWindow>);

impl Platform for DisplayPlatform {
    fn create_window_adapter(&self) -> Result<Rc<dyn WindowAdapter>, PlatformError> {
        Ok(self.0.clone())
    }

    fn duration_since_start(&self) -> Duration {
        Duration::from_micros(
            esp_hal::time::Instant::now()
                .duration_since_epoch()
                .as_micros(),
        )
    }
}

pub struct Ui {
    window: Rc<MinimalSoftwareWindow>,
    _demo: DemoScreen,
    touch: Option<LogicalPosition>,
}

impl Ui {
    pub fn new() -> Result<Self, PlatformError> {
        // Slint is used only by the main task.
        let window = MinimalSoftwareWindow::new(RepaintBufferType::NewBuffer);
        slint::platform::set_platform(alloc::boxed::Box::new(DisplayPlatform(window.clone())))
            .map_err(|_| PlatformError::Other("Slint platform already initialized".into()))?;
        let demo = DemoScreen::new()?;
        window.set_size(PhysicalSize::new(WIDTH as u32, HEIGHT as u32));
        demo.show()?;
        Ok(Self {
            window,
            _demo: demo,
            touch: None,
        })
    }

    pub fn touch(&mut self, sample: crate::gt911::Sample) {
        use crate::gt911::Sample;
        let button = PointerEventButton::Left;
        let event = match sample {
            Sample::Unchanged => return,
            Sample::Released => {
                let Some(position) = self.touch.take() else {
                    return;
                };
                self.window
                    .dispatch_event(WindowEvent::PointerReleased { position, button });
                WindowEvent::PointerExited
            }
            Sample::Pressed { x, y } => {
                let position = LogicalPosition::new(
                    x.min(WIDTH as u16 - 1) as f32,
                    y.min(HEIGHT as u16 - 1) as f32,
                );
                match self.touch.replace(position) {
                    None => WindowEvent::PointerPressed { position, button },
                    Some(previous) if previous == position => return,
                    Some(_) => WindowEvent::PointerMoved { position },
                }
            }
        };
        self.window.dispatch_event(event);
    }

    pub fn draw(&self, framebuffer: &mut [u8]) -> bool {
        slint::platform::update_timers_and_animations();

        // SAFETY: Framebuffer alignment covers Rgb565Pixel (a transparent u16).
        // Each pair of initialized bytes is a valid pixel in native endianness.
        let pixels = unsafe {
            core::slice::from_raw_parts_mut(
                framebuffer.as_mut_ptr().cast::<Rgb565Pixel>(),
                WIDTH * HEIGHT,
            )
        };
        self.window.draw_if_needed(|renderer| {
            renderer.render(pixels, WIDTH);
        })
    }
}
