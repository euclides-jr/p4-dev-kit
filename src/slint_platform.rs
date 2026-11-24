//! Slint platform setup for the board's software-rendered display.
use crate::{
    slint_ui::DemoScreen,
    waveshare_5dsi_touch_a::{HEIGHT, WIDTH},
};
use alloc::{boxed::Box, rc::Rc};
use core::time::Duration;
use slint::{
    Brush, ComponentHandle, LogicalPosition, PhysicalSize,
    platform::{
        Platform, PlatformError, PointerEventButton, WindowAdapter, WindowEvent,
        software_renderer::{
            DrawRectangleArgs, MinimalSoftwareWindow, PhysicalRegion, RepaintBufferType,
            Rgb565Pixel, TargetPixel, TargetPixelBuffer,
        },
    },
};

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

/// Board-specific window, input translation and framebuffer rendering.
pub struct Display {
    window: Rc<MinimalSoftwareWindow>,
    touch: Option<LogicalPosition>,
    fill_available: bool,
    fill_reported: bool,
}

impl Display {
    pub fn initialize() -> Result<(Self, DemoScreen), PlatformError> {
        let window = MinimalSoftwareWindow::new(RepaintBufferType::SwappedBuffers);
        slint::platform::set_platform(Box::new(DisplayPlatform(window.clone())))
            .map_err(|_| PlatformError::Other("Slint platform already initialized".into()))?;
        let screen = DemoScreen::new()?;
        window.set_size(PhysicalSize::new(WIDTH as u32, HEIGHT as u32));
        screen.show()?;
        Ok((
            Self {
                window,
                touch: None,
                fill_available: true,
                fill_reported: false,
            },
            screen,
        ))
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

    pub fn draw(
        &mut self,
        framebuffer: &mut [u8],
        ppa: Option<&mut crate::jpeg_hw::Decoder>,
    ) -> bool {
        slint::platform::update_timers_and_animations();
        let mut target = DisplayTarget {
            frame: framebuffer,
            ppa,
            fill_available: &mut self.fill_available,
            fill_reported: &mut self.fill_reported,
        };
        self.window.draw_if_needed(|renderer| {
            renderer.render_into_buffer(&mut target);
        })
    }

    pub fn request_redraw(&self) {
        use slint::platform::WindowAdapter;
        self.window.window().request_redraw();
    }
}

struct DisplayTarget<'a> {
    frame: &'a mut [u8],
    ppa: Option<&'a mut crate::jpeg_hw::Decoder>,
    fill_available: &'a mut bool,
    fill_reported: &'a mut bool,
}

impl DisplayTarget<'_> {
    fn fill(&mut self, brush: &Brush, region: &PhysicalRegion, bounds: [i32; 4]) -> bool {
        let Brush::SolidColor(color) = brush else {
            return false;
        };
        if !*self.fill_available || color.alpha() != 255 {
            return false;
        }
        let Some(ppa) = self.ppa.as_deref_mut() else {
            return false;
        };
        let [left, top, right, bottom] = bounds;
        let packed = Rgb565Pixel::from_rgb(color.red(), color.green(), color.blue()).0;
        for (origin, size) in region.iter() {
            let x = origin.x.max(left);
            let y = origin.y.max(top);
            let width = (origin.x + size.width as i32).min(right) - x;
            let height = (origin.y + size.height as i32).min(bottom) - y;
            if width <= 0 || height <= 0 {
                continue;
            }
            // Cache synchronization and DMA setup can outweigh a small CPU fill.
            if width * height < 32_768 {
                return false;
            }
            let start = esp_hal::time::Instant::now();
            if !ppa.fill_rectangle(
                self.frame,
                x as usize,
                y as usize,
                width as usize,
                height as usize,
                packed,
            ) {
                *self.fill_available = false;
                esp_println::println!("Slint PPA fill failed; using CPU fills");
                return false;
            }
            if !*self.fill_reported {
                esp_println::println!(
                    "Slint PPA fill: {width}x{height} RGB565 in {} ms",
                    start.elapsed().as_millis(),
                );
                *self.fill_reported = true;
            }
        }
        true
    }
}

impl TargetPixelBuffer for DisplayTarget<'_> {
    type TargetPixel = Rgb565Pixel;

    fn line_slice(&mut self, line: usize) -> &mut [Rgb565Pixel] {
        let offset = line * WIDTH * 2;
        let pixels = &mut self.frame[offset..offset + WIDTH * 2];
        // SAFETY: The framebuffer and its line stride are aligned to u16.
        // Rgb565Pixel is transparent over u16 and accepts all bit patterns.
        unsafe { core::slice::from_raw_parts_mut(pixels.as_mut_ptr().cast(), WIDTH) }
    }

    fn num_lines(&self) -> usize {
        HEIGHT
    }

    fn fill_background(&mut self, brush: &Brush, region: &PhysicalRegion) -> bool {
        self.fill(brush, region, [0, 0, WIDTH as i32, HEIGHT as i32])
    }

    fn draw_rectangle(&mut self, args: &DrawRectangleArgs, clip: &PhysicalRegion) -> bool {
        let positions = [args.x, args.y, args.x + args.width, args.y + args.height];
        if args.alpha != 255
            || args.border_width != 0.0
            || args.top_left_radius != 0.0
            || args.top_right_radius != 0.0
            || args.bottom_left_radius != 0.0
            || args.bottom_right_radius != 0.0
            || positions.iter().any(|&pos| pos as i32 as f32 != pos)
        {
            return false;
        }
        self.fill(&args.background, clip, positions.map(|pos| pos as i32))
    }
}
