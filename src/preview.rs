use crate::{jpeg_hw, slint_ui::Ui};
use slint::{Image, Rgb8Pixel, SharedPixelBuffer};

/// Decodes camera frames and draws the hardware-accelerated preview.
pub struct Preview {
    pub(crate) jpeg: jpeg_hw::Decoder,
    hardware_decode_reported: bool,
    hardware_error_reported: bool,
    pub(crate) ppa_available: bool,
    pub(crate) ppa_reported: bool,
}

impl Preview {
    pub fn new() -> Self {
        Self {
            jpeg: jpeg_hw::Decoder::new(),
            hardware_decode_reported: false,
            hardware_error_reported: false,
            ppa_available: true,
            ppa_reported: false,
        }
    }
}

impl Preview {
    pub fn update(
        &mut self,
        ui: &mut Ui,
        display: &crate::slint_platform::Display,
        frame: Option<crate::uvc::CameraEvent>,
    ) -> bool {
        use zune_jpeg::{
            JpegDecoder,
            zune_core::{bytestream::ZCursor, colorspace::ColorSpace, options::DecoderOptions},
        };

        let Some(frame) = frame else {
            return false;
        };
        let frame = match frame {
            crate::uvc::CameraEvent::Frame(frame) => frame,
            crate::uvc::CameraEvent::Disconnected => {
                ui.demo.set_camera_overlay(false);
                ui.demo.set_camera_ready(false);
                ui.demo.set_camera_frame(Image::default());
                return false;
            }
        };
        match self.jpeg.decode(&frame) {
            Ok(_) => {
                if !self.hardware_decode_reported {
                    esp_println::println!("USB camera: hardware JPEG decode completed");
                    self.hardware_decode_reported = true;
                }
                ui.demo.set_camera_overlay(true);
                ui.demo.set_camera_ready(true);
                display.request_redraw();
                return true;
            }
            Err(error) if !self.hardware_error_reported => {
                esp_println::println!(
                    "USB camera: hardware decode failed, using software: {error}"
                );
                self.hardware_error_reported = true;
            }
            Err(_) => (),
        }
        let options = DecoderOptions::default()
            .set_max_width(crate::uvc::WIDTH as usize)
            .set_max_height(crate::uvc::HEIGHT as usize)
            .jpeg_set_out_colorspace(ColorSpace::RGB);
        let mut decoder = JpegDecoder::new_with_options(ZCursor::new(frame.as_slice()), options);
        let mut pixels = SharedPixelBuffer::<Rgb8Pixel>::new(
            u32::from(crate::uvc::WIDTH),
            u32::from(crate::uvc::HEIGHT),
        );
        match decoder
            .decode_headers()
            .and_then(|()| decoder.decode_into(pixels.make_mut_bytes()))
        {
            Ok(())
                if decoder.dimensions()
                    == Some((crate::uvc::WIDTH as usize, crate::uvc::HEIGHT as usize)) =>
            {
                ui.demo.set_camera_overlay(false);
                ui.demo.set_camera_frame(Image::from_rgb8(pixels));
                ui.demo.set_camera_ready(true);
                true
            }
            Err(error) => {
                esp_println::println!("USB camera JPEG decode failed: {error:?}");
                false
            }
            _ => false,
        }
    }
    pub async fn draw_overlay(&mut self, framebuffer: &mut [u8], x: usize, y: usize) {
        if self.ppa_available {
            self.ppa_available = self.jpeg.draw_preview(framebuffer, x, y).await;
            if self.ppa_available && !self.ppa_reported {
                esp_println::println!("USB camera: PPA preview scaling completed at {x},{y}");
                self.ppa_reported = true;
            }
        }
        if !self.ppa_available {
            self.jpeg.draw_preview_software(framebuffer, x, y);
        }
    }
}
