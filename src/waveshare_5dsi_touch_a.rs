//! Native `esp-hal` support for the Waveshare 5-DSI-TOUCH-A (HX8394).
//!
//! The panel timings and DCS init sequence follow the working 720x1280
//! Waveshare esp_lcd_hx8394 2.1.0 C driver used by c-demo.

use core::ptr;

use alloc::boxed::Box;
use esp_hal::{
    delay::Delay,
    mipi_dsi::{
        MipiDsi,
        dbi::Error as DbiError,
        dpi::{ColorFormat, DpiClockSource, DpiConfig, FrameTiming},
    },
};

pub const WIDTH: usize = 720;
pub const HEIGHT: usize = 1280;
pub const FRAMEBUFFER_BYTES: usize = WIDTH * HEIGHT * 2;
pub const LANE_BIT_RATE_MBPS: f32 = 700.0;

/// RGB565 framebuffer aligned for the ESP32-P4 MIPI DSI VDMA engine.
#[repr(C, align(128))]
pub struct Framebuffer(pub [u8; FRAMEBUFFER_BYTES]);

pub const DPI_CONFIG: DpiConfig = DpiConfig {
    virtual_channel: 0,
    // Match the working Waveshare HX8394 C driver configuration.
    pixel_clock_mhz: 58.0,
    dpi_clk_src: DpiClockSource::PllF240m,
    in_color_format: ColorFormat::Rgb565,
    out_color_format: ColorFormat::Rgb565,
    timing: FrameTiming {
        h_active: WIDTH as u32,
        hsw: 20,
        hbp: 20,
        hfp: 40,
        v_active: HEIGHT as u32,
        vsw: 4,
        vbp: 10,
        vfp: 24,
    },
};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum InitError {
    Dbi(DbiError),
}

impl From<DbiError> for InitError {
    fn from(error: DbiError) -> Self {
        Self::Dbi(error)
    }
}

/// Sends the reset and HX8394 initialization sequence from the working C driver.
pub fn initialize_panel(bus: &mut MipiDsi<'_>) -> Result<(), InitError> {
    let delay = Delay::new();
    let mut dbi = bus.dbi(0);

    for command in INIT_SEQUENCE {
        dbi.write_cmd(command.command, command.parameters)?;
        if command.delay_ms != 0 {
            delay.delay_millis(command.delay_ms as u32);
        }
    }

    Ok(())
}

/// Allocates the aligned RGB565 scanout buffer in the PSRAM heap.
pub fn allocate_framebuffer() -> &'static mut Framebuffer {
    let mut framebuffer = Box::<Framebuffer>::new_uninit();
    // SAFETY: The allocation is aligned and all bytes are initialized here.
    unsafe {
        ptr::write_bytes(framebuffer.as_mut_ptr().cast::<u8>(), 0, FRAMEBUFFER_BYTES);
        Box::leak(framebuffer.assume_init())
    }
}

struct InitCommand {
    command: u8,
    parameters: &'static [u8],
    delay_ms: u16,
}

const INIT_SEQUENCE: &[InitCommand] = &[
    InitCommand {
        command: 0x01,
        parameters: &[],
        delay_ms: 120,
    },
    InitCommand {
        command: 0x11,
        parameters: &[],
        delay_ms: 120,
    },
    InitCommand {
        command: 0x36,
        parameters: &[0x00],
        delay_ms: 0,
    },
    InitCommand {
        command: 0x3A,
        parameters: &[0x55],
        delay_ms: 0,
    },
    InitCommand {
        command: 0xBA,
        parameters: &[0x61],
        delay_ms: 0,
    },
    InitCommand {
        command: 0xB9,
        parameters: &[0xFF, 0x83, 0x94],
        delay_ms: 0,
    },
    InitCommand {
        command: 0xB1,
        parameters: &[0x48, 0x0A, 0x6A, 0x09, 0x33, 0x54, 0x71, 0x71, 0x2E, 0x45],
        delay_ms: 0,
    },
    InitCommand {
        command: 0xBA,
        parameters: &[0x61, 0x03, 0x68, 0x6B, 0xB2, 0xC0],
        delay_ms: 0,
    },
    InitCommand {
        command: 0xB2,
        parameters: &[0x00, 0x80, 0x64, 0x0C, 0x06, 0x2F],
        delay_ms: 0,
    },
    InitCommand {
        command: 0xB4,
        parameters: &[
            0x1C, 0x78, 0x1C, 0x78, 0x1C, 0x78, 0x01, 0x0C, 0x86, 0x75, 0x00, 0x3F, 0x1C, 0x78,
            0x1C, 0x78, 0x1C, 0x78, 0x01, 0x0C, 0x86,
        ],
        delay_ms: 0,
    },
    InitCommand {
        command: 0xD3,
        parameters: &[
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x08, 0x32, 0x10, 0x05, 0x00, 0x05, 0x32,
            0x13, 0xC1, 0x00, 0x01, 0x32, 0x10, 0x08, 0x00, 0x00, 0x37, 0x03, 0x07, 0x07, 0x37,
            0x05, 0x05, 0x37, 0x0C, 0x40,
        ],
        delay_ms: 0,
    },
    InitCommand {
        command: 0xD5,
        parameters: &[
            0x18, 0x18, 0x18, 0x18, 0x22, 0x23, 0x20, 0x21, 0x04, 0x05, 0x06, 0x07, 0x00, 0x01,
            0x02, 0x03, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18,
            0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x19, 0x19,
            0x19, 0x19,
        ],
        delay_ms: 0,
    },
    InitCommand {
        command: 0xD6,
        parameters: &[
            0x18, 0x18, 0x19, 0x19, 0x21, 0x20, 0x23, 0x22, 0x03, 0x02, 0x01, 0x00, 0x07, 0x06,
            0x05, 0x04, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18,
            0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x19, 0x19,
            0x18, 0x18,
        ],
        delay_ms: 0,
    },
    InitCommand {
        command: 0xE0,
        parameters: &[
            0x07, 0x08, 0x09, 0x0D, 0x10, 0x14, 0x16, 0x13, 0x24, 0x36, 0x48, 0x4A, 0x58, 0x6F,
            0x76, 0x80, 0x97, 0xA5, 0xA8, 0xB5, 0xC6, 0x62, 0x63, 0x68, 0x6F, 0x72, 0x78, 0x7F,
            0x7F, 0x00, 0x02, 0x08, 0x0D, 0x0C, 0x0E, 0x0F, 0x10, 0x24, 0x36, 0x48, 0x4A, 0x58,
            0x6F, 0x78, 0x82, 0x99, 0xA4, 0xA0, 0xB1, 0xC0, 0x5E, 0x5E, 0x64, 0x6B, 0x6C, 0x73,
            0x7F, 0x7F,
        ],
        delay_ms: 0,
    },
    InitCommand {
        command: 0xCC,
        parameters: &[0x0B],
        delay_ms: 0,
    },
    InitCommand {
        command: 0xC0,
        parameters: &[0x1F, 0x73],
        delay_ms: 0,
    },
    InitCommand {
        command: 0xB6,
        parameters: &[0x6B, 0x6B],
        delay_ms: 0,
    },
    InitCommand {
        command: 0xD4,
        parameters: &[0x02],
        delay_ms: 0,
    },
    InitCommand {
        command: 0xBD,
        parameters: &[0x01],
        delay_ms: 0,
    },
    InitCommand {
        command: 0xB1,
        parameters: &[0x00],
        delay_ms: 0,
    },
    InitCommand {
        command: 0xBD,
        parameters: &[0x00],
        delay_ms: 0,
    },
    InitCommand {
        command: 0xBF,
        parameters: &[0x40, 0x81, 0x50, 0x00, 0x1A, 0xFC, 0x01],
        delay_ms: 0,
    },
    InitCommand {
        command: 0x3A,
        parameters: &[0x50],
        delay_ms: 0,
    },
    InitCommand {
        command: 0x11,
        parameters: &[],
        delay_ms: 200,
    },
    InitCommand {
        command: 0xB2,
        parameters: &[
            0x00, 0x80, 0x64, 0x0C, 0x06, 0x2F, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x18,
        ],
        delay_ms: 0,
    },
    InitCommand {
        command: 0x29,
        parameters: &[],
        delay_ms: 80,
    },
];
