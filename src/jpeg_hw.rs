//! Baseline MJPEG decode for the ESP32-P4 camera preview.
//! Register and descriptor layout follows Espressif's ESP32-P4 JPEG and DMA2D HALs.

use alloc::boxed::Box;
use core::sync::atomic::{Ordering, fence};
#[cfg(feature = "ppa-preview-sleep")]
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use esp_hal::time::{Duration, Instant};
#[cfg(feature = "ppa-preview-sleep")]
use esp_hal::{interrupt, peripherals::Interrupt};
use esp32p4::{HP_SYS_CLKRST, JPEG, PPA};

#[path = "jpeg_tables.rs"]
mod tables;

const WIDTH: usize = crate::uvc::WIDTH as usize;
const HEIGHT: usize = crate::uvc::HEIGHT as usize;
const MAX_SCAN: usize = crate::uvc::MAX_FRAME_BYTES;
const MAX_PIXELS: usize = WIDTH * HEIGHT;
const DMA2D_BASE: usize = 0x5008_8000;
const DMA2D_CLOCK: usize = 0xa04;
const TX: usize = 0;
const RX: usize = 0x500;
const PPA_TX: usize = 0x100;
const PPA_RX: usize = 0x600;
const CH_CONFIG: usize = 0;
const CH_INT_RAW: usize = 4;
const CH_INT_CLEAR: usize = 0x10;
const CH_LINK_CONTROL: usize = 0x1c;
const CH_LINK_ADDRESS: usize = 0x20;
const CH_STATE: usize = 0x24;
const TX_TRIGGER: usize = 0x38;
const TX_DESCRIPTOR_PORT_BLOCK: usize = 0x6c;
const RX_TRIGGER: usize = 0x53c;
const RX_COLOR: usize = 0x54c;
const RX_SCRAMBLE: usize = 0x550;
const RX_COLOR_PARAMS: usize = 0x554;
const CHANNEL_RESET: u32 = 1 << 24;
const DISABLE_COMMANDS: u32 = 1 << 25;
const BURST_CONFIG: u32 = (1 << 2) | (4 << 6) | (1 << 12);
const JPEG_ERROR_MASK: u32 = 0x03fc_e1fe;
pub const PREVIEW_WIDTH: usize = WIDTH / 2;
pub const PREVIEW_HEIGHT: usize = HEIGHT / 2;
#[cfg(feature = "ppa-preview-sleep")]
static PREVIEW_DONE: Signal<CriticalSectionRawMutex, ()> = Signal::new();

#[cfg(feature = "ppa-preview-sleep")]
#[esp_hal::handler]
fn ppa_preview_interrupt() {
    // SAFETY: This handler owns the preview interrupt enable bits.
    let ppa = unsafe { &*PPA::ptr() };
    ppa.int_ena()
        .modify(|_, w| w.sr_eof().clear_bit().sr_param_cfg_err().clear_bit());
    PREVIEW_DONE.signal(());
}

#[repr(C, align(128))]
struct Aligned<T>(T);

pub struct Decoder {
    scan: Box<Aligned<[u8; MAX_SCAN]>>,
    pixels: Box<Aligned<[u16; MAX_PIXELS]>>,
    tx_descriptor: Aligned<[u32; 6]>,
    rx_descriptor: Aligned<[u32; 6]>,
}

#[derive(Debug)]
pub enum Error {
    Unsupported,
    InvalidHeader,
    Transfer { jpeg: u32, dma_tx: u32, dma_rx: u32 },
}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Unsupported => f.write_str("unsupported baseline JPEG format"),
            Self::InvalidHeader => f.write_str("invalid JPEG header"),
            Self::Transfer {
                jpeg,
                dma_tx,
                dma_rx,
            } => {
                write!(
                    f,
                    "JPEG status {jpeg:#x}, DMA TX {dma_tx:#x}, RX {dma_rx:#x}"
                )
            }
        }
    }
}

#[derive(Clone, Copy)]
struct Huffman<'a> {
    lengths: &'a [u8],
    values: &'a [u8],
}

struct Header<'a> {
    quant: [Option<&'a [u8]>; 4],
    huffman: [Huffman<'a>; 4],
    components: [[u8; 3]; 3],
    restart: u16,
    scan: &'a [u8],
}

impl<'a> Header<'a> {
    fn parse(frame: &'a [u8]) -> Result<Self, Error> {
        use tables::*;
        const DC_VALUES: [u8; 12] = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11];
        let mut header = Self {
            quant: [None; 4],
            huffman: [
                Huffman {
                    lengths: &LUMINANCE_DC_COEFFICIENTS,
                    values: &DC_VALUES,
                },
                Huffman {
                    lengths: &CHROMINANCE_DC_COEFFICIENTS,
                    values: &DC_VALUES,
                },
                Huffman {
                    lengths: &LUMINANCE_AC_COEFFICIENTS,
                    values: &LUMINANCE_AC_VALUES,
                },
                Huffman {
                    lengths: &CHROMINANCE_AC_COEFFICIENTS,
                    values: &CHROMINANCE_AC_VALUES,
                },
            ],
            components: [[0; 3]; 3],
            restart: 0,
            scan: &[],
        };
        let mut rest = frame
            .strip_prefix(&[0xff, 0xd8])
            .ok_or(Error::InvalidHeader)?;
        loop {
            let [0xff, marker, hi, lo, ..] = rest else {
                return Err(Error::InvalidHeader);
            };
            if *marker == 0xff {
                rest = &rest[1..];
                continue;
            }
            let length = usize::from(u16::from_be_bytes([*hi, *lo]));
            let end = length + 2;
            let data = rest.get(4..end).ok_or(Error::InvalidHeader)?;
            match *marker {
                0xdb => {
                    let tables = data.chunks_exact(65);
                    if !tables.remainder().is_empty() {
                        return Err(Error::Unsupported);
                    }
                    for table in tables {
                        *header
                            .quant
                            .get_mut(table[0] as usize)
                            .ok_or(Error::Unsupported)? = Some(&table[1..]);
                    }
                }
                0xc0 => {
                    let [8, h0, h1, w0, w1, 3, components @ ..] = data else {
                        return Err(Error::Unsupported);
                    };
                    if usize::from(u16::from_be_bytes([*w0, *w1])) != WIDTH
                        || usize::from(u16::from_be_bytes([*h0, *h1])) != HEIGHT
                        || components.len() != 9
                    {
                        return Err(Error::Unsupported);
                    }
                    for (component, bytes) in
                        header.components.iter_mut().zip(components.chunks_exact(3))
                    {
                        component.copy_from_slice(bytes);
                    }
                }
                0xc4 => {
                    let mut tables = data;
                    while !tables.is_empty() {
                        let [selector, lengths @ ..] =
                            tables.get(..17).ok_or(Error::InvalidHeader)?
                        else {
                            return Err(Error::InvalidHeader);
                        };
                        let index = match *selector {
                            0 => 0,
                            1 => 1,
                            0x10 => 2,
                            0x11 => 3,
                            _ => return Err(Error::Unsupported),
                        };
                        let count = lengths
                            .iter()
                            .map(|count| usize::from(*count))
                            .sum::<usize>();
                        let limit = if index < 2 { 12 } else { 162 };
                        if !(1..=limit).contains(&count) {
                            return Err(Error::InvalidHeader);
                        }
                        let values = tables.get(17..17 + count).ok_or(Error::InvalidHeader)?;
                        header.huffman[index] = Huffman { lengths, values };
                        tables = &tables[17 + count..];
                    }
                }
                0xdd => {
                    header.restart =
                        u16::from_be_bytes(data.try_into().map_err(|_| Error::InvalidHeader)?);
                }
                0xda => {
                    if !matches!(data, [3, _, _, _, _, _, _, 0, 63, 0])
                        || !matches!(
                            header.components,
                            [[_, 0x11 | 0x21 | 0x22, _], [_, 0x11, _], [_, 0x11, _]]
                        )
                        || header
                            .components
                            .iter()
                            .any(|c| header.quant.get(c[2] as usize).is_none_or(Option::is_none))
                    {
                        return Err(Error::Unsupported);
                    }
                    header.scan = rest;
                    return Ok(header);
                }
                0xe0..=0xef | 0xfe => (),
                _ => return Err(Error::Unsupported),
            }
            rest = &rest[end..];
        }
    }
}

impl Decoder {
    /// Call once, before starting any other DMA2D client.
    pub fn new() -> Self {
        // SAFETY: The firmware reserves the JPEG codec and DMA2D for this decoder.
        let clocks = unsafe { &*HP_SYS_CLKRST::ptr() };
        critical_section::with(|_| {
            clocks.soc_clk_ctrl1().modify(|_, w| {
                w.jpeg_sys_clk_en()
                    .set_bit()
                    .dma2d_sys_clk_en()
                    .set_bit()
                    .ppa_sys_clk_en()
                    .set_bit()
            });
            clocks
                .hp_rst_en0()
                .modify(|_, w| w.rst_en_jpeg().set_bit().rst_en_dma2d().set_bit());
            clocks
                .hp_rst_en0()
                .modify(|_, w| w.rst_en_jpeg().clear_bit().rst_en_dma2d().clear_bit());
            clocks.hp_rst_en1().modify(|_, w| w.rst_en_ppa().set_bit());
            clocks
                .hp_rst_en1()
                .modify(|_, w| w.rst_en_ppa().clear_bit());
        });
        dma_write(DMA2D_CLOCK, 0b111);
        dma_write(DMA2D_CLOCK, 0b100);
        #[cfg(feature = "ppa-preview-sleep")]
        interrupt::bind_handler(Interrupt::PPA, ppa_preview_interrupt);
        Self {
            scan: zeroed_buffer(),
            pixels: zeroed_buffer(),
            tx_descriptor: Aligned([0; 6]),
            rx_descriptor: Aligned([0; 6]),
        }
    }

    pub fn decode(&mut self, frame: &[u8]) -> Result<&[u16], Error> {
        let header = Header::parse(frame)?;
        let scan_len = header.scan.len();
        let scan = self
            .scan
            .0
            .get_mut(..scan_len)
            .ok_or(Error::InvalidHeader)?;
        scan.copy_from_slice(header.scan);
        let sampling = header.components[0][1];
        let (block_width, block_height, macroblock, input_color) = match sampling {
            0x11 => (40, 8, 0, 2),
            0x21 => (32, 8, 1, 0),
            _ => (32, 16, 2, 0),
        };
        if HEIGHT % block_height != 0 || WIDTH % 16 != 0 {
            return Err(Error::Unsupported);
        }
        configure_jpeg(&header);
        const DMA_OWNER: u32 = 1 << 31;
        const MODE_2D: u32 = 1 << 29;
        const MULTIPLE_BLOCKS: u32 = 1 << 28;
        const RGB565: u32 = 3 << 28;
        let low = (scan_len as u32) & 0x3fff;
        let high = (scan_len as u32) >> 14;
        self.tx_descriptor.0 = [
            DMA_OWNER | (low << 14) | low,
            (1 << 28) | (high << 14) | high,
            0,
            scan.as_ptr() as u32,
            0,
            0,
        ];
        self.rx_descriptor.0 = [
            DMA_OWNER | MODE_2D | ((block_width as u32) << 14) | block_height as u32,
            RGB565 | ((WIDTH as u32) << 14) | HEIGHT as u32,
            MULTIPLE_BLOCKS,
            self.pixels.0.as_mut_ptr() as u32,
            0,
            0,
        ];
        dma_write(TX + CH_CONFIG, BURST_CONFIG | (3 << 9) | (1 << 1));
        dma_write(RX + CH_CONFIG, BURST_CONFIG | (macroblock << 9) | (1 << 16));
        dma_write(TX_TRIGGER, 0);
        dma_write(RX_TRIGGER, 0);
        configure_rgb565(input_color);
        writeback(
            scan.as_ptr() as u32,
            scan_len.next_multiple_of(128) as u32,
            true,
        );
        writeback(self.pixels.0.as_ptr() as u32, (MAX_PIXELS * 2) as u32, true);
        let tx = &raw mut self.tx_descriptor as u32;
        let rx = &raw mut self.rx_descriptor as u32;
        writeback(tx, 128, false);
        writeback(rx, 128, false);
        fence(Ordering::SeqCst);
        dma_write(TX + CH_LINK_ADDRESS, tx);
        dma_write(RX + CH_LINK_ADDRESS, rx);
        dma_write(TX + CH_LINK_CONTROL, 1 << 21);
        dma_write(RX + CH_LINK_CONTROL, 1 << 22);
        let jpeg = jpeg_regs();
        jpeg.config().modify(|_, w| w.jpeg_start().set_bit());
        let start = Instant::now();
        let result = loop {
            let jpeg = jpeg.int_raw().read().bits();
            let dma_rx = dma_read(RX + CH_INT_RAW);
            let dma_tx = dma_read(TX + CH_INT_RAW);
            if jpeg & JPEG_ERROR_MASK != 0
                || dma_rx & 0xc != 0
                || dma_tx & 4 != 0
                || start.elapsed() >= Duration::from_millis(150)
            {
                break Err(Error::Transfer {
                    jpeg,
                    dma_tx,
                    dma_rx,
                });
            }
            if dma_rx & (1 << 1) != 0 {
                break Ok(());
            }
            core::hint::spin_loop();
        };
        stop_dma(TX, RX);
        // SAFETY: The DMA target is aligned and isolated from other allocations.
        // Both DMA channels have stopped, and CPU access resumes after invalidation.
        critical_section::with(|_| unsafe {
            Cache_Invalidate_Addr(
                (1 << 4) | (1 << 5),
                self.pixels.0.as_ptr() as u32,
                (MAX_PIXELS * 2) as u32,
            )
        });
        fence(Ordering::SeqCst);
        result?;
        Ok(&self.pixels.0)
    }

    /// Scale the decoded RGB565 frame by half into the DSI back buffer.
    pub async fn draw_preview(&mut self, frame: &mut [u8], x: usize, y: usize) -> bool {
        use crate::waveshare_5dsi_touch_a::{HEIGHT as PANEL_HEIGHT, WIDTH as PANEL_WIDTH};
        if frame.len() != PANEL_WIDTH * PANEL_HEIGHT * 2
            || !frame.as_ptr().addr().is_multiple_of(128)
            || x > PANEL_WIDTH - PREVIEW_WIDTH
            || y > PANEL_HEIGHT - PREVIEW_HEIGHT
        {
            return false;
        }

        // SAFETY: This preview owns PPA and uses its own DMA2D channels. The aligned
        // descriptors are reused after each engine stops.
        let ppa = unsafe { &*PPA::ptr() };
        ppa.sr_scal_rotate()
            .modify(|_, w| w.scal_rotate_rst().set_bit());
        ppa.sr_scal_rotate()
            .modify(|_, w| w.scal_rotate_rst().clear_bit());
        ppa.int_clr().write(|w| {
            // SAFETY: The interrupt clear register accepts write-one-to-clear bits.
            unsafe { w.bits(u32::MAX) }
        });

        const DMA_OWNER_2D_EOF: u32 = (1 << 31) | (1 << 30) | (1 << 29);
        const RGB565: u32 = 3 << 28;
        self.tx_descriptor.0 = [
            DMA_OWNER_2D_EOF | ((WIDTH as u32) << 14) | HEIGHT as u32,
            RGB565 | ((WIDTH as u32) << 14) | HEIGHT as u32,
            0,
            self.pixels.0.as_ptr() as u32,
            0,
            0,
        ];
        self.rx_descriptor.0 = [
            DMA_OWNER_2D_EOF | (2 << 14) | 2,
            RGB565 | ((PANEL_WIDTH as u32) << 14) | PANEL_HEIGHT as u32,
            ((x as u32) << 14) | y as u32,
            frame.as_mut_ptr() as u32,
            0,
            0,
        ];
        dma_write(
            PPA_TX + CH_CONFIG,
            BURST_CONFIG | (3 << 9) | (1 << 11) | (1 << 1),
        );
        dma_write(PPA_RX + CH_CONFIG, BURST_CONFIG | (3 << 9) | (1 << 11));
        dma_write(TX_TRIGGER + PPA_TX, 1);
        dma_write(RX_TRIGGER + PPA_RX - RX, 1);
        let block = if ppa.sr_byte_order().read().sr_bk_size_sel().bit() {
            18
        } else {
            34
        };
        dma_write(PPA_TX + TX_DESCRIPTOR_PORT_BLOCK, (block << 14) | block);
        // Preserve Slint's CPU writes before the PPA updates the shared back buffer.
        writeback(frame.as_ptr() as u32, frame.len() as u32, true);
        let tx = &raw mut self.tx_descriptor as u32;
        let rx = &raw mut self.rx_descriptor as u32;
        writeback(tx, 128, false);
        writeback(rx, 128, false);
        fence(Ordering::SeqCst);
        dma_write(PPA_TX + CH_LINK_ADDRESS, tx);
        dma_write(PPA_RX + CH_LINK_ADDRESS, rx);
        dma_write(PPA_TX + CH_LINK_CONTROL, 1 << 21);
        dma_write(PPA_RX + CH_LINK_CONTROL, 1 << 22);
        // SAFETY: 2 selects RGB565, and 8/16 is a valid PPA scaling factor.
        unsafe {
            ppa.sr_color_mode()
                .write(|w| w.sr_rx_cm().bits(2).sr_tx_cm().bits(2));
            ppa.sr_scal_rotate().write(|w| {
                w.sr_scal_x_int()
                    .bits(0)
                    .sr_scal_x_frag()
                    .bits(8)
                    .sr_scal_y_int()
                    .bits(0)
                    .sr_scal_y_frag()
                    .bits(8)
            });
        }
        ppa.sr_byte_order()
            .modify(|_, w| w.sr_rx_byte_swap_en().set_bit());
        let transfer = PreviewTransfer { frame };
        #[cfg(feature = "ppa-preview-sleep")]
        {
            PREVIEW_DONE.reset();
            ppa.int_ena()
                .modify(|_, w| w.sr_eof().set_bit().sr_param_cfg_err().set_bit());
        }
        ppa.sr_scal_rotate()
            .modify(|_, w| w.scal_rotate_start().set_bit());
        let start = Instant::now();
        #[cfg(feature = "ppa-preview-sleep")]
        let _ = embassy_time::with_timeout(
            embassy_time::Duration::from_millis(150),
            PREVIEW_DONE.wait(),
        )
        .await;
        let ok = loop {
            let rx_status = dma_read(PPA_RX + CH_INT_RAW);
            if ppa.int_raw().read().sr_param_cfg_err().bit()
                || rx_status & 0xc != 0
                || dma_read(PPA_TX + CH_INT_RAW) & 4 != 0
                || start.elapsed() >= Duration::from_millis(150)
            {
                break false;
            }
            if rx_status & (1 << 1) != 0 {
                break true;
            }
            // The PPA can signal completion before DMA writes the final pixels.
            #[cfg(feature = "ppa-preview-sleep")]
            embassy_time::Timer::after_micros(250).await;
            #[cfg(not(feature = "ppa-preview-sleep"))]
            core::hint::spin_loop();
        };
        if !ok {
            esp_println::println!(
                "USB camera: PPA preview error, PPA {:#x}, parameters {:#x}, engine state {:#x}, DMA TX {:#x}, RX {:#x}",
                ppa.int_raw().read().bits(),
                ppa.sr_param_err_st().read().bits(),
                ppa.sr_status().read().bits(),
                dma_read(PPA_TX + CH_INT_RAW),
                dma_read(PPA_RX + CH_INT_RAW),
            );
            esp_println::println!(
                "USB camera: PPA DMA state TX {:#x} RX {:#x}, link TX {:#x} RX {:#x}",
                dma_read(PPA_TX + CH_STATE),
                dma_read(PPA_RX + CH_STATE),
                dma_read(PPA_TX + CH_LINK_CONTROL),
                dma_read(PPA_RX + CH_LINK_CONTROL),
            );
        }
        drop(transfer);
        ok
    }

    pub fn draw_preview_software(&self, frame: &mut [u8], x: usize, y: usize) {
        use crate::waveshare_5dsi_touch_a::{HEIGHT as PANEL_HEIGHT, WIDTH as PANEL_WIDTH};
        if frame.len() != PANEL_WIDTH * PANEL_HEIGHT * 2
            || x > PANEL_WIDTH - PREVIEW_WIDTH
            || y > PANEL_HEIGHT - PREVIEW_HEIGHT
        {
            return;
        }
        for (row, source) in self.pixels.0.chunks_exact(WIDTH).step_by(2).enumerate() {
            let offset = ((y + row) * PANEL_WIDTH + x) * 2;
            let output = &mut frame[offset..offset + PREVIEW_WIDTH * 2];
            for (pixel, value) in output.chunks_exact_mut(2).zip(source.iter().step_by(2)) {
                pixel.copy_from_slice(&value.to_be_bytes());
            }
        }
    }

    pub fn fill_rectangle(
        &mut self,
        frame: &mut [u8],
        x: usize,
        y: usize,
        width: usize,
        height: usize,
        color: u16,
    ) -> bool {
        use crate::waveshare_5dsi_touch_a::{HEIGHT as PANEL_HEIGHT, WIDTH as PANEL_WIDTH};
        if frame.len() != PANEL_WIDTH * PANEL_HEIGHT * 2
            || !frame.as_ptr().addr().is_multiple_of(128)
            || width == 0
            || height == 0
            || x > PANEL_WIDTH
            || y > PANEL_HEIGHT
            || width > PANEL_WIDTH - x
            || height > PANEL_HEIGHT - y
        {
            return false;
        }

        // SAFETY: Access is serialized with JPEG decoding and camera scaling.
        let ppa = unsafe { &*PPA::ptr() };
        let blend = ppa.blend_trans_mode();
        blend.modify(|_, w| w.blend_rst().set_bit());
        blend.modify(|_, w| w.blend_rst().clear_bit());
        ppa.int_clr().write(|w| {
            w.blend_param_cfg_err()
                .clear_bit_by_one()
                .blend_eof()
                .clear_bit_by_one()
        });
        self.rx_descriptor.0 = [
            (7 << 29) | ((width as u32) << 14) | height as u32,
            (3 << 28) | ((PANEL_WIDTH as u32) << 14) | PANEL_HEIGHT as u32,
            ((x as u32) << 14) | y as u32,
            frame.as_mut_ptr() as u32,
            0,
            0,
        ];

        // Preserve adjacent Slint pixels that share the touched cache lines.
        let row_bytes = PANEL_WIDTH * 2;
        let cache_start = (y * row_bytes) & !127;
        let cache_end = ((y + height) * row_bytes).next_multiple_of(128);
        let address = frame.as_ptr() as u32 + cache_start as u32;
        let length = (cache_end - cache_start) as u32;
        writeback(address, length, true);
        let rx = &raw mut self.rx_descriptor as u32;
        writeback(rx, 128, false);
        fence(Ordering::SeqCst);
        dma_write(PPA_RX + CH_CONFIG, BURST_CONFIG | (3 << 9));
        dma_write(RX_TRIGGER + PPA_RX - RX, 2);
        dma_write(PPA_RX + CH_LINK_ADDRESS, rx);
        dma_write(PPA_RX + CH_LINK_CONTROL, 1 << 22);
        // The fixed-color register is RGB888 even when the target is RGB565.
        let rgb888 = (u32::from(color & 0xf800) << 8)
            | (u32::from(color & 0x07e0) << 5)
            | (u32::from(color & 0x001f) << 3);
        // SAFETY: The dimensions fit the hardware fields and mode 2 selects RGB565.
        unsafe {
            ppa.blend_color_mode()
                .modify(|_, w| w.blend_tx_cm().bits(2));
            ppa.blend_fix_pixel()
                .write(|w| w.blend_tx_fix_pixel().bits(rgb888));
            ppa.blend_tx_size().write(|w| {
                w.blend_hb()
                    .bits(width as u16)
                    .blend_vb()
                    .bits(height as u16)
            });
        }
        blend.modify(|_, w| {
            w.blend_bypass()
                .clear_bit()
                .blend_fix_pixel_fill_en()
                .set_bit()
                .blend_en()
                .set_bit()
        });
        blend.modify(|_, w| w.update().set_bit());

        let start = Instant::now();
        let ok = loop {
            let raw = dma_read(PPA_RX + CH_INT_RAW);
            if raw & 0xc != 0
                || ppa.int_raw().read().blend_param_cfg_err().bit()
                || start.elapsed() >= Duration::from_millis(75)
            {
                break false;
            }
            if raw & (1 << 1) != 0 {
                break true;
            }
            core::hint::spin_loop();
        };
        if !ok {
            esp_println::println!(
                "Slint PPA fill status: normal {:#x}, blend {:#x}, DMA {:#x}",
                ppa.int_raw().read().bits(),
                ppa.blend_st().read().bits(),
                dma_read(PPA_RX + CH_INT_RAW),
            );
        }
        stop_dma(PPA_TX, PPA_RX);
        // SAFETY: The affected range contains full cache lines, and CPU writes
        // were flushed before DMA. The peripheral has stopped.
        critical_section::with(|_| unsafe {
            Cache_Invalidate_Addr((1 << 4) | (1 << 5), address, length)
        });
        fence(Ordering::SeqCst);

        let first = (y * PANEL_WIDTH + x) * 2;
        let last = ((y + height - 1) * PANEL_WIDTH + x + width - 1) * 2;
        let expected = color.to_le_bytes();
        if ok && (frame[first..first + 2] != expected || frame[last..last + 2] != expected) {
            esp_println::println!(
                "Slint PPA RGB565 fill mismatch: expected {expected:?}, first {:?}, last {:?}",
                &frame[first..first + 2],
                &frame[last..last + 2],
            );
        }
        ok && frame[first..first + 2] == expected && frame[last..last + 2] == expected
    }
}

struct PreviewTransfer<'a> {
    frame: &'a mut [u8],
}

impl Drop for PreviewTransfer<'_> {
    fn drop(&mut self) {
        // SAFETY: This transfer is the sole PPA preview transaction.
        let ppa = unsafe { &*PPA::ptr() };
        ppa.int_ena()
            .modify(|_, w| w.sr_eof().clear_bit().sr_param_cfg_err().clear_bit());
        stop_dma(PPA_TX, PPA_RX);
        // SAFETY: The complete, cache-line-aligned frame is exclusively borrowed,
        // CPU writes were flushed before starting DMA, and the channels are stopped.
        critical_section::with(|_| unsafe {
            Cache_Invalidate_Addr(
                (1 << 4) | (1 << 5),
                self.frame.as_ptr() as u32,
                self.frame.len() as u32,
            )
        });
        fence(Ordering::SeqCst);
    }
}

fn zeroed_buffer<const N: usize, T: Copy + Default>() -> Box<Aligned<[T; N]>> {
    let mut buffer = Box::<Aligned<[T; N]>>::new_uninit();
    // SAFETY: Used only for u8 and u16, for which the all-zero pattern is valid.
    unsafe {
        buffer.as_mut_ptr().write_bytes(0, 1);
        buffer.assume_init()
    }
}

fn jpeg_regs() -> &'static esp32p4::jpeg::RegisterBlock {
    // SAFETY: This module is the sole owner of the JPEG peripheral.
    unsafe { &*JPEG::ptr() }
}

fn configure_jpeg(header: &Header<'_>) {
    let jpeg = jpeg_regs();
    jpeg.config().modify(|_, w| w.soft_rst().set_bit());
    jpeg.config().modify(|_, w| {
        w.soft_rst()
            .clear_bit()
            .mode()
            .set_bit()
            .qnr_fifo_en()
            .set_bit()
    });
    jpeg.int_clr().write(|w| unsafe { w.bits(u32::MAX) });
    jpeg.pic_size().reset();
    jpeg.pic_size()
        .write(|w| unsafe { w.ha().bits(WIDTH as u16).va().bits(HEIGHT as u16) });
    jpeg.decode_conf().write(|w| unsafe {
        w.component_num()
            .bits(3)
            .restart_interval()
            .bits(header.restart)
    });
    jpeg.dqt_info().write(|w| unsafe {
        w.t0_dqt_info()
            .bits(0)
            .t1_dqt_info()
            .bits(1)
            .t2_dqt_info()
            .bits(2)
            .t3_dqt_info()
            .bits(3)
    });
    for (id, table) in header.quant.iter().enumerate() {
        if let Some(table) = table {
            let mut ordered = [0; 64];
            for (&value, &index) in table.iter().zip(&tables::ZIGZAG_ARR) {
                ordered[index as usize] = value;
            }
            let port = [
                jpeg.t0qnr().as_ptr(),
                jpeg.t1qnr().as_ptr(),
                jpeg.t2qnr().as_ptr(),
                jpeg.t3qnr().as_ptr(),
            ][id];
            for value in ordered {
                // SAFETY: These PAC read-only registers are FIFO write ports in decode mode.
                unsafe { port.write_volatile(u32::from(value)) };
            }
        }
    }
    for (index, &[id, sampling, quant]) in header.components.iter().enumerate() {
        jpeg.c(index).write(|w| unsafe {
            w.id()
                .bits(id)
                .x_factor()
                .bits(sampling >> 4)
                .y_factor()
                .bits(sampling & 15)
                .dqt_tbl_sel()
                .bits(quant)
        });
    }
    let huffman_ports = [
        (
            jpeg.dht_totlen_dc0().as_ptr(),
            jpeg.dht_codemin_dc0().as_ptr(),
            jpeg.dht_val_dc0().as_ptr(),
            12,
        ),
        (
            jpeg.dht_totlen_dc1().as_ptr(),
            jpeg.dht_codemin_dc1().as_ptr(),
            jpeg.dht_val_dc1().as_ptr(),
            12,
        ),
        (
            jpeg.dht_totlen_ac0().as_ptr(),
            jpeg.dht_codemin_ac0().as_ptr(),
            jpeg.dht_val_ac0().as_ptr(),
            162,
        ),
        (
            jpeg.dht_totlen_ac1().as_ptr(),
            jpeg.dht_codemin_ac1().as_ptr(),
            jpeg.dht_val_ac1().as_ptr(),
            162,
        ),
    ];
    for (table, (total_port, min_port, value_port, capacity)) in
        header.huffman.iter().zip(huffman_ports)
    {
        let mut total = 0;
        let mut code = 0;
        for (index, &count) in table.lengths.iter().enumerate() {
            total += u32::from(count);
            let minimum = if count == 0 {
                0xffff
            } else {
                (code << (15 - index)) & 0xffff
            };
            code = (code + u32::from(count)) << 1;
            // SAFETY: All three ports belong to the current Huffman FIFO.
            unsafe {
                total_port.write_volatile(total);
                min_port.write_volatile(minimum);
            }
        }
        for index in 0..capacity {
            let value = u32::from(table.values.get(index).copied().unwrap_or(0));
            unsafe {
                value_port.write_volatile(value);
            }
        }
    }
}

fn configure_rgb565(input: u32) {
    dma_write(RX_COLOR, (input << 3) | (1 << 2));
    dma_write(RX_SCRAMBLE, 1 << 3);
    // JPEG full-range YCbCr-to-RGB coefficients in the DMA2D fixed-point layout.
    for (row, [a, b, c, d]) in [
        [256_i32, 0, 359, -45952],
        [256, -88, -183, 34688],
        [256, 454, 0, -58112],
    ]
    .into_iter()
    .enumerate()
    {
        dma_write(
            RX_COLOR_PARAMS + row * 8,
            a as u32 | (((b as u32) & 0x7ff) << 10),
        );
        dma_write(
            RX_COLOR_PARAMS + row * 8 + 4,
            ((c as u32) & 0x3ff) | (((d as u32) & 0x3ffff) << 10),
        );
    }
}

fn stop_dma(tx: usize, rx: usize) {
    dma_write(tx + CH_LINK_CONTROL, 1 << 20);
    dma_write(rx + CH_LINK_CONTROL, 1 << 21);
    dma_write(tx + CH_CONFIG, dma_read(tx + CH_CONFIG) | DISABLE_COMMANDS);
    dma_write(rx + CH_CONFIG, dma_read(rx + CH_CONFIG) | DISABLE_COMMANDS);
    let start = Instant::now();
    while dma_read(tx + CH_STATE) & (1 << 24) == 0 || dma_read(rx + CH_STATE) & (1 << 23) == 0 {
        if start.elapsed() >= Duration::from_millis(5) {
            dma_write(DMA2D_CLOCK, 0b111);
            dma_write(DMA2D_CLOCK, 0b100);
            break;
        }
        core::hint::spin_loop();
    }
    for channel in [tx, rx] {
        dma_write(channel + CH_CONFIG, CHANNEL_RESET);
        dma_write(channel + CH_CONFIG, 0);
        dma_write(channel + CH_INT_CLEAR, u32::MAX);
    }
}

fn dma_write(offset: usize, value: u32) {
    // SAFETY: The caller uses this module's fixed offsets for the reserved DMA2D channels.
    unsafe { ((DMA2D_BASE + offset) as *mut u32).write_volatile(value) };
}

fn dma_read(offset: usize) -> u32 {
    // SAFETY: The caller uses this module's fixed offsets for the reserved DMA2D channels.
    unsafe { ((DMA2D_BASE + offset) as *const u32).read_volatile() }
}

unsafe extern "C" {
    fn Cache_WriteBack_Addr(bus: u32, address: u32, size: u32);
    fn Cache_Invalidate_Addr(bus: u32, address: u32, size: u32);
}

fn writeback(address: u32, size: u32, psram: bool) {
    // SAFETY: Callers pass a live, cache-line-aligned DMA allocation and its size.
    critical_section::with(|_| unsafe {
        Cache_WriteBack_Addr(1 << 4, address, size);
        if psram {
            Cache_WriteBack_Addr(1 << 5, address, size);
        }
    });
}
