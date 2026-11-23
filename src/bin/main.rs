#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

extern crate alloc;

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_hal::{
    clock::CpuClock,
    delay::Delay,
    gpio::{Level, Output, OutputConfig},
    i2c::master::{Config as I2cConfig, I2c},
    i2s::master::I2s,
    mipi_dsi::{Config as DsiConfig, DataLanes, MipiDsi},
    system::Stack,
    time::Rate,
    timer::timg::TimerGroup,
    usb::otg::Usb,
};
use static_cell::{ConstStaticCell, StaticCell};

#[path = "../waveshare_5dsi_touch_a.rs"]
mod waveshare_5dsi_touch_a;

#[path = "../gt911.rs"]
mod gt911;

#[path = "../slint_ui.rs"]
mod slint_ui;

#[path = "../usb_host.rs"]
mod usb_host;

#[path = "../uvc.rs"]
mod uvc;

#[path = "../jpeg_hw.rs"]
mod jpeg_hw;

#[path = "../perf.rs"]
mod perf;

#[path = "../audio.rs"]
mod audio;

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    esp_println::println!("{info}");
    loop {}
}

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[esp_rtos::main]
async fn main(_spawner: Spawner) -> ! {
    // generator version: 2.0.0
    // generator parameters: -o esp32p4 -o embassy -o unstable-hal -o codex --toolchain stable

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_println::println!("P4 display firmware starting");

    // Keep small, frequently accessed allocations such as JPEG working buffers in SRAM.
    esp_alloc::heap_allocator!(size: 128 * 1024);

    // The 720x1280 RGB565 framebuffer must live in PSRAM and be 64-byte aligned
    // for the MIPI-DSI VDMA engine.
    esp_alloc::psram_allocator!(peripherals.PSRAM, esp_hal::psram, esp_hal::psram::PsramConfig { timing: esp_hal::psram::PsramTimingParams::MHZ_125, ..Default::default() });

    esp_println::println!("PSRAM initialized");

    esp_println::println!("Hello from the ESP32-P4-WIFI6-DEV-KIT!");

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0, peripherals.FROM_CPU_INTR0);

    // The display's backlight controller is at 0x45 on GPIO7/GPIO8.
    let mut i2c = I2c::new(
        peripherals.I2C1,
        I2cConfig::default().with_frequency(Rate::from_khz(100)),
    )
    .expect("configure display I2C")
    .with_sda(peripherals.GPIO7)
    .with_scl(peripherals.GPIO8);
    // The panel controller may need time to become ready after power-up.
    let mut backlight_ready = false;
    for _ in 0..20 {
        if i2c.write(0x45, &[0x95, 0x11]).is_ok() {
            backlight_ready = true;
            break;
        }
        Timer::after(Duration::from_millis(100)).await;
    }
    if backlight_ready {
        i2c.write(0x45, &[0x95, 0x17]).expect("configure backlight");
        i2c.write(0x45, &[0x96, 0x00]).expect("set backlight start level");
        Timer::after(Duration::from_millis(100)).await;
        i2c.write(0x45, &[0x96, 0xff]).expect("turn on backlight");
    } else {
        esp_println::println!("Backlight controller unavailable at 0x45");
    }

    Delay::new().delay_millis(1000);
    let framebuffer = waveshare_5dsi_touch_a::allocate_framebuffer();
    let backbuffer = waveshare_5dsi_touch_a::allocate_framebuffer();
    let mut ui = slint_ui::Ui::new().expect("initialize Slint demo");
    ui.draw(&mut framebuffer.0).await;

    esp_println::println!("Slint framebuffer drawn");
    let touch = gt911::Gt911::detect(&mut i2c);
    esp_println::println!("GT911 touchscreen detected: {}", touch.is_some());
    if let Some(touch) = &touch {
        if let Err(error) = touch.check_status(&mut i2c) {
            esp_println::println!("GT911 status check failed: {error:?}");
        }
    }
    let amplifier = Output::new(peripherals.GPIO53, Level::Low, OutputConfig::default());
    let _audio = match I2s::new(peripherals.I2S0, peripherals.DMA_CH1, audio::i2s_config()) {
        Ok(i2s) => {
            let i2s = i2s.with_mclk(peripherals.GPIO13).into_async();
            let tx = i2s.i2s_tx
                .with_bclk(peripherals.GPIO12)
                .with_ws(peripherals.GPIO10)
                .with_dout(peripherals.GPIO9)
                .build();
            let rx = i2s.i2s_rx.with_din(peripherals.GPIO11).build();
            match audio::Audio::new(&mut i2c, tx, rx, amplifier) {
                Ok(audio) => Some(audio.check_dma().await),
                Err(error) => {
                    esp_println::println!("ES8311 initialization failed: {error:?}");
                    None
                }
            }
        }
        Err(error) => {
            esp_println::println!("Audio I2S initialization failed: {error:?}");
            None
        }
    };

    let mut bus = MipiDsi::new(
        peripherals.MIPI_DSI,
        peripherals.VDMA_CH0,
        DsiConfig::default()
            .with_num_data_lanes(DataLanes::_2)
            .with_lane_bit_rate_mbps(waveshare_5dsi_touch_a::LANE_BIT_RATE_MBPS),
    )
    .expect("initialize MIPI-DSI PHY");

    waveshare_5dsi_touch_a::initialize_panel(&mut bus).expect("initialize HX8394 panel");
    let mut display = bus
        .dpi(waveshare_5dsi_touch_a::DPI_CONFIG, &[&mut framebuffer.0, &mut backbuffer.0])
        .expect("start MIPI-DSI video stream");
    esp_println::println!("Waveshare 5-DSI-TOUCH-A display is streaming");
    static USB_STACK: ConstStaticCell<Stack<8192>> = ConstStaticCell::new(Stack::new());
    static USB_EXECUTOR: StaticCell<esp_rtos::embassy::Executor> = StaticCell::new();
    let usb_hs = peripherals.USB_HS;
    // Keep isochronous polling active while the main core decodes and draws.
    esp_rtos::start_second_core(
        peripherals.CPU_CTRL,
        peripherals.FROM_CPU_INTR1,
        USB_STACK.take(),
        move || {
            let usb = Usb::new_hs(usb_hs);
            USB_EXECUTOR.init(esp_rtos::embassy::Executor::new()).run(|spawner| {
                spawner.spawn(usb_host::run(usb).expect("spawn the USB host task once"));
                if let Some(touch) = touch {
                    spawner.spawn(gt911::run(touch, i2c).expect("spawn the touchscreen task once"));
                }
            });
        },
    );
    let mut preview_frames = 0_u32;
    let mut performance = perf::Sampler::new(preview_frames);
    let mut counter = ui.count();
    loop {
        while let Ok(sample) = gt911::EVENTS.try_receive() {
            ui.touch(sample);
        }
        let current_count = ui.count();
        if current_count != counter {
            counter = current_count;
            esp_println::println!("Slint touch counter: {counter}");
        }
        let frame_start = esp_hal::time::Instant::now();
        let camera_updated = ui.update_camera();
        let decode_done = esp_hal::time::Instant::now();
        if ui.draw(display.framebuffer_mut()).await {
            let draw_done = esp_hal::time::Instant::now();
            display.commit();
            display.wait_for_vsync();
            if camera_updated {
                preview_frames = preview_frames.saturating_add(1);
                if matches!(preview_frames, 1 | 5 | 30) || preview_frames.is_multiple_of(300) {
                    esp_println::println!(
                        "USB camera: rendered {preview_frames} preview frame(s), decode {} ms, draw {} ms, present {} ms",
                        (decode_done - frame_start).as_millis(),
                        (draw_done - decode_done).as_millis(),
                        draw_done.elapsed().as_millis(),
                    );
                }
            }
        }
        let delay_ms = if camera_updated { 1 } else { 16 };
        performance.report(preview_frames);
        Timer::after(Duration::from_millis(delay_ms)).await;
    }
}
