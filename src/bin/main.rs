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
    i2c::master::{Config as I2cConfig, I2c},
    mipi_dsi::{Config as DsiConfig, DataLanes, MipiDsi},
    time::Rate,
    timer::timg::TimerGroup,
};

#[path = "../waveshare_5dsi_touch_a.rs"]
mod waveshare_5dsi_touch_a;

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
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
async fn main(spawner: Spawner) -> ! {
    // generator version: 2.0.0
    // generator parameters: -o esp32p4 -o embassy -o unstable-hal -o codex --toolchain stable

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_println::println!("P4 display firmware starting");

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
    i2c.write(0x45, &[0x95, 0x11])
        .expect("enable backlight controller");
    i2c.write(0x45, &[0x95, 0x17])
        .expect("configure backlight controller");
    i2c.write(0x45, &[0x96, 0x00])
        .expect("set backlight start level");
    Delay::new().delay_millis(100);
    i2c.write(0x45, &[0x96, 0xFF])
        .expect("turn on display backlight");

    Delay::new().delay_millis(1000);
    let framebuffer = waveshare_5dsi_touch_a::allocate_test_framebuffer();

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
        .dpi(waveshare_5dsi_touch_a::DPI_CONFIG, &[&mut framebuffer.0])
        .expect("start MIPI-DSI video stream");
    esp_println::println!("Waveshare 5-DSI-TOUCH-A display is streaming");
    for offset in (0..0x94).step_by(4) {
        // SAFETY: aligned reads within the ESP32-P4 DSI bridge register block.
        let value = unsafe { core::ptr::read_volatile((esp_hal::peripherals::MIPI_DSI_BRIDGE::PTR as *const u8).add(offset).cast::<u32>()) };
        esp_println::println!("BRG {:02x} {:08x}", offset, value);
    }
    // Measure scanout separately from application rendering throughput.
    display.wait_for_vsync();
    let started = esp_hal::time::Instant::now();
    for _ in 0..120 {
        display.wait_for_vsync();
    }
    let elapsed_us = started.elapsed().as_micros();
    esp_println::println!("Scanout: 120 frames in {} us ({} milli-Hz)",
        elapsed_us, 120_000_000_000u64 / elapsed_us);

    let _ = spawner;

    loop {
        Timer::after(Duration::from_secs(5)).await;
        let underrun = esp_hal::peripherals::MIPI_DSI_BRIDGE::regs()
            .int_raw()
            .read()
            .underrun()
            .bit_is_set();
        esp_println::println!("DSI framebuffer underrun: {underrun}");
    }

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v1.2.2/examples
}
