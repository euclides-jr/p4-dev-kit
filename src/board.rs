use crate::{app::App, audio, gt911, usb_host, waveshare_5dsi_touch_a};
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_hal::{
    clock::CpuClock,
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

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
pub async fn run(spawner: Spawner) -> ! {
    // generator version: 2.0.0
    // generator parameters: -o esp32p4 -o embassy -o unstable-hal -o codex --toolchain stable

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_println::logger::init_logger(log::LevelFilter::Warn);
    esp_println::println!("P4 display firmware starting");

    // Keep small working allocations in SRAM while leaving enough main-stack
    // headroom for the ESP-Hosted protobuf encoder/decoder call chain.
    esp_alloc::heap_allocator!(size: 64 * 1024);

    // The 720x1280 RGB565 framebuffer must live in PSRAM and be 64-byte aligned
    // for the MIPI-DSI VDMA engine.
    esp_alloc::psram_allocator!(
        peripherals.PSRAM,
        esp_hal::psram,
        esp_hal::psram::PsramConfig {
            timing: esp_hal::psram::PsramTimingParams::MHZ_125,
            ..Default::default()
        }
    );

    esp_println::println!("PSRAM initialized");

    esp_println::println!("Hello from the ESP32-P4-WIFI6-DEV-KIT!");

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0, peripherals.FROM_CPU_INTR0);

    // The Wi-Fi coprocessor is wired to SDMMC slot 1 on this board.
    static SD_HOST: StaticCell<esp_hal::sdmmc::SdHostController<'static>> = StaticCell::new();
    let host = SD_HOST.init(
        esp_hal::sdmmc::SdHostController::new(peripherals.SDHOST, Default::default())
            .expect("configure SDMMC controller"),
    );
    let slot = host
        .slot::<1>(Default::default())
        .expect("configure Wi-Fi SDIO slot")
        .with_clk(peripherals.GPIO18)
        .with_cmd(peripherals.GPIO19)
        .with_data0(peripherals.GPIO14)
        .with_data1(peripherals.GPIO15)
        .with_data2(peripherals.GPIO16)
        .with_data3(peripherals.GPIO17)
        .into_async();
    let reset = Output::new(peripherals.GPIO54, Level::Low, OutputConfig::default());

    // The display's backlight controller is at 0x45 on GPIO7/GPIO8.
    let mut i2c = I2c::new(
        peripherals.I2C1,
        I2cConfig::default().with_frequency(Rate::from_khz(100)),
    )
    .expect("configure display I2C")
    .with_sda(peripherals.GPIO7)
    .with_scl(peripherals.GPIO8);
    // The panel controller may need time to become ready after power-up.
    let mut backlight_setup = false;
    for _ in 0..20 {
        let result = (|| {
            i2c.write(0x45, &[0x95, 0x11])?;
            i2c.write(0x45, &[0x95, 0x17])?;
            i2c.write(0x45, &[0x96, 0x00])
        })();
        if result.is_ok() {
            backlight_setup = true;
            break;
        }
        Timer::after_millis(100).await;
    }
    if !backlight_setup {
        esp_println::println!("Backlight controller did not respond during startup");
    }
    Timer::after_millis(1000).await;

    let framebuffer = waveshare_5dsi_touch_a::allocate_framebuffer();
    let backbuffer = waveshare_5dsi_touch_a::allocate_framebuffer();
    // Set to false to remove the preview and route camera events elsewhere.
    const ENABLE_PREVIEW: bool = false;
    let mut app = App::new(ENABLE_PREVIEW).expect("initialize Slint demo");
    app.draw(&mut framebuffer.0).await;

    esp_println::println!("Slint framebuffer drawn");
    let touch = gt911::Gt911::detect(&mut i2c);
    esp_println::println!("GT911 touchscreen detected: {}", touch.is_some());
    if let Some(touch) = &touch {
        if let Err(error) = touch.check_status(&mut i2c) {
            esp_println::println!("GT911 status check failed: {error:?}");
        }
    }
    let amplifier = Output::new(peripherals.GPIO53, Level::Low, OutputConfig::default());
    let audio = match I2s::new(peripherals.I2S0, peripherals.DMA_CH1, audio::i2s_config()) {
        Ok(i2s) => {
            let i2s = i2s.with_mclk(peripherals.GPIO13).into_async();
            let tx = i2s
                .i2s_tx
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

    if let Some(audio) = audio
        && option_env!("SENDSPIN_SERVER_IP").is_some()
        && option_env!("SENDSPIN_IDENTITY_KEY").is_some()
    {
        let layout =
            core::alloc::Layout::from_size_align(crate::sendspin::pcm::PSRAM_BYTES, 4).unwrap();
        let memory = unsafe {
            esp_alloc::HEAP.alloc_caps(esp_alloc::MemoryCapability::External.into(), layout)
        };
        if memory.is_null() {
            esp_println::println!("Sendspin: unable to allocate PSRAM buffers");
        } else {
            unsafe {
                memory.write_bytes(0, layout.size());
            }
            let memory = unsafe { core::slice::from_raw_parts_mut(memory, layout.size()) };
            crate::sendspin::pcm::init(memory);
            spawner.spawn(crate::audio::playback(audio).unwrap());
        }
    }

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
        .dpi(
            waveshare_5dsi_touch_a::DPI_CONFIG,
            &[&mut framebuffer.0, &mut backbuffer.0],
        )
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
            USB_EXECUTOR
                .init(esp_rtos::embassy::Executor::new())
                .run(|spawner| {
                    spawner.spawn(usb_host::run(usb).expect("spawn the USB host task once"));
                });
        },
    );
    spawner
        .spawn(gt911::run(touch, i2c, backlight_setup).expect("spawn the display I2C task once"));
    spawner.spawn(crate::wifi::run(slot, reset, spawner).expect("spawn Wi-Fi task once"));
    loop {
        let updated = app.update();
        if app.draw(display.framebuffer_mut()).await {
            display.commit();
            if embassy_time::with_timeout(
                Duration::from_millis(100),
                display.wait_for_vsync_async(),
            )
            .await
            .is_err()
            {
                esp_println::println!("Display VSYNC missing; waiting before drawing again");
                display.wait_for_vsync_async().await;
            }
            app.presented(updated);
        }
        Timer::after(Duration::from_millis(if updated { 1 } else { 16 })).await;
    }
}
