#![no_std]
#![no_main]
#![deny(clippy::mem_forget, clippy::large_stack_frames)]

extern crate alloc;

#[path = "../waveshare_5dsi_touch_a.rs"]
mod waveshare_5dsi_touch_a;

#[path = "../gt911.rs"]
mod gt911;

#[path = "../slint_platform.rs"]
mod slint_platform;

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

#[path = "../preview.rs"]
mod preview;

#[path = "../app.rs"]
mod app;

#[path = "../board.rs"]
mod board;

#[path = "../sendspin.rs"]
mod sendspin;

#[path = "../wifi.rs"]
mod wifi;

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    esp_println::println!("{info}");
    loop {}
}

esp_bootloader_esp_idf::esp_app_desc!();

#[esp_rtos::main]
async fn main(spawner: embassy_executor::Spawner) -> ! {
    board::run(spawner).await
}
