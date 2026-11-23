use core::future::pending;

use embassy_futures::select::{Either, select};
use embassy_usb_driver::host::DeviceEvent;
use embassy_usb_host::{BusRoute, BusState, bus};
use esp_hal::usb::otg::{Usb, embassy_usb_host::Driver};
use esp_println::println;

#[embassy_executor::task]
pub async fn run(usb: Usb<'static>) {
    let driver = Driver::new(usb);
    let state = BusState::new();
    let (mut controller, handle) = bus(driver, &state);
    let mut config = [0; 4096];
    println!("USB OTG host task started, waiting for a device");

    loop {
        let speed = controller.wait_for_connection().await;
        println!("USB device attached at {speed:?} speed");

        match handle.enumerate(BusRoute::Direct(speed), &mut config).await {
            Ok((info, config_len)) => {
                let descriptor = info.device_desc;
                println!(
                    "USB device enumerated: VID={:04x} PID={:04x} class={:02x}",
                    descriptor.vendor_id, descriptor.product_id, descriptor.device_class,
                );
                let capture = crate::uvc::stream(&handle, &info, &config[..config_len]);
                let monitor_disconnect = async {
                    while controller.wait_for_device_event().await != DeviceEvent::Disconnected {}
                    // The host controller wakes the active pipe on disconnect.
                    // Let capture finish its read before dropping its buffer.
                    pending::<()>().await
                };
                if let Either::First(Err(error)) = select(capture, monitor_disconnect).await {
                    println!("USB camera: {error}");
                }
                crate::uvc::CAMERA_FRAME.signal(None);
                handle.free_address(info.device_address);
            }
            Err(error) => println!("USB device enumeration failed: {error:?}"),
        }
    }
}
