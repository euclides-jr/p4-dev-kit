use alloc::vec::Vec;
use core::alloc::Layout;

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_usb_driver::{
    EndpointAddress, EndpointInfo, EndpointType,
    host::{HostError, PipeError, UsbHostAllocator, UsbPipe, pipe},
};
use embassy_usb_host::{
    BusHandle,
    descriptor::{ConfigurationDescriptor, EndpointDescriptor},
    handler::EnumerationInfo,
};
use esp_alloc::{HEAP, MemoryCapability};
use esp_println::println;

pub const WIDTH: u16 = 1280;
pub const HEIGHT: u16 = 720;
pub const MAX_FRAME_BYTES: usize = 512 * 1024;
const MAX_PACKET_BYTES: usize = 1024;
const TARGET_FRAME_INTERVAL: u32 = 10_000_000 / 15;

pub static CAMERA_FRAME: Signal<CriticalSectionRawMutex, Option<Vec<u8>>> = Signal::new();

fn append_camera_payload(frame: &mut Vec<u8>, data: &[u8]) -> bool {
    let required = frame.len() + data.len();
    if required > MAX_FRAME_BYTES {
        return false;
    }
    if required > frame.capacity() {
        let capacity = required.next_power_of_two().max(16 * 1024);
        let layout = Layout::array::<u8>(capacity).expect("bounded camera buffer layout");
        // SAFETY: capacity is nonzero and bounded. HEAP is the global allocator,
        // so Vec can free the allocation using this same layout.
        let ptr = unsafe { HEAP.alloc_caps(MemoryCapability::External.into(), layout) };
        if ptr.is_null() {
            return false;
        }
        // SAFETY: The allocation is unique and can hold capacity bytes. Its
        // initialized length is zero. Check capacity before every later append
        // so Vec never reallocates the storage into internal SRAM.
        let mut replacement = unsafe { Vec::from_raw_parts(ptr, 0, capacity) };
        replacement.extend_from_slice(frame);
        *frame = replacement;
    }
    frame.extend_from_slice(data);
    true
}

#[derive(Debug)]
pub enum StreamError {
    Host(HostError),
    InvalidProbe,
    UnsupportedBandwidth(u32),
}

impl From<HostError> for StreamError {
    fn from(error: HostError) -> Self {
        Self::Host(error)
    }
}

impl From<PipeError> for StreamError {
    fn from(error: PipeError) -> Self {
        HostError::from(error).into()
    }
}

impl core::fmt::Display for StreamError {
    fn fmt(&self, formatter: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Host(error) => write!(formatter, "{error:?}"),
            Self::InvalidProbe => formatter.write_str("invalid UVC probe response"),
            Self::UnsupportedBandwidth(bytes) => write!(
                formatter,
                "no single-transaction isochronous endpoint for {bytes} bytes per microframe"
            ),
        }
    }
}

struct VideoMode {
    interface: u8,
    format: u8,
    frame: u8,
    interval: [u8; 4],
    probe_len: u8,
}

fn find_mode(config: ConfigurationDescriptor<'_>) -> Option<VideoMode> {
    let mut probe_len = 26;
    for interface in config
        .iter_interface()
        .filter(|i| i.interface_class == 0x0e)
    {
        if interface.interface_subclass == 1 {
            // The VideoControl header contains bcdUVC in bytes 3 and 4.
            for (_, descriptor) in interface.iter_descriptors() {
                if let [_, 0x24, 1, lo, hi, ..] = descriptor {
                    probe_len = match u16::from_le_bytes([*lo, *hi]) {
                        0x0150.. => 48,
                        0x0110.. => 34,
                        _ => 26,
                    };
                }
            }
        }
        if interface.interface_subclass != 2 || interface.alternate_setting != 0 {
            continue;
        }
        let mut mjpeg_format = None;
        for (_, descriptor) in interface.iter_descriptors() {
            match descriptor {
                [_, 0x24, 0x06, index, ..] => mjpeg_format = Some(*index),
                [_, 0x24, 0x04 | 0x10, ..] => mjpeg_format = None,
                [_, 0x24, 0x07, frame, _, w0, w1, h0, h1, ..]
                    if u16::from_le_bytes([*w0, *w1]) == WIDTH
                        && u16::from_le_bytes([*h0, *h1]) == HEIGHT =>
                {
                    let format = mjpeg_format?;
                    let default = descriptor.get(21..25)?.try_into().ok()?;
                    // UVC intervals are in units of 100 ns.
                    let interval = if *descriptor.get(25)? > 0 {
                        descriptor[26..]
                            .chunks_exact(4)
                            .filter_map(|bytes| bytes.try_into().ok())
                            .min_by_key(|bytes| {
                                u32::from_le_bytes(*bytes).abs_diff(TARGET_FRAME_INTERVAL)
                            })
                            .unwrap_or(default)
                    } else {
                        default
                    };
                    return Some(VideoMode {
                        interface: interface.interface_number,
                        format,
                        frame: *frame,
                        interval,
                        probe_len,
                    });
                }
                _ => (),
            }
        }
    }
    None
}

fn stream_endpoint(
    config: ConfigurationDescriptor<'_>,
    interface: u8,
    payload_size: u32,
) -> Option<(u8, EndpointDescriptor)> {
    config
        .iter_interface()
        .filter(|i| i.interface_number == interface && i.alternate_setting != 0)
        .filter_map(|i| {
            let endpoint = i.iter_endpoints().find(|endpoint| {
                endpoint.is_in()
                    && endpoint.ep_type() == EndpointType::Isochronous
                    && endpoint.max_packet_size as usize <= MAX_PACKET_BYTES
                    && u32::from(endpoint.max_packet_size) >= payload_size
            })?;
            Some((i.alternate_setting, endpoint))
        })
        .min_by_key(|(_, endpoint)| endpoint.max_packet_size)
}

pub async fn stream<'d, A: UsbHostAllocator<'d>>(
    bus: &BusHandle<'d, A>,
    device: &EnumerationInfo,
    descriptors: &[u8],
) -> Result<(), StreamError> {
    let config = ConfigurationDescriptor::try_from_slice(descriptors)?;
    let Some(mode) = find_mode(config) else {
        return Ok(());
    };
    let mut control = bus.alloc_pipe::<pipe::Control, pipe::InOut>(
        device.device_address,
        &EndpointInfo {
            addr: EndpointAddress::from(0),
            ep_type: EndpointType::Control,
            max_packet_size: u16::from(device.device_desc.max_packet_size0),
            interval_ms: 0,
        },
        device.split(),
    )?;

    let mut parameters = [0; 48];
    let probe = &mut parameters[..mode.probe_len as usize];
    probe[0] = 1; // bmHint: keep the requested frame interval.
    probe[2] = mode.format;
    probe[3] = mode.frame;
    probe[4..8].copy_from_slice(&mode.interval);
    probe[22..26].copy_from_slice(&(MAX_PACKET_BYTES as u32).to_le_bytes());
    let interface = mode.interface;
    let length = mode.probe_len;
    control
        .control_out(&[0x21, 1, 0, 1, interface, 0, length, 0], probe)
        .await?;
    let received = control
        .control_in(&[0xa1, 0x81, 0, 1, interface, 0, length, 0], probe)
        .await?;
    if received < 26 || probe[2] != mode.format || probe[3] != mode.frame {
        return Err(StreamError::InvalidProbe);
    }
    let payload = u32::from_le_bytes([probe[22], probe[23], probe[24], probe[25]]);
    let interval = u32::from_le_bytes([probe[4], probe[5], probe[6], probe[7]]);
    println!(
        "USB camera probe: format {} frame {} interval {} (100 ns units), payload {}",
        mode.format, mode.frame, interval, payload,
    );
    let (alternate, endpoint) = stream_endpoint(config, interface, payload)
        .or_else(|| {
            // The camera's requested bandwidth can exceed the host driver's
            // one-transaction limit even for its smallest frame mode.
            (payload > MAX_PACKET_BYTES as u32)
                .then(|| stream_endpoint(config, interface, MAX_PACKET_BYTES as u32))
                .flatten()
        })
        .ok_or(StreamError::UnsupportedBandwidth(payload))?;
    if payload > u32::from(endpoint.max_packet_size) {
        probe[22..26].copy_from_slice(&u32::from(endpoint.max_packet_size).to_le_bytes());
        println!(
            "USB camera: limiting payload to {} bytes",
            endpoint.max_packet_size
        );
    }
    control
        .control_out(
            &[0x21, 1, 0, 2, interface, 0, received as u8, 0],
            &probe[..received],
        )
        .await?;
    let mut video = bus.alloc_pipe::<pipe::Isochronous, pipe::In>(
        device.device_address,
        &endpoint.into(),
        device.split(),
    )?;
    control
        .control_out(&[0x01, 0x0b, alternate, 0, interface, 0, 0, 0], &[])
        .await?;
    drop(control);
    println!(
        "USB camera: streaming {WIDTH}x{HEIGHT} MJPEG on alternate {alternate}, packet {}",
        endpoint.max_packet_size,
    );

    let mut packet = [0; MAX_PACKET_BYTES];
    let mut jpeg = Vec::new();
    let mut frame_id = None;
    let mut reported_frame = false;
    let mut capture_started = embassy_time::Instant::now();
    let mut captured_frames = 0_u32;
    let mut captured_bytes = 0_u32;
    loop {
        let received = match video
            .request_in(&mut packet[..endpoint.max_packet_size as usize])
            .await
        {
            Ok(received) => received,
            Err(PipeError::Timeout | PipeError::BadResponse | PipeError::DataToggleError) => {
                jpeg.clear();
                continue;
            }
            Err(error) => return Err(error.into()),
        };
        let [header_len, flags, ..] = packet[..received] else {
            continue;
        };
        let id = flags & 1;
        if frame_id.replace(id) != Some(id) {
            jpeg.clear();
        }
        if header_len < 2 || flags & 0x40 != 0 {
            jpeg.clear();
            continue;
        }
        let Some(data) = packet[..received].get(header_len as usize..) else {
            jpeg.clear();
            continue;
        };
        if jpeg.is_empty() && !data.starts_with(&[0xff, 0xd8]) {
            continue;
        }
        if !append_camera_payload(&mut jpeg, data) {
            jpeg.clear();
            continue;
        }
        if flags & 2 != 0 {
            if jpeg.ends_with(&[0xff, 0xd9]) {
                if !reported_frame {
                    println!("USB camera: received first JPEG ({} bytes)", jpeg.len());
                    reported_frame = true;
                }
                captured_frames += 1;
                captured_bytes += jpeg.len() as u32;
                let elapsed_ms = capture_started.elapsed().as_millis();
                if elapsed_ms >= 5000 {
                    println!(
                        "USB capture: {captured_frames} JPEG frames, {captured_bytes} bytes, {elapsed_ms} ms"
                    );
                    capture_started = embassy_time::Instant::now();
                    captured_frames = 0;
                    captured_bytes = 0;
                }
                CAMERA_FRAME.signal(Some(core::mem::take(&mut jpeg)));
            }
            jpeg.clear();
        }
    }
}
