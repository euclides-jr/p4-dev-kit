use core::cell::RefCell;

pub const CAPACITY: usize = 768 * 1024;
const TCP_RX_BYTES: usize = 64 * 1024;
const MESSAGE_BYTES: usize = 16 * 1024;
pub const PSRAM_BYTES: usize = CAPACITY + TCP_RX_BYTES + 2 * MESSAGE_BYTES;

pub struct Buffers {
    pub tcp_rx: &'static mut [u8],
    pub frame: &'static mut [u8],
    pub plaintext: &'static mut [u8],
}

static BUFFERS: critical_section::Mutex<RefCell<Option<Buffers>>> =
    critical_section::Mutex::new(RefCell::new(None));

pub fn take_buffers() -> Option<Buffers> {
    critical_section::with(|cs| BUFFERS.borrow_ref_mut(cs).take())
}
pub const FRAME_BYTES: usize = 4;
pub const BYTES_PER_SECOND: i64 = crate::audio::SAMPLE_RATE as i64 * FRAME_BYTES as i64;

struct Queue {
    bytes: &'static mut [u8],
    read: usize,
    len: usize,
    first_time: i64,
    generation: u32,
    ended: bool,
}

static QUEUE: critical_section::Mutex<RefCell<Option<Queue>>> =
    critical_section::Mutex::new(RefCell::new(None));

pub fn init(memory: &'static mut [u8]) {
    let (bytes, buffers) = memory.split_at_mut(CAPACITY);
    let (tcp_rx, messages) = buffers.split_at_mut(TCP_RX_BYTES);
    let (frame, plaintext) = messages.split_at_mut(MESSAGE_BYTES);
    critical_section::with(|cs| {
        BUFFERS.borrow_ref_mut(cs).replace(Buffers {
            tcp_rx,
            frame,
            plaintext,
        });
        QUEUE.borrow_ref_mut(cs).replace(Queue {
            bytes,
            read: 0,
            len: 0,
            first_time: 0,
            generation: 0,
            ended: true,
        });
    });
}

pub fn enabled() -> bool {
    critical_section::with(|cs| QUEUE.borrow_ref(cs).is_some())
}

pub fn reset() {
    critical_section::with(|cs| {
        if let Some(queue) = QUEUE.borrow_ref_mut(cs).as_mut() {
            queue.len = 0;
            queue.ended = false;
            queue.generation = queue.generation.wrapping_add(1);
        }
    });
}

pub fn end() {
    critical_section::with(|cs| {
        if let Some(queue) = QUEUE.borrow_ref_mut(cs).as_mut() {
            queue.ended = true;
        }
    });
}

pub fn push(data: &[u8], timestamp: i64) -> bool {
    critical_section::with(|cs| {
        let mut slot = QUEUE.borrow_ref_mut(cs);
        let Some(queue) = slot.as_mut() else {
            return false;
        };
        if data.is_empty() || !data.len().is_multiple_of(FRAME_BYTES) {
            return false;
        }
        if queue.len != 0 {
            let expected = queue.first_time.saturating_add(duration(queue.len));
            if expected.abs_diff(timestamp) > 15_000 {
                queue.len = 0;
                queue.generation = queue.generation.wrapping_add(1);
            }
        }
        if data.len() > queue.bytes.len() - queue.len {
            return false;
        }
        if queue.len == 0 {
            queue.first_time = timestamp;
        }
        let write = (queue.read + queue.len) % queue.bytes.len();
        let first = data.len().min(queue.bytes.len() - write);
        queue.bytes[write..write + first].copy_from_slice(&data[..first]);
        queue.bytes[..data.len() - first].copy_from_slice(&data[first..]);
        queue.len += data.len();
        true
    })
}

pub struct Status {
    pub len: usize,
    pub first_time: i64,
    pub generation: u32,
    pub ended: bool,
}

pub fn status() -> Option<Status> {
    critical_section::with(|cs| {
        QUEUE.borrow_ref(cs).as_ref().map(|queue| Status {
            len: queue.len,
            first_time: queue.first_time,
            generation: queue.generation,
            ended: queue.ended,
        })
    })
}

pub fn pop(target: &mut [u8]) -> usize {
    critical_section::with(|cs| {
        let mut slot = QUEUE.borrow_ref_mut(cs);
        let Some(queue) = slot.as_mut() else { return 0 };
        let count = target.len().min(queue.len);
        let first = count.min(queue.bytes.len() - queue.read);
        target[..first].copy_from_slice(&queue.bytes[queue.read..queue.read + first]);
        target[first..count].copy_from_slice(&queue.bytes[..count - first]);
        consume(queue, count);
        count
    })
}

pub fn discard_before(now: i64) {
    critical_section::with(|cs| {
        if let Some(queue) = QUEUE.borrow_ref_mut(cs).as_mut() {
            let late = now.saturating_sub(queue.first_time).max(0);
            let frames = late.saturating_mul(BYTES_PER_SECOND / FRAME_BYTES as i64) / 1_000_000;
            let bytes = frames.saturating_mul(FRAME_BYTES as i64);
            consume(queue, (bytes as usize).min(queue.len));
        }
    });
}

fn consume(queue: &mut Queue, count: usize) {
    queue.read = (queue.read + count) % queue.bytes.len();
    queue.len -= count;
    queue.first_time = queue.first_time.saturating_add(duration(count));
}

fn duration(bytes: usize) -> i64 {
    bytes as i64 * 1_000_000 / BYTES_PER_SECOND
}
