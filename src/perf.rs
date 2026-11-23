use core::sync::atomic::{AtomicU32, Ordering};
use esp_alloc::{HEAP, MemoryCapability};
use esp_hal::{system::Cpu, time::Instant};
use rtos_trace::{RtosTrace, TaskInfo};

static ACTIVE_SINCE: [AtomicU32; 2] = [const { AtomicU32::new(0) }; 2];
static SCHEDULED_US: [AtomicU32; 2] = [const { AtomicU32::new(0) }; 2];

fn clock_us() -> u64 {
    Instant::now().duration_since_epoch().as_micros()
}

struct TaskTrace;

rtos_trace::global_trace!(TaskTrace);

impl RtosTrace for TaskTrace {
    fn task_exec_begin(_id: u32) {
        ACTIVE_SINCE[Cpu::current() as usize].store(clock_us() as u32, Ordering::Relaxed);
    }

    fn task_exec_end() {
        let core = Cpu::current() as usize;
        let start = ACTIVE_SINCE[core].swap(0, Ordering::Relaxed);
        if start != 0 {
            SCHEDULED_US[core]
                .fetch_add((clock_us() as u32).wrapping_sub(start), Ordering::Relaxed);
        }
    }

    fn start() {}
    fn stop() {}
    fn task_new(_id: u32) {}
    fn task_new_stackless(_id: u32, _name: &'static str, _priority: u32) {}
    fn task_terminate(_id: u32) {}
    fn task_send_info(_id: u32, _info: TaskInfo) {}
    fn task_ready_begin(_id: u32) {}
    fn task_ready_end(_id: u32) {}
    fn system_idle() {}
    fn isr_enter() {}
    fn isr_exit() {}
    fn isr_exit_to_scheduler() {}
    fn name_marker(_id: u32, _name: &'static str) {}
    fn marker(_id: u32) {}
    fn marker_begin(_id: u32) {}
    fn marker_end(_id: u32) {}
}

fn snapshot() -> (u64, [u32; 2]) {
    let now = clock_us();
    let used = core::array::from_fn(|core| SCHEDULED_US[core].load(Ordering::Relaxed));
    (now, used)
}

pub struct Sampler {
    time_us: u64,
    scheduled_us: [u32; 2],
    frames: u32,
    warmed_up: bool,
}

impl Sampler {
    pub fn new(frames: u32) -> Self {
        let (time_us, scheduled_us) = snapshot();
        Self {
            time_us,
            scheduled_us,
            frames,
            warmed_up: false,
        }
    }

    pub fn report(&mut self, frames: u32) {
        if clock_us() - self.time_us < 5_000_000 {
            return;
        }
        let (now, scheduled) = snapshot();
        let elapsed = now - self.time_us;
        if !self.warmed_up {
            // The first completed task interval can include time before the sampler started.
            self.warmed_up = true;
            self.time_us = now;
            self.scheduled_us = scheduled;
            self.frames = frames;
            return;
        }
        let core0 = u64::from(scheduled[0].wrapping_sub(self.scheduled_us[0])) * 1000 / elapsed;
        let core1 = u64::from(scheduled[1].wrapping_sub(self.scheduled_us[1])) * 1000 / elapsed;
        let stats = HEAP.stats();
        let mut sram = (0, 0);
        let mut psram = (0, 0);
        for region in stats.region_stats.iter().flatten() {
            let target = if region.capabilities.contains(MemoryCapability::Internal) {
                &mut sram
            } else {
                &mut psram
            };
            target.0 += region.used;
            target.1 += region.size;
        }
        esp_println::println!(
            "Perf: {} ms, {} frames, scheduled CPU {core0}/{core1} permille, SRAM {}/{}, PSRAM {}/{} bytes",
            elapsed / 1000,
            frames - self.frames,
            sram.0,
            sram.1,
            psram.0,
            psram.1,
        );
        self.time_us = now;
        self.scheduled_us = scheduled;
        self.frames = frames;
    }
}
