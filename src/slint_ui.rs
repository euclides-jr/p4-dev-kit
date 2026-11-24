//! Application UI state: counter and camera preview.
slint::include_modules!();

pub struct Ui {
    pub(crate) demo: DemoScreen,
}

impl Ui {
    pub fn new(demo: DemoScreen, preview_enabled: bool) -> Self {
        demo.set_show_preview(preview_enabled);
        Self { demo }
    }

    pub fn count(&self) -> i32 {
        self.demo.get_count()
    }

    pub fn preview_overlay_position(&self) -> Option<(usize, usize)> {
        self.demo.get_camera_overlay().then(|| {
            (
                self.demo.get_camera_x() as usize,
                self.demo.get_camera_y() as usize,
            )
        })
    }
}
