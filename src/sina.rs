use nalgebra::Vector3;
use rerun::{RecordingStream, RecordingStreamBuilder, Points2D, Color};

use crate::navigation;

pub struct Sina {
    navigator: navigation::Navigator,
}

impl Sina {
    pub fn new(semantic_path: String) -> Self {
        Self {
            navigator: navigation::Navigator::new(semantic_path),
        }
    }

    pub fn launch(&mut self, start: (f64, f64), end: (f64, f64)) -> anyhow::Result<()> {
        let record: RecordingStream = RecordingStreamBuilder::new("SINA").spawn()?;
        self.navigator.launch(&record)?;
        if let Some(p) = self.navigator.compute_path(start, end) {
            p.log(&record, "path")?;
        }
        Ok(())
    }

    pub fn path(&self) {
    }
}
