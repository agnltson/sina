use nalgebra::Vector3;
use rerun::{RecordingStream, RecordingStreamBuilder, Points2D, Color};

use crate::navigation;

pub struct Sina {
}

impl Sina {
    pub fn new() -> Self {
        Self {
        }
    }

    pub fn launch(&mut self, semantic_path: String) -> anyhow::Result<()> {
        let record: RecordingStream = RecordingStreamBuilder::new("SINA").spawn()?;
        navigation::Navigator::new(semantic_path).launch(record)?;
        Ok(())
    }
}
