use rerun::{RecordingStreamBuilder, RecordingStream, EncodedImage, Color, Points2D};
use nalgebra::Vector3;
use std::sync::mpsc;

use super::navgraph::NavGraph;
use super::path::Path;
use super::Point;

pub struct Navigator {
     navgraph: NavGraph,
     path: Path,
}

impl Navigator {
    pub fn new(filepath: String) -> Self {
        Self {
            navgraph: NavGraph::new(&filepath),
            path: Path::new(&filepath),
        }
    }

    pub fn launch(&mut self, record: RecordingStream) -> anyhow::Result<()> {
        self.log_plan(&record, "navigator")?;
        self.log_path(&record, "navigator")?;
        Ok(())
    }

    fn log_plan(&self, record: &RecordingStream, log_path: &str) -> anyhow::Result<()> {
        self.navgraph.log(
            record,
            format!("{}/plan", log_path).as_str(),
            )?;
        Ok(())
    }

    fn log_path(&self, record: &RecordingStream, log_path: &str) -> anyhow::Result<()> {
        self.path.log(
            record,
            format!("{}/plan", log_path).as_str(),
            )?;
        Ok(())
    }
}

