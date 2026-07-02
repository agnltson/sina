use rerun::{RecordingStreamBuilder, RecordingStream, EncodedImage, Color, Points2D};
use nalgebra::Vector3;
use std::sync::mpsc;

use super::navgraph::NavGraph;
use super::path::Path;
use super::Point;

pub struct Navigator {
     navgraph: NavGraph,
     video_path: Path,
}

impl Navigator {
    pub fn new(filepath: String) -> Self {
        Self {
            navgraph: NavGraph::new(&filepath),
            video_path: Path::from_closed_loop(&filepath),
        }
    }

    pub fn compute_path(&self, start: (f64, f64), end: (f64, f64)) -> Option<Path> {
        let point_path = self.navgraph.find_path(start.into(), end.into())?;
        Some(Path::from_points(point_path))
    }

    pub fn launch(&mut self, record: &RecordingStream) -> anyhow::Result<()> {
        self.log_plan(&record, "navigator")?;
        self.log_videopath(&record, "navigator")?;
        Ok(())
    }

    fn log_plan(&self, record: &RecordingStream, log_path: &str) -> anyhow::Result<()> {
        self.navgraph.log(
            record,
            format!("{}/plan", log_path).as_str(),
            )?;
        Ok(())
    }

    fn log_videopath(&self, record: &RecordingStream, log_path: &str) -> anyhow::Result<()> {
        self.video_path.log(
            record,
            format!("{}/video_path", log_path).as_str(),
            )?;
        Ok(())
    }
}

