use rerun::{RecordingStreamBuilder, RecordingStream, EncodedImage, Color, Points2D};
use nalgebra::Vector3;
use std::sync::mpsc;

use super::navgraph::NavGraph;
use super::Point;
use crate::sensor_data::SensorData;

pub struct Navigator {
     navgraph: NavGraph,
     position: Option<Point>,
     path: Option<Vec<usize>>,
}

impl Navigator {
    pub fn new(filepath: String) -> Self {
        Self {
            navgraph: NavGraph::new(&filepath),
            position: Some((0.0, 0.0).into()),
            path: None,
        }
    }

    // This will host all the semantic navigation
    pub fn launch(&mut self, record: RecordingStream, pos_rx: mpsc::Receiver<Point>) -> anyhow::Result<()> {
        self.log_plan(&record, "navigator")?;
        loop {
            // blocking because nothing to compute if the position is unchanged
            if let Ok(pos) = pos_rx.recv() {
                self.position = Some(pos);
            }
        }

        Ok(())
    }

    fn log_plan(&self, record: &RecordingStream, log_path: &str) -> anyhow::Result<()> {
        self.navgraph.log(
            record,
            format!("{}/plan", log_path).as_str(),
            )?;
        Ok(())
    }
}

