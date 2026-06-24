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
            self.log_position(&record, "navigator/position")?;
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

    fn log_position(
        &self,
        record: &RecordingStream,
        log_path: &str,
    ) -> anyhow::Result<()> {
        if let Some(pos) = self.position {
            //println!("New pos logged {:?}", pos);
            let (x, y): (f64, f64) = pos.into();

            record.log(
                format!("{}/position", log_path).as_str(),
                &Points2D::new([[x as f32, y as f32]])
                    .with_colors([Color::from_rgb(255, 0, 0)])
                    .with_radii([0.15]),
            )?;
        }

        Ok(())
    }
}

