use rerun::{RecordingStreamBuilder, RecordingStream, EncodedImage, Color, Points2D};
use nalgebra::Vector3;
use std::sync::mpsc::Receiver;

use super::navgraph::NavGraph;
use super::utils::Point;
use crate::navigation::VisualisationData;
use crate::loggable::Loggable;

pub struct Navigator {
     navgraph: NavGraph,
     position: Option<Point>,
     path: Option<Vec<usize>>,
     rec: RecordingStream,
}

impl Navigator {
    pub fn new(filepath: &str) -> Self {
        Self {
            navgraph: NavGraph::new(filepath),
            position: Some((0.0, 0.0).into()),
            path: None,
            rec: RecordingStreamBuilder::new("SIMA").spawn().unwrap(),

        }
    }

    pub fn run(&mut self, visual_data_receiver: Receiver<VisualisationData>) -> Result<(), Box<dyn std::error::Error>> {

        let log_path = "navigator/";

        let _ = self.navgraph.render_rerun(&self.rec, log_path);

        while let Ok(visual_data) = visual_data_receiver.recv() {
            match visual_data {
                VisualisationData::Position(pos_v3) => {
                    let p: Point = (pos_v3.x, pos_v3.y).into();
                    self.position = Some(p);
                    p.render_rerun(&self.rec, &(String::from(log_path) + "position"))?;
                },
                VisualisationData::LeftImage(jpeg, features) => render_image_with_features(&self.rec, "camera/LeftImage", jpeg, features)?,
                VisualisationData::RightImage(jpeg, features) => render_image_with_features(&self.rec, "camera/RightImage", jpeg, features)?,
            }

            if let Some(path) = &self.path {
                let _ = self.navgraph.render_rerun_path(&path, &self.rec, log_path);
            }
        }
        Ok(())
    }
}

pub fn render_image_with_features(
    rec: &RecordingStream,
    log_path: &str,
    jpeg: Vec<u8>,
    features: Vec<[f32; 2]>,
) -> Result<(), Box<dyn std::error::Error>> {
    rec.log(log_path, &EncodedImage::from_file_contents(jpeg))?;

    if !features.is_empty() {
        rec.log(
            format!("{}/features", log_path),
            &Points2D::new(features)
                .with_radii([3.0])
                .with_colors([Color::from_rgb(0, 255, 0)]),
        )?;
    }

    Ok(())
}
