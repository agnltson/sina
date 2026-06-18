use opencv::{
    core::{Mat, Point2f, Vector},
    imgcodecs,
    imgproc,
};
use rerun::{RecordingStreamBuilder, RecordingStream, EncodedImage, Color, Points2D};
use nalgebra::Vector3;
use std::sync::mpsc::Receiver;

use super::navgraph::NavGraph;
use super::utils::Point;
use crate::navigation::VisualisationData;

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
                VisualisationData::Position(pos_v3, timestamp) => {
                    let p: Point = (pos_v3.x, pos_v3.y).into();
                    self.position = Some(p);
                    p.render_rerun(&self.rec, &(String::from(log_path) + "position"))?;
                },
                VisualisationData::LeftImage(jpeg, timestamp) => render_image_with_features(&self.rec, "camera/LeftImage", jpeg)?,
                VisualisationData::RightImage(jpeg, timestamp) => render_image_with_features(&self.rec, "camera/RightImage", jpeg)?,
            }

            if let Some(path) = &self.path {
                let _ = self.navgraph.render_rerun_path(&path, &self.rec, log_path);
            }
        }
        Ok(())
    }
}

fn render_image(
    rec: &RecordingStream,
    log_path: &str,
    jpeg: Vec<u8>,
) -> Result<(), Box<dyn std::error::Error>> {
    rec.log(
        log_path,
        &EncodedImage::new(jpeg),
    )?;

    Ok(())
}

pub fn render_image_with_features(
    rec: &RecordingStream,
    log_path: &str,
    jpeg: Vec<u8>,
) -> Result<(), Box<dyn std::error::Error>> {
    let buf = Vector::from_slice(&jpeg);
    let frame = imgcodecs::imdecode(&buf, imgcodecs::IMREAD_COLOR)?;

    let mut gray = Mat::default();
    imgproc::cvt_color(&frame, &mut gray, imgproc::COLOR_BGR2GRAY, 0)?;

    // Shi-Tomasi
    let mut corners = Vector::<Point2f>::new();
    imgproc::good_features_to_track(
        &gray,
        &mut corners,
        200,  // max corners
        0.01, // quality level
        10.0, // min distance
        &Mat::default(),
        3,
        false,
        0.04,
    )?;

    // Log image
    rec.log(log_path, &EncodedImage::from_file_contents(jpeg))?;

    // Log features
    let positions: Vec<[f32; 2]> = corners.iter().map(|p| [p.x, p.y]).collect();
    if !positions.is_empty() {
        rec.log(
            format!("{}/features", log_path),
            &Points2D::new(positions)
                .with_radii([3.0])
                .with_colors([Color::from_rgb(0, 255, 0)]),
        )?;
    }

    Ok(())
}
