use std::{
    sync::mpsc,
};
use std::collections::HashMap;
use rerun::{RecordingStream, EncodedImage};
use nalgebra::{
    Vector3,
    UnitQuaternion,
    Isometry3,
    Translation3,
    Matrix3,
};
use apriltag::{
    Detector,
    DetectorBuilder,
    Family,
    pose::TagParams,
};
use apriltag_image::ImageExt;


use crate::sensor_data::{
    SensorData,
    MagMessage,
    ImageMessage,
};

#[derive(Debug)]
pub struct PosSys {
    position: Option<Vector3<f64>>,
    tag_world_poses: HashMap<u32, Isometry3<f64>>,
    detector: Detector,
    tag_params: TagParams,
}

impl PosSys {
    pub fn new() -> Self {

        let mut apriltag = HashMap::new();
        apriltag.insert(0, tag_pose_upright(
                Vector3::new(2.5, 0.0, 1.2),
                Vector3::new(0.0, 1.0, 0.0),
            ));

        Self {
            position: None,
            tag_world_poses: apriltag,
            detector: build_detector(),
            tag_params: tag_params(),
        }
    }

    pub fn launch(
        &mut self,
        record: RecordingStream,
        sensor_rx: mpsc::Receiver<SensorData>,
        position_tx: mpsc::Sender<Vector3<f64>>
    ) -> anyhow::Result<()> {
        loop {
            if let Ok(data) = sensor_rx.recv() {
                match data {
                    SensorData::Image(image) => {
                        self.update_from_image(&image);
                        self.log_image(&record, "camera", image.jpeg)?;
                    },
                    _ => (),
                }
            }
            if let Some(pos) = self.position {
                position_tx.send(pos)?;
            }
        }
    }

    fn update_from_image(&mut self, image: &ImageMessage) {
        let dyn_img = match image::load_from_memory(&image.jpeg) {
            Ok(img) => img,
            Err(_) => return,
        };

        let gray = dyn_img.to_luma8();

        let apriltag_img = match apriltag::Image::from_image_buffer(&gray) {
            img => img,
        };

        let detections = self.detector.detect(&apriltag_img);

        for det in detections {
            let tag_id = det.id() as u32;

            let Some(tag_world_pose) = self.tag_world_poses.get(&tag_id) else {
                continue;
            };

            let Some(pose) = det.estimate_tag_pose(&self.tag_params) else {
                continue;
            };

            let r = pose.rotation();
            let t = pose.translation();

            let rot_mat = Matrix3::from_row_slice(r.data());
            let translation = Vector3::new(t.data()[0], t.data()[1], t.data()[2]);

            let camera_from_tag = Isometry3::from_parts(
                Translation3::from(translation),
                UnitQuaternion::from_matrix(&rot_mat),
            );

            let tag_from_camera = camera_from_tag.inverse();

            let camera_world_pose = tag_world_pose * tag_from_camera;

            self.position = Some(camera_world_pose.translation.vector);
        }
    }

    pub fn log_image(
        &self,
        rec: &RecordingStream,
        log_path: &str,
        jpeg: Vec<u8>,
    ) -> anyhow::Result<()> {
        rec.log(
            format!("{}/camera/image", log_path).as_str(),
            &rerun::EncodedImage::from_file_contents(jpeg),
        )?;
        Ok(())
    }
}

const CAM_FX: f64 = 450.0;
const CAM_FY: f64 = 450.0;
const CAM_CX: f64 = 704.0; // PINHOLE_WIDTH / 2
const CAM_CY: f64 = 704.0; // PINHOLE_HEIGHT / 2

const TAG_SIZE_M: f64 = 0.16; // in meter

pub fn tag_params() -> TagParams {
    TagParams {
        tagsize: TAG_SIZE_M,
        fx: CAM_FX,
        fy: CAM_FY,
        cx: CAM_CX,
        cy: CAM_CY,
    }
}

pub fn build_detector() -> Detector {
    let family = Family::tag_36h11();

    let mut detector = DetectorBuilder::new()
        .add_family_bits(family, 1)
        .build()
        .expect("AprilTag Detector build failed");

    detector.set_thread_number(num_cpus::get() as u8);
    detector.set_decimation(2.0);
    detector.set_refine_edges(true);

    detector
}

fn tag_pose_upright(position: Vector3<f64>, facing_direction: Vector3<f64>) -> Isometry3<f64> {
    let world_up = Vector3::new(0.0, 0.0, 1.0);

    let z_axis = facing_direction.normalize();
    let x_axis = world_up.cross(&z_axis).normalize();
    let y_axis = z_axis.cross(&x_axis).normalize();

    let rot_mat = Matrix3::from_columns(&[x_axis, y_axis, z_axis]);
    let orientation = UnitQuaternion::from_matrix(&rot_mat);

    Isometry3::from_parts(Translation3::from(position), orientation)
}
