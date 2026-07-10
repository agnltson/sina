use std::{
    sync::mpsc,
    thread,
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


use crate::{
    sensor_data::{
        SensorData,
        MagMessage,
        ImageMessage,
    },
    config::{
        Config,
    },
};

#[derive(Debug)]
pub struct PosSys {
    position: Option<Vector3<f64>>,
    orientation: UnitQuaternion<f64>,
    image_tx: mpsc::SyncSender<ImageMessage>,
    pose_rx: mpsc::Receiver<(Vector3<f64>, UnitQuaternion<f64>)>,
}

impl PosSys {
    pub fn new(config: Config) -> Self {
        let mut tag_world_poses = HashMap::new();
        tag_world_poses.insert(0, tag_pose_upright(
            Vector3::new(2.5, 0.0, 1.2),
            Vector3::new(0.0, 1.0, 0.0),
        ));

        // Size one, if we receive a new image before the worker finished working the waiting image
        // is replaced with the last image received.
        let (image_tx, image_rx) = mpsc::sync_channel::<ImageMessage>(1);
        let (pose_tx, pose_rx) = mpsc::channel::<(Vector3<f64>, UnitQuaternion<f64>)>();

        thread::spawn(move || {
            Self::start_worker(&config, image_rx, pose_tx, &tag_world_poses);
        });

        Self {
            position: None,
            orientation: UnitQuaternion::identity(),
            image_tx,
            pose_rx,
        }
    }

    fn start_worker(
        config: &Config,
        image_rx: mpsc::Receiver<ImageMessage>,
        pose_tx: mpsc::Sender<(Vector3<f64>, UnitQuaternion<f64>)>,
        tag_world_poses: &HashMap<u32, Isometry3<f64>>
    ) {
            let mut detector = build_detector(config);
            let tag_params = tag_params(config);

            while let Ok(image) = image_rx.recv() {
                if let Some(pose) = detect_pose(&mut detector, &tag_params, tag_world_poses, &image) {
                    let _ = pose_tx.send(pose);
                }
            }
    }

    pub fn launch(
        &mut self,
        record: RecordingStream,
        sensor_rx: mpsc::Receiver<SensorData>,
        position_tx: mpsc::Sender<(Vector3<f64>, UnitQuaternion<f64>)>,
    ) -> anyhow::Result<()> {
        loop {
            match sensor_rx.recv() {
                Ok(SensorData::Image(image)) => {
                    self.log_image(&record, "camera", image.jpeg.clone())?;
                    let _ = self.image_tx.try_send(image);
                }
                Ok(_) => {}
                Err(_) => break,
            }

            while let Ok((pos, orient)) = self.pose_rx.try_recv() {
                self.position = Some(pos);
                self.orientation = orient;
            }

            if let Some(pos) = self.position {
                position_tx.send((pos, self.orientation))?;
            }
        }
        Ok(())
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

fn detect_pose(
    detector: &mut Detector,
    tag_params: &TagParams,
    tag_world_poses: &HashMap<u32, Isometry3<f64>>,
    image: &ImageMessage,
) -> Option<(Vector3<f64>, UnitQuaternion<f64>)> {
    let dyn_img = image::load_from_memory(&image.jpeg).ok()?;
    let gray = dyn_img.to_luma8();
    let apriltag_img = apriltag::Image::from_image_buffer(&gray);
    let detections = detector.detect(&apriltag_img);

    for det in detections {
        let tag_id = det.id() as u32;
        let Some(tag_world_pose) = tag_world_poses.get(&tag_id) else {
            continue;
        };
        let Some(pose) = det.estimate_tag_pose(tag_params) else {
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

        let camera_world_pose = tag_world_pose * camera_from_tag.inverse();

        return Some((camera_world_pose.translation.vector, camera_world_pose.rotation));
    }

    None
}

pub fn tag_params(config: &Config) -> TagParams {
    TagParams {
        tagsize: config.apriltag.tag_size_m,
        fx: config.streaming.fx,
        fy: config.streaming.fy,
        cx: config.streaming.cx,
        cy: config.streaming.cy,
    }
}

pub fn build_detector(config: &Config) -> Detector {
    let family = match config.apriltag.tag_family.as_str() {
        "tag_16h5" => Family::tag_16h5(),
        "tag_25h9" => Family::tag_25h9(),
        "tag_36h11" => Family::tag_36h11(),
        "tag_circle_21h7" => Family::tag_circle_21h7(),
        "tag_circle_49h12" => Family::tag_circle_49h12(),
        "tag_custom_48h12" => Family::tag_custom_48h12(),
        "tag_standard_41h12" => Family::tag_standard_41h12(),
        "tag_standard_52h13" => Family::tag_standard_52h13(),
        other => panic!("Unknow AprilTag family : {other}.\n
            Available families are:\n
            \t- tag_16h5\n
            \t- tag_25h9\n
            \t- tag_36h11\n
            \t- tag_circle_21h7\n
            \t- tag_circle_49h12\n
            \t- tag_custom_48h12\n
            \t- tag_standard_41h12\n
            \t- tag_standard_52h13
            "),
    };

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
