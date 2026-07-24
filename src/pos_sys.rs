use std::{
    sync::mpsc,
    thread,
    path::Path,
    fs,
    collections::HashMap
};
use rerun::RecordingStream;
use nalgebra::{
    Vector2,
    Vector3,
    UnitQuaternion,
    Quaternion,
    Isometry3,
    Translation3,
    Matrix3,
};
use serde::Deserialize;

use crate::{
    apriltag_ffi::{Detector, TagParams},
    sensor_data::{
        SensorData,
        ImageMessage,
    },
    config::{
        Config,
    },
};

const POSE_ESTIMATION_ITERS: i32 = 50;

const QUAT_NORM_MIN: f64 = 0.5;
const QUAT_NORM_MAX: f64 = 2.0;

#[derive(Debug, Deserialize)]
struct TagWorldConfig {
    tags: Vec<TagWorldEntry>,
}

#[derive(Debug, Deserialize)]
struct TagWorldEntry {
    id: u32,
    position_world: [f64; 3],
    orientation_world_wxyz: [f64; 4],
    #[allow(dead_code)]
    #[serde(default)]
    num_observations: u32,
    #[allow(dead_code)]
    #[serde(default)]
    position_std_m: [f64; 3],
    #[allow(dead_code)]
    #[serde(default)]
    best_reprojection_error: f64,
}


#[derive(Debug)]
pub struct PosSys {
    position: Option<Vector3<f64>>,
    orientation: UnitQuaternion<f64>,
    image_tx: mpsc::SyncSender<ImageMessage>,
    pose_rx: mpsc::Receiver<(Vector3<f64>, UnitQuaternion<f64>)>,
    worker_handle: thread::JoinHandle<()>,
    config: Config,
    tag_world_poses: HashMap<u32, Isometry3<f64>>,
}

impl PosSys {
    pub fn new(config: Config, data_path: String) -> Self {
        let full_path = format!("{}/tags_world_config.json", data_path);
        let tag_world_poses = match load_tag_world_poses(&full_path) {
            Ok(poses) => {
                println!("Loaded {} tags from {data_path}", poses.len());
                poses
            }
            Err(e) => {
                eprintln!("Unable to load tags from {data_path}: {e}");
                HashMap::new()
            }
        };

        let (worker_handle, image_tx, pose_rx) = Self::spawn_worker(config.clone(), tag_world_poses.clone());

        Self {
            position: None,
            orientation: UnitQuaternion::identity(),
            image_tx,
            pose_rx,
            worker_handle,
            config,
            tag_world_poses,
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
                println!("Worker is running");
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
        if cfg!(debug_assertions) {
            self.log_tags(&record, "navigator/anchors")?;
        }
        loop {
            match sensor_rx.recv() {
                Ok(SensorData::Image(image)) => {
                    if cfg!(debug_assertions) {
                        self.log_image(&record, "camera", image.jpeg.clone())?;
                    }

                    if self.worker_handle.is_finished() {
                        eprintln!("Restarting AprilTag worker...");
                        let (handle, image_tx, pose_rx) =
                            Self::spawn_worker(self.config.clone(), self.tag_world_poses.clone());
                        self.worker_handle = handle;
                        self.image_tx = image_tx;
                        self.pose_rx = pose_rx;
                    }

                    let _ = self.image_tx.try_send(image);
                }
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

    fn spawn_worker(
        config: Config,
        tag_world_poses: HashMap<u32, Isometry3<f64>>,
    ) -> (
        thread::JoinHandle<()>,
        mpsc::SyncSender<ImageMessage>,
        mpsc::Receiver<(Vector3<f64>, UnitQuaternion<f64>)>,
    ) {
        // Size one, if we receive a new image before the worker finished working the waiting image
        // is replaced with the last image received.
        let (image_tx, image_rx) = mpsc::sync_channel::<ImageMessage>(1);
        let (pose_tx, pose_rx) = mpsc::channel::<(Vector3<f64>, UnitQuaternion<f64>)>();

        let handle = thread::spawn(move || {
            PosSys::start_worker(&config, image_rx, pose_tx, &tag_world_poses);
        });

        (handle, image_tx, pose_rx)
    }

    #[cfg(debug_assertions)]
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

    #[cfg(debug_assertions)]
    fn log_tags(
        &self,
        rec: &RecordingStream,
        log_path: &str,
        ) -> anyhow::Result<()> {
        const ARROW_LENGTH: f64 = 0.3;

        for (tag_id, pose) in &self.tag_world_poses {
            let position = pose.translation.vector;
            let facing = pose.rotation.transform_vector(&Vector3::new(0.0, 0.0, -1.0)) * ARROW_LENGTH;

            let facing_xy = Vector2::new(facing.x, facing.y);
            let facing_xy = if facing_xy.norm() > 1e-6 {
                facing_xy.normalize() * ARROW_LENGTH
            } else {
                Vector2::new(0.0, 0.0)
            };

            rec.log(
                format!("{}/tags/{tag_id}", log_path).as_str(),
                &rerun::Points2D::new([[position.x as f32, position.y as f32]])
                    .with_colors([rerun::Color::from_rgb(255, 165, 0)])
                    .with_radii([0.05]),
            )?;

            rec.log(
                format!("{}/tags/{tag_id}/facing", log_path).as_str(),
                &rerun::Arrows2D::from_vectors([[facing_xy.x as f32, facing_xy.y as f32]])
                    .with_origins([[position.x as f32, position.y as f32]])
                    .with_colors([rerun::Color::from_rgb(255, 165, 0)]),
            )?;
        }

        Ok(())
    }
}

pub fn load_tag_world_poses<P: AsRef<Path>>(
    path: P,
) -> anyhow::Result<HashMap<u32, Isometry3<f64>>> {
    let raw = fs::read_to_string(path)?;
    let parsed: TagWorldConfig = serde_json::from_str(&raw)?;

    let mut tag_world_poses = HashMap::new();

    for entry in parsed.tags {
        let [qw, qx, qy, qz] = entry.orientation_world_wxyz;

        if ![qw, qx, qy, qz].iter().all(|v| v.is_finite())
            || !entry.position_world.iter().all(|v| v.is_finite())
        {
            eprintln!("Tag {}: position/orientation non-finite, skipped", entry.id);
            continue;
        }

        let norm = (qw * qw + qx * qx + qy * qy + qz * qz).sqrt();
        if !(QUAT_NORM_MIN..=QUAT_NORM_MAX).contains(&norm) {
            eprintln!(
                "Tag {}: corrupted quaternion (norm={norm:.3}), skipped",
                entry.id
            );
            continue;
        }

        let orientation = UnitQuaternion::from_quaternion(Quaternion::new(qw, qx, qy, qz));
        let position = Vector3::new(
            entry.position_world[0],
            entry.position_world[1],
            entry.position_world[2],
        );

        let pose = Isometry3::from_parts(Translation3::from(position), orientation);
        tag_world_poses.insert(entry.id, pose);
    }

    Ok(tag_world_poses)
}

fn detect_pose(
    detector: &mut Detector,
    tag_params: &TagParams,
    tag_world_poses: &HashMap<u32, Isometry3<f64>>,
    image: &ImageMessage,
) -> Option<(Vector3<f64>, UnitQuaternion<f64>)> {
    let dyn_img = image::load_from_memory(&image.jpeg).ok()?;
    let gray = dyn_img.to_luma8();
    let (width, height) = gray.dimensions();
    // `image::GrayImage` (`ImageBuffer`) est un buffer compact : stride == width.
    let detections = detector.detect(gray.as_raw(), width as i32, height as i32, width as i32)?;

    println!("Detected: {} tag(s)", detections.len());
    for det in detections.iter() {
        let tag_id = det.id() as u32;
        let Some(tag_world_pose) = tag_world_poses.get(&tag_id) else {
            continue;
        };

        let Some(best) = det.estimate_pose_orthogonal(tag_params, POSE_ESTIMATION_ITERS) else {
            continue;
        };

        let rot_mat = Matrix3::from_row_slice(&best.rotation);
        let det_val = rot_mat.determinant();
        if !det_val.is_finite() || (det_val - 1.0).abs() > 0.1 {
            eprintln!("Invalid rotation matrix (det={det_val}), ignored");
            continue;
        }
        let translation = Vector3::new(best.translation[0], best.translation[1], best.translation[2]);

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
    match config.apriltag.tag_family.as_str() {
        "tag16h5" | "tag25h9" | "tag36h11" | "tagcircle21h7" | "tagcircle49h12"
        | "tagcustom48h12" | "tagstandard41h12" | "tagstandard52h13" => {}
        other => panic!("Unknow AprilTag family : {other}.\n
            Available families are:\n
            \t- tag16h5\n
            \t- tag25h9\n
            \t- tag36h11\n
            \t- tagcircle21h7\n
            \t- tagcircle49h12\n
            \t- tagcustom48h12\n
            \t- tagstandard41h12\n
            \t- tagstandard52h13
            "),
    };

    Detector::new(
        &config.apriltag.tag_family,
        num_cpus::get() as u8,
        2.0,
        0.0,
        true,
    )
    .expect("AprilTag Detector build failed")
}
