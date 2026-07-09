use apriltag::{Detector, DetectorBuilder, Family, Image, TagParams};
use crossbeam_channel::Receiver;
use nalgebra::{Isometry3, Matrix3, Rotation3, Translation3, UnitQuaternion, Vector3};

use super::decoder::DecodedFrame;
use super::pose::{interpolate_pose, PoseSample};

#[derive(Debug, Clone)]
pub struct TagObservation {
    pub frame_index: usize,
    pub tag_id: usize,
    pub translation_world: [f64; 3],
    pub quaternion_world_wxyz: [f64; 4],
    pub reprojection_error: f64,
}

#[derive(Debug, Clone, Copy)]
pub struct CameraParams {
    pub fx: f64,
    pub fy: f64,
    pub cx: f64,
    pub cy: f64,
    pub tag_size_m: f64,
}

const POSE_ESTIMATION_ITERS: usize = 50;

pub fn worker_loop(
    rx: Receiver<DecodedFrame>,
    csv_poses: &[PoseSample],
    frame_timestamps_us: &[i64],
    camera: CameraParams,
    tag_family_str: &str,
    max_hamming: usize,
) -> anyhow::Result<Vec<TagObservation>> {
    let tag_family: Family = tag_family_str
        .parse()
        .map_err(|e| anyhow::anyhow!("invalid family tag '{tag_family_str}': {e:?}"))?;

    let mut detector: Detector = DetectorBuilder::new().add_family_bits(tag_family, max_hamming).build()?;

    let tag_params = TagParams {
        tagsize: camera.tag_size_m,
        fx: camera.fx,
        fy: camera.fy,
        cx: camera.cx,
        cy: camera.cy,
    };

    let mut observations = Vec::new();

    for frame in rx.iter() {
        let mut image = match Image::zeros_with_stride(frame.width, frame.height, frame.stride) {
            Ok(img) => img,
            Err(e) => continue,
        };
        if image.as_slice().len() != frame.gray_data.len() {
            continue;
        }
        image.as_slice_mut().copy_from_slice(&frame.gray_data);

        let detections = detector.detect(&image);
        if detections.is_empty() {
            continue;
        }

        let target_us = match frame_timestamps_us.get(frame.frame_index) {
            Some(ts) => *ts,
            None => continue,
        };

        let world_cam = match interpolate_pose(csv_poses, target_us) {
            Some(iso) => iso,
            None => continue,
        };

        for detection in &detections {
            let estimations =
                detection.estimate_tag_pose_orthogonal_iteration(&tag_params, POSE_ESTIMATION_ITERS);
            let best = match estimations
                .into_iter()
                .min_by(|a, b| a.error.partial_cmp(&b.error).unwrap())
            {
                Some(b) => b,
                None => continue,
            };

            let rotation_data = best.pose.rotation().data().to_vec();
            let translation_data = best.pose.translation().data().to_vec();
            if rotation_data.len() != 9 || translation_data.len() != 3 {
                continue;
            }

            let rot_matrix = Matrix3::from_row_slice(&rotation_data);
            let cam_tag_rotation =
                UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix_unchecked(rot_matrix));
            let cam_tag_translation = Translation3::from(Vector3::new(
                translation_data[0],
                translation_data[1],
                translation_data[2],
            ));
            let cam_tag = Isometry3::from_parts(cam_tag_translation, cam_tag_rotation);

            let world_tag = world_cam * cam_tag;

            let q = world_tag.rotation.quaternion();
            observations.push(TagObservation {
                frame_index: frame.frame_index,
                tag_id: detection.id(),
                translation_world: [
                    world_tag.translation.x,
                    world_tag.translation.y,
                    world_tag.translation.z,
                ],
                quaternion_world_wxyz: [q.w, q.i, q.j, q.k],
                reprojection_error: best.error,
            });
        }
    }

    Ok(observations)
}
