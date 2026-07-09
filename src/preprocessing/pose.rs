use nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3};
use serde::Deserialize;

#[derive(Debug, Deserialize)]
struct CsvRow {
    #[allow(dead_code)]
    graph_uid: String,
    tracking_timestamp_us: i64,
    #[allow(dead_code)]
    utc_timestamp_ns: i64,
    tx_world_device: f64,
    ty_world_device: f64,
    tz_world_device: f64,
    qx_world_device: f64,
    qy_world_device: f64,
    qz_world_device: f64,
    qw_world_device: f64,
}

#[derive(Debug, Clone, Copy)]
pub struct PoseSample {
    pub timestamp_us: i64,
    pub isometry: Isometry3<f64>,
}

pub fn load_poses(path: &str) -> anyhow::Result<Vec<PoseSample>> {
    let mut rdr = csv::ReaderBuilder::new()
        .has_headers(true)
        .from_path(path)?;

    let mut poses = Vec::new();
    for result in rdr.deserialize() {
        let row: CsvRow = result?;

        let translation = Translation3::new(
            row.tx_world_device,
            row.ty_world_device,
            row.tz_world_device,
        );
        let quaternion = nalgebra::Quaternion::new(
            row.qw_world_device,
            row.qx_world_device,
            row.qy_world_device,
            row.qz_world_device,
        );
        let rotation = UnitQuaternion::from_quaternion(quaternion);

        poses.push(PoseSample {
            timestamp_us: row.tracking_timestamp_us,
            isometry: Isometry3::from_parts(translation, rotation),
        });
    }

    poses.sort_by_key(|p| p.timestamp_us);

    Ok(poses)
}

pub fn interpolate_pose(poses: &[PoseSample], target_us: i64) -> Option<Isometry3<f64>> {
    match poses.binary_search_by_key(&target_us, |p| p.timestamp_us) {
        Ok(idx) => Some(poses[idx].isometry),
        Err(idx) => {
            if idx == 0 || idx == poses.len() {
                return None;
            }
            let before = &poses[idx - 1];
            let after = &poses[idx];

            let span = (after.timestamp_us - before.timestamp_us) as f64;
            if span <= 0.0 {
                return Some(before.isometry);
            }
            let t = (target_us - before.timestamp_us) as f64 / span;

            let translation = before
                .isometry
                .translation
                .vector
                .lerp(&after.isometry.translation.vector, t);
            let rotation = before
                .isometry
                .rotation
                .slerp(&after.isometry.rotation, t);

            Some(Isometry3::from_parts(
                Translation3::from(Vector3::from(translation)),
                rotation,
            ))
        }
    }
}
