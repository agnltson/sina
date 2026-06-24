use nalgebra::{Vector3, UnitQuaternion, OMatrix, Const, Matrix3};
use opencv::core::Point2f;
use std::collections::VecDeque;

pub const MAX_POS_SAVED: usize = 20;
pub const STATE_SIZE: usize = 15+6*MAX_POS_SAVED;

pub const FX: f64 = 150.0;
pub const FY: f64 = 150.0;
pub const CX: f64 = 256.0;  // 512/2
pub const CY: f64 = 256.0;  // 512/2

const R_CAM_IMU: [[f64; 3]; 3] = [
    [-0.00850024,  0.99647865, -0.08341486],
    [ 0.99988281,  0.00740777, -0.01339755],
    [-0.01273245, -0.08351896, -0.99642484],
];

const T_CAM_IMU: [f64; 3] = [0.00058498, -0.00020483, -0.00659877];

pub fn r_cam_imu() -> Matrix3<f64> {
    Matrix3::from_row_slice(&[
        -0.00850024,  0.99647865, -0.08341486,
         0.99988281,  0.00740777, -0.01339755,
        -0.01273245, -0.08351896, -0.99642484,
    ])
}

pub fn t_cam_imu() -> Vector3<f64> {
    Vector3::new(0.00058498, -0.00020483, -0.00659877)
}

pub struct State {
    pub p: Vector3<f64>,
    pub v: Vector3<f64>,
    pub q: UnitQuaternion<f64>,
    pub ba: Vector3<f64>,
    pub bg: Vector3<f64>,
    pub saved: VecDeque<(Vector3<f64>, UnitQuaternion<f64>)>,
}

impl State {
    pub fn new(
        p: Vector3<f64>,
        v: Vector3<f64>,
        q: UnitQuaternion<f64>,
        ba: Vector3<f64>,
        bg: Vector3<f64>,
        saved: VecDeque<(Vector3<f64>, UnitQuaternion<f64>)>,
    ) -> Self {
        Self {
            p,
            v,
            q,
            ba,
            bg,
            saved,
        }
    }
}

pub struct FeatureTrack {
    pub id: u64,
    pub observations: Vec<(usize, Point2f)>, // clone_index, pixel coords
}
