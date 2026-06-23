use nalgebra::{Vector3, UnitQuaternion, OMatrix, Const};
use opencv::core::Point2f;
use std::collections::VecDeque;

pub const MAX_POS_SAVED: usize = 20;
pub const STATE_SIZE: usize = 15+6*MAX_POS_SAVED;

pub const FX: f64 = 150.0;
pub const FY: f64 = 150.0;
pub const CX: f64 = 256.0;  // 512/2
pub const CY: f64 = 256.0;  // 512/2

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
