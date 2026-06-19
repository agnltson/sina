use nalgebra::{Vector3, UnitQuaternion};
use std::collections::VecDeque;

const N: usize = 32;

pub struct State {
    pub p: Vector3<f64>,
    pub v: Vector3<f64>,
    pub q: UnitQuaternion<f64>,
    pub ba: Vector3<f64>,
    pub bg: Vector3<f64>,
    pub window: VecDeque<Vec<u8>>,
    pub window_size: usize,
}

impl State {
    pub fn new(
        p: Vector3<f64>,
        v: Vector3<f64>,
        q: UnitQuaternion<f64>,
        ba: Vector3<f64>,
        bg: Vector3<f64>,
    ) -> Self {
        Self {
            p,
            v,
            q,
            ba,
            bg,
            window: VecDeque::new(),
            window_size: N,
        }
    }
}
