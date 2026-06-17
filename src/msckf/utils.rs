use nalgebra::{Vector3, UnitQuaternion};

pub struct State {
    pub p: Vector3<f64>,
    pub v: Vector3<f64>,
    pub q: UnitQuaternion<f64>,
    pub ba: Vector3<f64>,
    pub bg: Vector3<f64>,
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
        }
    }
}

pub struct ErrorState {
    pub dp: Vector3<f64>,
    pub dv: Vector3<f64>,
    pub dtheta: f64,
    pub dba: Vector3<f64>,
    pub dbg: Vector3<f64>,
}

impl ErrorState {
    pub fn new(
        dp: Vector3<f64>,
        dv: Vector3<f64>,
        dtheta: f64,
        dba: Vector3<f64>,
        dbg: Vector3<f64>,
    ) -> Self {
        Self {
            dp,
            dv,
            dtheta,
            dba,
            dbg,
        }
    }
}
