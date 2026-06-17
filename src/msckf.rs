pub mod utils;

use nalgebra::{Vector3, Quaternion, UnitQuaternion};

use utils::State;
use crate::sensor_data::ImuMessage;

pub fn propagate_cov() {

}

pub fn predict_state(previous_state: &State, imu: &ImuMessage, delta_t_ns: u64) -> State {
    let delta_t_s = delta_t_ns as f64 / 10e+9;
    let pk = predict_pos(previous_state, imu, delta_t_s);
    let vk = predict_vel(previous_state, imu, delta_t_s);
    let qk = predict_quat(previous_state, imu, delta_t_s);
    State::new(
        pk,
        vk,
        qk,
        previous_state.ba,
        previous_state.bg,
    )
}

#[inline(always)]
fn predict_pos(previous_state: &State, imu: &ImuMessage, delta_t_s: f64) -> Vector3<f64> {
    previous_state.p +
    previous_state.v.scale(delta_t_s) +
    previous_state.q.to_rotation_matrix() * ((imu.accel_msec2 - previous_state.ba).scale(0.5 * delta_t_s * delta_t_s))
}

#[inline(always)]
fn predict_vel(previous_state: &State, imu: &ImuMessage, delta_t_s: f64) -> Vector3<f64> {
    previous_state.v +
    previous_state.q.to_rotation_matrix() * (imu.accel_msec2 - previous_state.ba).scale(delta_t_s)
}

#[inline(always)]
fn predict_quat(previous_state: &State, imu: &ImuMessage, delta_t_s: f64) -> UnitQuaternion<f64> {
    previous_state.q *
    UnitQuaternion::from_scaled_axis(0.5 * (imu.gyro_radsec - previous_state.bg).scale(delta_t_s))
}
