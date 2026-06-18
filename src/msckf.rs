pub mod utils;

use nalgebra::{Vector3, Quaternion, UnitQuaternion};
use std::sync::mpsc::{Sender, Receiver};

use utils::State;
use crate::device_stream::DeviceStream;
use crate::sensor_data::{SensorData, ImuMessage, ImageMessage};
use crate::navigation::VisualisationData;

pub struct MSCKF {
    current_state: State,
    current_timestamp: Option<u64>,
}

impl MSCKF {
    pub fn new() -> Self {
        let axisangle = Vector3::x() * std::f64::consts::FRAC_PI_2;
        Self {
            current_state: State::new(
                Vector3::new(0.0, 0.0, 0.0),
                Vector3::new(0.0, 0.0, 0.0),
                UnitQuaternion::new(axisangle),
                Vector3::new(0.0, 0.0, 0.0),
                Vector3::new(0.0, 0.0, 0.0),
            ),
            current_timestamp: None,
        }
    }

    pub fn run(
        &mut self,
        sensor_data_receiver: Receiver<SensorData>,
        visual_data_sender: Sender<VisualisationData>
        ) -> Result<(), Box<dyn std::error::Error>> {

        while let Ok(data) = sensor_data_receiver.recv() {
            match data {
                SensorData::Imu(m) => {
                    if m.imu_idx == 0 {
                        self.update(&m);
                        if visual_data_sender.send(VisualisationData::Position(self.current_state.p, m.timestamp_ns)).is_err() {
                            break;
                        }
                    } else {
                        continue;
                    }
                },
                SensorData::Image(m) => {
                    if m.camera == 1 {
                        if visual_data_sender.send(VisualisationData::LeftImage(m.jpeg, m.timestamp_ns)).is_err() {
                            break;
                        }
                    } else {
                        if visual_data_sender.send(VisualisationData::RightImage(m.jpeg, m.timestamp_ns)).is_err() {
                            break;
                        }
                    }
                },
            }
        }
        Ok(())

    }

    pub fn update(&mut self, imu: &ImuMessage) {
        if self.current_timestamp.is_none() {
            self.current_timestamp = Some(imu.timestamp_ns);
            return;
        }
        let previous_timestamp = self.current_timestamp.unwrap();
        let previous_state = &self.current_state;

        let Some(dt_ns) = imu.timestamp_ns.checked_sub(previous_timestamp) else {
            eprintln!(
                "timestamp non monotone: prev={}, curr={}",
                previous_timestamp,
                imu.timestamp_ns
            );
            return;
        };

        let delta_t_s = dt_ns as f64 * 1e-9;

        let pk = predict_pos(previous_state, imu, delta_t_s);
        let vk = predict_vel(previous_state, imu, delta_t_s);
        let qk = predict_quat(previous_state, imu, delta_t_s);
        self.current_state = State::new(
            pk,
            vk,
            qk,
            previous_state.ba,
            previous_state.bg,
        );
        self.current_timestamp = Some(imu.timestamp_ns);
    }

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

