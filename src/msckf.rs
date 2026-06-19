pub mod utils;

use opencv::{
    core::{Mat, Point2f, Vector},
    imgcodecs,
    imgproc,
};
use nalgebra::{Vector3, Quaternion, UnitQuaternion};
use std::sync::mpsc;
use rerun::{RecordingStream, Color, EncodedImage, Points2D};

use utils::State;
use crate::device_stream::DeviceStream;
use crate::sensor_data::{SensorData, ImuMessage, ImageMessage};
use crate::sensor_buffer::SensorBuffer;
use crate::navigation::VisualisationData;

pub struct MSCKF {
    state: State,
    timestamp: Option<u64>,
}

impl MSCKF {
    pub fn new() -> Self {
        let axisangle = Vector3::x() * std::f64::consts::FRAC_PI_2;
        Self {
            state: State::new(
                Vector3::new(0.0, 0.0, 0.0),
                Vector3::new(0.0, 0.0, 0.0),
                UnitQuaternion::new(axisangle),
                Vector3::new(0.0, 0.0, 0.0),
                Vector3::new(0.0, 0.0, 0.0),
            ),
            timestamp: None,
        }
    }

    pub fn launch(
        &mut self,
        recording: RecordingStream,
        buffer_rx: mpsc::Receiver<SensorBuffer>,
        pos_tx: mpsc::Sender<Vector3<f64>>
        ) -> anyhow::Result<()> {

        loop {
            if let Ok(mut buffer) = buffer_rx.recv() {

                self.log_update(&recording, &buffer)?;

                self.update(&mut buffer);
                pos_tx.send(self.state.p)?;
            }
        }
        Ok(())

    }

    pub fn update(&mut self, buffer: &mut SensorBuffer) {
        while let Some(imu) = buffer.pop_imu() {
            if self.timestamp.is_none() {
                self.timestamp = Some(imu.timestamp_ns);
                return;
            }
            let previous_timestamp = self.timestamp.unwrap();

            let delta_t_s = (imu.timestamp_ns - previous_timestamp) as f64 * 1e-9;
            self.state = self.compute_estimate(&imu, delta_t_s);
            self.timestamp = Some(imu.timestamp_ns);
        }
    }

    fn compute_estimate(&self, imu: &ImuMessage, delta_t_s: f64) -> State {
        let previous_state = &self.state;

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

    fn log_update(&self, record: &RecordingStream, buffer: &SensorBuffer) -> anyhow::Result<()> {
        let log_path = "msckf";
        if let Some(m) = buffer.get_image() {
            let features = compute_features(&m.jpeg)?;

            record.log(
                format!("{}/image", log_path).as_str(),
                &EncodedImage::from_file_contents(m.jpeg),
                )?;

            if !features.is_empty() {
                record.log(
                    format!("{}/features", log_path).as_str(),
                    &Points2D::new(features)
                        .with_radii([3.0])
                        .with_colors([Color::from_rgb(0, 255, 0)]),
                )?;
            }
        }
        Ok(())
    }
}

fn compute_features(jpeg: &Vec<u8>) -> anyhow::Result<Vec<[f32; 2]>>  {
    let buf = Vector::from_slice(jpeg);
    let frame = imgcodecs::imdecode(&buf, imgcodecs::IMREAD_COLOR)?;

    let mut gray = Mat::default();
    imgproc::cvt_color(&frame, &mut gray, imgproc::COLOR_BGR2GRAY, 0)?;

    // Shi-Tomasi
    let mut corners = Vector::<Point2f>::new();
    imgproc::good_features_to_track(
        &gray,
        &mut corners,
        200,  // max corners
        0.01, // quality level
        10.0, // min distance
        &Mat::default(),
        3,
        false,
        0.04,
    )?;
    let positions: Vec<[f32; 2]> = corners.iter().map(|p| [p.x, p.y]).collect();
    Ok(positions)
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

