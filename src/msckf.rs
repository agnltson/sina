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
    buffer: SensorBuffer,
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
            buffer: SensorBuffer::new(),
        }
    }

    pub fn launch(
        &mut self,
        recording: RecordingStream,
        imu_rx: mpsc::Receiver<ImuMessage>,
        image_rx: mpsc::Receiver<ImageMessage>,
        pos_tx: mpsc::Sender<Vector3<f64>>,
    ) -> anyhow::Result<()> {
        loop {
            while let Ok(imu) = imu_rx.try_recv() {
                self.buffer.push_imu(imu);
            }
            while let Ok(img) = image_rx.try_recv() {
                self.buffer.push_image(img);
            }

            if let Some(pos) = self.update(&recording)? {
                pos_tx.send(pos)?;
            }
        }
    }

    fn update(&mut self, recording: &RecordingStream) -> anyhow::Result<Option<Vector3<f64>>> {
        let Some((left, right)) = self.buffer.pop_synced_image() else {
            return Ok(None);
        };

        let image_timestamp = (left.timestamp_ns + right.timestamp_ns) / 2;

        for imu in self.buffer.drain_imu_until(image_timestamp) {
            let Some(prev) = self.timestamp else {
                self.timestamp = Some(imu.timestamp_ns);
                continue;
            };
            let delta_t_s = (imu.timestamp_ns - prev) as f64 * 1e-9;
            self.state = self.compute_estimate(&imu, delta_t_s);
            self.timestamp = Some(imu.timestamp_ns);
        }

        self.log_images(recording, &left, &right)?;

        Ok(Some(self.state.p))
    }

    fn log_images(
        &self,
        recording: &RecordingStream,
        left: &ImageMessage,
        right: &ImageMessage,
    ) -> anyhow::Result<()> {
        let log_path = "msckf";

        let lfeatures = compute_features(&left.jpeg)?;
        let rfeatures = compute_features(&right.jpeg)?;

        recording.log(
            format!("{}/left_image", log_path),
            &EncodedImage::from_file_contents(left.jpeg.clone()),
        )?;
        if !lfeatures.is_empty() {
            recording.log(
                format!("{}/left_image/features", log_path),
                &Points2D::new(lfeatures)
                    .with_radii([3.0])
                    .with_colors([Color::from_rgb(0, 255, 0)]),
            )?;
        }

        recording.log(
            format!("{}/right_image", log_path),
            &EncodedImage::from_file_contents(right.jpeg.clone()),
        )?;
        if !rfeatures.is_empty() {
            recording.log(
                format!("{}/right_image/features", log_path),
                &Points2D::new(rfeatures)
                    .with_radii([3.0])
                    .with_colors([Color::from_rgb(0, 255, 0)]),
            )?;
        }

        Ok(())
    }

    fn compute_estimate(&self, imu: &ImuMessage, delta_t_s: f64) -> State {
        let previous_state = &self.state;
        State::new(
            predict_pos(previous_state, imu, delta_t_s),
            predict_vel(previous_state, imu, delta_t_s),
            predict_quat(previous_state, imu, delta_t_s),
            previous_state.ba,
            previous_state.bg,
        )
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

