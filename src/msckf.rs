pub mod utils;

use opencv::{
    video,
    core::{Mat, Point2f, Vector, Size, TermCriteria, TermCriteria_EPS, TermCriteria_COUNT},
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
    prev_left: Option<Mat>,
    prev_right: Option<Mat>,
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
            prev_left: None,
            prev_right: None,
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

        // Estimation
        for imu in self.buffer.drain_imu_until(image_timestamp) {
            let Some(prev) = self.timestamp else {
                self.timestamp = Some(imu.timestamp_ns);
                continue;
            };
            let delta_t_s = (imu.timestamp_ns - prev) as f64 * 1e-9;
            self.state = self.compute_estimate(&imu, delta_t_s);
            self.timestamp = Some(imu.timestamp_ns);
        }

        // Video measurement
        let curr_left = jpeg_to_gray(&left.jpeg)?;
        let curr_right = jpeg_to_gray(&right.jpeg)?;

        if let (Some(prev_l), Some(prev_r)) = (&self.prev_left, &self.prev_right) {
            let (prev_pts_l, curr_pts_l) = extract_features_temporal(prev_l, &curr_left)?;
            let (prev_pts_r, curr_pts_r) = extract_features_temporal(prev_r, &curr_right)?;
        }

        self.prev_left = Some(curr_left);
        self.prev_right = Some(curr_right);


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

        let left_mat = jpeg_to_gray(&left.jpeg)?;
        let right_mat = jpeg_to_gray(&right.jpeg)?;

        let (_, left_pts) = extract_features_temporal(&self.prev_left.clone().unwrap(), &left_mat)?;
        let (_, right_pts) = extract_features_temporal(&self.prev_right.clone().unwrap(), &right_mat)?;

        recording.log(
            format!("{}/left_image", log_path),
            &EncodedImage::from_file_contents(left.jpeg.clone()),
        )?;
        if !left_pts.is_empty() {
            let pts: Vec<[f32; 2]> = left_pts.iter().map(|p| [p.x, p.y]).collect();
            recording.log(
                format!("{}/left_image/features", log_path),
                &Points2D::new(pts)
                    .with_radii([3.0])
                    .with_colors([Color::from_rgb(0, 255, 0)]),
            )?;
        }

        recording.log(
            format!("{}/right_image", log_path),
            &EncodedImage::from_file_contents(right.jpeg.clone()),
        )?;
        if !right_pts.is_empty() {
            let pts: Vec<[f32; 2]> = right_pts.iter().map(|p| [p.x, p.y]).collect();
            recording.log(
                format!("{}/right_image/features", log_path),
                &Points2D::new(pts)
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

fn jpeg_to_gray(jpeg: &[u8]) -> anyhow::Result<Mat> {
    let buf = Vector::from_slice(jpeg);
    Ok(imgcodecs::imdecode(&buf, imgcodecs::IMREAD_GRAYSCALE)?)
}

fn extract_features_temporal(
    prev: &Mat,
    curr: &Mat,
) -> anyhow::Result<(Vector<Point2f>, Vector<Point2f>)> {
    let mut prev_corners = Vector::<Point2f>::new();
    imgproc::good_features_to_track(
        prev, &mut prev_corners, 200, 0.01, 10.0,
        &Mat::default(), 3, false, 0.04,
    )?;

    if prev_corners.is_empty() {
        return Ok((Vector::new(), Vector::new()));
    }

    let mut curr_corners = Vector::<Point2f>::new();
    let mut status = Vector::<u8>::new();
    let mut err = Vector::<f32>::new();
    video::calc_optical_flow_pyr_lk(
        prev, curr,
        &prev_corners, &mut curr_corners,
        &mut status, &mut err,
        Size::new(21, 21), 3,
        TermCriteria::new(
            TermCriteria_EPS + TermCriteria_COUNT, 30, 0.01,
        )?,
        0, 1e-4,
    )?;

    let prev_pts = prev_corners.iter()
        .zip(status.iter())
        .filter(|(_, s)| *s == 1)
        .map(|(p, _)| p)
        .collect();

    let curr_pts = curr_corners.iter()
        .zip(curr_corners.iter())
        .zip(status.iter())
        .filter(|(_, s)| *s == 1)
        .map(|((_, c), _)| c)
        .collect();

    Ok((prev_pts, curr_pts))
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

