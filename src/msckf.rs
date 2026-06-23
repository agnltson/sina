pub mod utils;

use opencv::{
    video,
    core::{Mat, Point2f, Vector, Size, TermCriteria, TermCriteria_EPS, TermCriteria_COUNT},
    imgcodecs,
    imgproc,
};
use nalgebra::{
    Quaternion, UnitQuaternion,
    OMatrix, OVector, Matrix3, Vector3,
    Const, SMatrix, SVector,
};

use std::sync::mpsc;
use rerun::{RecordingStream, Color, EncodedImage, Points2D};
use std::collections::VecDeque;

use utils::{
    State, STATE_SIZE, MAX_POS_SAVED, FeatureTrack,
    FX, FY, CX, CY,
};
use crate::device_stream::DeviceStream;
use crate::sensor_data::{SensorData, ImuMessage, ImageMessage};
use crate::sensor_buffer::SensorBuffer;
use crate::navigation::VisualisationData;

const SIGMA_A: f64 = 16e-3;
const SIGMA_G: f64 = 1.5e-3;

const SIGMA_BA: f64 = 2.8e-4 * 10.0;
const SIGMA_BG: f64 = 1.7e-5 * 10.0;

pub struct MSCKF {
    state: State,
    timestamp: Option<u64>,
    buffer: SensorBuffer,
    cov_mat: OMatrix<f64, Const<STATE_SIZE>, Const<STATE_SIZE>>,
    prev_left: Option<Mat>,
    prev_right: Option<Mat>,
    active_tracks_left: Vec<FeatureTrack>,
    active_tracks_right: Vec<FeatureTrack>,
    next_feature_id: u64,
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
                VecDeque::new(),
            ),
            timestamp: None,
            buffer: SensorBuffer::new(),
            cov_mat: OMatrix::<f64, Const<STATE_SIZE>, Const<STATE_SIZE>>::zeros(),
            prev_left: None,
            prev_right: None,
            active_tracks_left: Vec::new(),
            active_tracks_right: Vec::new(),
            next_feature_id: 0,
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

            // Compute the covariance propagation of the previous step
            self.cov_mat = self.propagate_covariance(&imu, delta_t_s);

            self.state = self.compute_estimate(&imu, delta_t_s);

            self.timestamp = Some(imu.timestamp_ns);
        }

        self.clone_pos();

        // Video measurement
        let curr_left = jpeg_to_gray(&left.jpeg)?;
        let curr_right = jpeg_to_gray(&right.jpeg)?;

        if let (Some(prev_l), Some(prev_r)) = (&self.prev_left, &self.prev_right) {
            //let (prev_pts_l, curr_pts_l) = extract_features_temporal(prev_l, &curr_left)?;
            //let (prev_pts_r, curr_pts_r) = extract_features_temporal(prev_r, &curr_right)?;
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

        let left_pts: Vec<[f32; 2]> = self.active_tracks_left
            .iter()
            .filter_map(|t| t.observations.last())
            .map(|(_, p)| [p.x, p.y])
            .collect();

        let right_pts: Vec<[f32; 2]> = self.active_tracks_right
            .iter()
            .filter_map(|t| t.observations.last())
            .map(|(_, p)| [p.x, p.y])
            .collect();

        recording.log(
            format!("{}/left_image", log_path),
            &EncodedImage::from_file_contents(left.jpeg.clone()),
        )?;
        if !left_pts.is_empty() {
            recording.log(
                format!("{}/left_image/features", log_path),
                &Points2D::new(left_pts)
                    .with_radii([3.0])
                    .with_colors([Color::from_rgb(0, 255, 0)]),
            )?;
        }

        recording.log(
            format!("{}/right_image", log_path),
            &EncodedImage::from_file_contents(right.jpeg.clone()),
        )?;
        if !right_pts.is_empty() {
            recording.log(
                format!("{}/right_image/features", log_path),
                    &Points2D::new(right_pts)
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
            VecDeque::new(),
        )
    }

    fn propagate_covariance(
        &self,
        imu: &ImuMessage,
        dt: f64,
    ) -> OMatrix<f64, Const<STATE_SIZE>, Const<STATE_SIZE>> {

        let r = self.state.q.to_rotation_matrix().into_inner();
        let a = imu.accel_msec2 - self.state.ba;
        let w = imu.gyro_radsec - self.state.bg;

        let mut f = SMatrix::<f64, 15, 15>::zeros();
        // dp/dv
        f.fixed_view_mut::<3, 3>(0, 3)
            .copy_from(&Matrix3::identity());
        // dv/dtheta
        f.fixed_view_mut::<3, 3>(3, 6)
            .copy_from(&(-r * skew(&a)));
        // dv/dba
        f.fixed_view_mut::<3, 3>(3, 9)
            .copy_from(&(-r));
        // dtheta/dtheta
        f.fixed_view_mut::<3, 3>(6, 6)
            .copy_from(&(-skew(&w)));
        // dtheta/dbg
        f.fixed_view_mut::<3, 3>(6, 12)
            .copy_from(&(-Matrix3::identity()));

        let fd: SMatrix<f64, 15, 15> = SMatrix::<f64, 15, 15>::identity() + f * dt;

        let mut g = SMatrix::<f64, 15, 12>::zeros();
        // accel noise -> vel
        g.fixed_view_mut::<3, 3>(3, 0).copy_from(&(-r));
        // gyro noise -> rotation
        g.fixed_view_mut::<3, 3>(6, 3).copy_from(&(-Matrix3::identity()));
        // random step ba
        g.fixed_view_mut::<3, 3>(9, 6).copy_from(&Matrix3::identity());
        // random step bg
        g.fixed_view_mut::<3, 3>(12, 9).copy_from(&Matrix3::identity());

        let q = SMatrix::<f64, 12, 12>::from_diagonal(&SVector::<f64, 12>::from([
            SIGMA_A * SIGMA_A, SIGMA_A * SIGMA_A, SIGMA_A * SIGMA_A,
            SIGMA_G * SIGMA_G, SIGMA_G * SIGMA_G, SIGMA_G * SIGMA_G,
            SIGMA_BA * SIGMA_BA, SIGMA_BA * SIGMA_BA, SIGMA_BA * SIGMA_BA,
            SIGMA_BG * SIGMA_BG, SIGMA_BG * SIGMA_BG, SIGMA_BG * SIGMA_BG,
        ]));

        let p = self.cov_mat;
        let mut p_new = p.clone();

        let p_ii: SMatrix<f64, 15, 15> = p.fixed_view::<15, 15>(0, 0).into();
        let p_ii_new = fd * p_ii * fd.transpose() + g * q * g.transpose() * dt;
        p_new.fixed_view_mut::<15, 15>(0, 0).copy_from(&p_ii_new);

        let n_clones = self.state.saved.len();
        if n_clones > 0 {
            let n = n_clones*6;

            let p_ic = p.view((0, 15), (15, n)).into_owned();
            let p_ic_new = fd * p_ic;
            p_new.view_mut((0, 15), (15, n)).copy_from(&p_ic_new);

            p_new.view_mut((15, 0), (n, 15)).copy_from(&p_ic_new.transpose());
        }

        p_new
    }

    fn clone_pos(&mut self) {
        if self.state.saved.len() == MAX_POS_SAVED {
            self.remove_oldest_clone();
        }
        let i = self.state.saved.len();
        let row = 15 + 6 * i;

        // Cloning
        self.state.saved.push_back((self.state.p, self.state.q));

        let mut j = SMatrix::<f64, 6, 15>::zeros();
        j.fixed_view_mut::<3, 3>(0, 0).copy_from(&Matrix3::identity()); // dp_clone/dp
        j.fixed_view_mut::<3, 3>(3, 6).copy_from(&Matrix3::identity()); // dq_clone/dq

        // P augmentation
        let p_ii: SMatrix<f64, 15, 15> = self.cov_mat.fixed_view::<15, 15>(0, 0).into();
        let p_cc_new = j * p_ii * j.transpose();
        self.cov_mat.fixed_view_mut::<6, 6>(row, row).copy_from(&p_cc_new);

        let p_i_imu = j * p_ii;
        self.cov_mat.fixed_view_mut::<6, 15>(row, 0).copy_from(&p_i_imu);
        self.cov_mat.fixed_view_mut::<15, 6>(0, row).copy_from(&p_i_imu.transpose());

        for k in 0..i {
            let col = 15 + 6 * k;
            let p_imu_j: SMatrix<f64, 15, 6> = self.cov_mat.fixed_view::<15, 6>(0, col).into();
            let p_ij = j * p_imu_j;
            self.cov_mat.fixed_view_mut::<6, 6>(row, col).copy_from(&p_ij);
            self.cov_mat.fixed_view_mut::<6, 6>(col, row).copy_from(&p_ij.transpose());
        }
    }

    fn remove_oldest_clone(&mut self) {
        self.state.saved.pop_front();

        let n = self.state.saved.len();
        let total = 15 + 6 * (n + 1);

        for i in 0..n {
            let src = 15 + 6 * (i + 1);
            let dst = 15 + 6 * i;

            let block: SMatrix<f64, 6, 6> = self.cov_mat.fixed_view::<6, 6>(src, src).into();
            self.cov_mat.fixed_view_mut::<6, 6>(dst, dst).copy_from(&block);

            let imu_block: SMatrix<f64, 15, 6> = self.cov_mat.fixed_view::<15, 6>(0, src).into();
            self.cov_mat.fixed_view_mut::<15, 6>(0, dst).copy_from(&imu_block);
            self.cov_mat.fixed_view_mut::<6, 15>(dst, 0).copy_from(&imu_block.transpose());

            for j in 0..n {
                let src_j = 15 + 6 * (j + 1);
                let dst_j = 15 + 6 * j;
                let cross: SMatrix<f64, 6, 6> = self.cov_mat.fixed_view::<6, 6>(src, src_j).into();
                self.cov_mat.fixed_view_mut::<6, 6>(dst, dst_j).copy_from(&cross);
                self.cov_mat.fixed_view_mut::<6, 6>(dst_j, dst).copy_from(&cross.transpose());
            }
        }

        let freed = 15 + 6 * n;
        self.cov_mat.view_mut((freed, 0), (6, STATE_SIZE)).fill(0.0);
        self.cov_mat.view_mut((0, freed), (STATE_SIZE, 6)).fill(0.0);
    }

    fn track_features(&mut self, prev: &Mat, curr: &Mat, clone_index: usize, tracks: &mut Vec<FeatureTrack>) -> anyhow::Result<()> {
        if tracks.is_empty() {
            let mut corners = Vector::<Point2f>::new();
            imgproc::good_features_to_track(prev, &mut corners, 200, 0.01, 10.0, &Mat::default(), 3, false, 0.04)?;
            for pt in corners.iter() {
                tracks.push(FeatureTrack {
                    id: self.next_feature_id,
                    observations: vec![(clone_index, pt)],
                });
                self.next_feature_id += 1;
            }
            return Ok(());
        }

        // Get current tracked features pos
        let prev_pts: Vector<Point2f> = tracks.iter()
            .map(|t| t.observations.last().unwrap().1)
            .collect();

        // Compute the next pos
        let (curr_pts, status) = extract_features_temporal(prev, curr, &prev_pts)?;

        // Update or lost
        let mut lost = vec![];
        for (i, track) in tracks.iter_mut().enumerate() {
            if status.get(i).unwrap_or(0) == 1 {
                track.observations.push((clone_index, curr_pts.get(i).unwrap()));
            } else {
                lost.push(i);
            }
        }

        // use lost features as measure for msckf
        for i in lost.iter().rev() {
            let track = tracks.remove(*i);
            // self.process_lost_feature(track);
        }

        // Detect again if they are not enough features tracked
        if tracks.len() < 50 {
            // good_features_to_track + ajouter les nouvelles
        }
        Ok(())
    }
}

fn skew(v: &Vector3<f64>) -> Matrix3<f64> {
    Matrix3::new(
         0.0,  -v.z,   v.y,
         v.z,   0.0,  -v.x,
        -v.y,   v.x,   0.0,
    )
}


fn jpeg_to_gray(jpeg: &[u8]) -> anyhow::Result<Mat> {
    let buf = Vector::from_slice(jpeg);
    Ok(imgcodecs::imdecode(&buf, imgcodecs::IMREAD_GRAYSCALE)?)
}

fn extract_features_temporal(
    prev: &Mat,
    curr: &Mat,
    prev_pts: &Vector<Point2f>,
) -> anyhow::Result<(Vector<Point2f>, Vector<u8>)> {
    let mut curr_corners = Vector::<Point2f>::new();
    let mut status = Vector::<u8>::new();
    let mut err = Vector::<f32>::new();

    video::calc_optical_flow_pyr_lk(
        prev, curr,
        prev_pts, &mut curr_corners,
        &mut status, &mut err,
        Size::new(21, 21), 3,
        TermCriteria::new(TermCriteria_EPS + TermCriteria_COUNT, 30, 0.01)?,
        0, 1e-4,
    )?;

    Ok((curr_corners, status))
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

