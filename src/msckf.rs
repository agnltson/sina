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
    Const, SMatrix, SVector, DMatrix, DVector
};

use std::sync::mpsc;
use rerun::{RecordingStream, Color, EncodedImage, Points2D};
use std::collections::VecDeque;

use utils::{
    State, STATE_SIZE, MAX_POS_SAVED, FeatureTrack,
    FX, FY, CX, CY, r_cam_imu, t_cam_imu,
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
        let a_imu = Vector3::new(-9.75, -0.91, 0.75).normalize();
        let g_world = Vector3::new(0.0, 0.0, 1.0); // normalisé
        let initial_q = UnitQuaternion::rotation_between(&a_imu, &g_world)
            .unwrap_or(UnitQuaternion::identity());

        Self {
            state: State::new(
                Vector3::new(0.0, 0.0, 0.0),
                Vector3::new(0.0, 0.0, 0.0),
                initial_q,
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
        let clone_index = self.state.saved.len().saturating_sub(1);

        if let (Some(prev_l), Some(prev_r)) = (self.prev_left.take(), self.prev_right.take()) {
            println!("update: appel track_features");
            let mut tracks_left = std::mem::take(&mut self.active_tracks_left);
            let mut tracks_right = std::mem::take(&mut self.active_tracks_right);

            self.track_features(&prev_l, &curr_left, clone_index, &mut tracks_left)?;
            self.track_features(&prev_r, &curr_right, clone_index, &mut tracks_right)?;

            self.active_tracks_left = tracks_left;
            self.active_tracks_right = tracks_right;
        }

        self.prev_left = Some(curr_left);
        self.prev_right = Some(curr_right);


        //self.log_images(recording, &left, &right)?;

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
        // Dans compute_estimate, avant tout calcul
        println!("accel raw = {:?}", imu.accel_msec2);
        println!("gyro raw = {:?}", imu.gyro_radsec);

        let g_world = Vector3::new(0.0, 0.0, 9.81);
        let a_corrected = imu.accel_msec2 - self.state.ba;
        let a_world = self.state.q.to_rotation_matrix() * a_corrected;
        println!("a_world = {:?}, devrait être proche de [0, 0, 9.81]", a_world);

        let previous_state = &self.state;
        State::new(
            predict_pos(previous_state, imu, delta_t_s),
            predict_vel(previous_state, imu, delta_t_s),
            predict_quat(previous_state, imu, delta_t_s),
            previous_state.ba,
            previous_state.bg,
            previous_state.saved.clone(), // ← garder les clones
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

    fn process_lost_feature(&mut self, track: FeatureTrack) {
        println!("process_lost_feature: {} observations", track.observations.len());
        let Some(p_f_c1) = self.triangulate(&track) else {
            return;
        };
        println!("triangulate: ok, p_f_c1 = {:?}", p_f_c1);
        self.msckf_update(&track, &p_f_c1);
    }

    fn msckf_update(&mut self, track: &FeatureTrack, p_f_c1: &Vector3<f64>) {
        let Some((r, h_f, h_c)) = self.compute_residuals_and_jacobians(track, p_f_c1) else {
            println!("msckf_update: residuals failed");
            return;
        };
        println!("msckf_update: r.norm() = {}", r.norm());
        if r.norm() > 1.0 {
            println!("résidu trop grand, feature rejetée");
            return;
        }

        // --- Left nullspace de H_f ---
        // H_f est 2M x 3, son left nullspace est de dimension 2M-3
        // On utilise la décomposition QR : H_f = Q * [R; 0]
        // Q = [Q1 | Q2] où Q2 (2M x 2M-3) est le left nullspace
        //let qr = h_f.clone().qr();
        //let q = qr.clone().q(); // 2M x 2M
        let m = track.observations.len();
        // Q2 : colonnes 3..2M de Q
        let m = track.observations.len();
        if 2 * m <= 3 {
            return; // pas assez d'observations pour le left nullspace
        }
        let svd = h_f.svd(true, false);
        let u = svd.u.unwrap(); // 2M x 2M — forme complète
        println!("u shape: {}x{}", u.nrows(), u.ncols());
        let v = u.columns(3, 2*m - 3).into_owned();

        // Projeter résidus et Jacobien
        let r_o = v.transpose() * &r;       // (2M-3) x 1
        let h_o = v.transpose() * &h_c;     // (2M-3) x STATE_SIZE

        // --- Update EKF ---
        let p = self.cov_mat;
        let sigma_img: f64 = 1.0 / FX; // bruit pixel en coordonnées normalisées

        // S = H_o * P * H_o^T + R_o
        let ph_t = p * h_o.transpose();
        let s = &h_o * &ph_t + 
            DMatrix::<f64>::identity(2*m - 3, 2*m - 3) * sigma_img * sigma_img;

        let Some(s_inv) = s.try_inverse() else {
            println!("msckf_update: S not invertible");
            return;
        };

        // Gain de Kalman K = P * H_o^T * S^-1
        let k = ph_t * s_inv; // STATE_SIZE x (2M-3)

        // Correction de l'état
        let delta_x = &k * &r_o; // STATE_SIZE x 1
        println!("delta_x norm = {}", delta_x.norm());

        self.apply_correction(&DVector::from_column_slice(delta_x.as_slice()));

        // Mise à jour de la covariance : P = (I - K*H_o) * P
        let i_kh = DMatrix::<f64>::identity(STATE_SIZE, STATE_SIZE) - &k * &h_o;
        let p_dyn = DMatrix::<f64>::from_row_slice(
            STATE_SIZE,
            STATE_SIZE,
            p.as_slice(),
        );
        let p_new = &i_kh * p_dyn;
        self.cov_mat.copy_from(&OMatrix::<f64, Const<STATE_SIZE>, Const<STATE_SIZE>>::from_iterator(
            p_new.iter().cloned()
        ));
    }

    fn apply_correction(&mut self, delta_x: &DVector<f64>) {
        // Position
        self.state.p += Vector3::new(delta_x[0], delta_x[1], delta_x[2]);
        // Vitesse
        self.state.v += Vector3::new(delta_x[3], delta_x[4], delta_x[5]);
        // Rotation (correction additive sur l'angle-axe)
        let dtheta = Vector3::new(delta_x[6], delta_x[7], delta_x[8]);
        self.state.q = self.state.q * UnitQuaternion::from_scaled_axis(dtheta);
        // Biais
        self.state.ba += Vector3::new(delta_x[9],  delta_x[10], delta_x[11]);
        self.state.bg += Vector3::new(delta_x[12], delta_x[13], delta_x[14]);

        // Correction des poses clonées
        for (i, (p_c, q_c)) in self.state.saved.iter_mut().enumerate() {
            let base = 15 + 6 * i;
            *p_c += Vector3::new(delta_x[base],   delta_x[base+1], delta_x[base+2]);
            let dtheta_c = Vector3::new(delta_x[base+3], delta_x[base+4], delta_x[base+5]);
            *q_c = *q_c * UnitQuaternion::from_scaled_axis(dtheta_c);
        }
    }

    fn track_features(&mut self, prev: &Mat, curr: &Mat, clone_index: usize, tracks: &mut Vec<FeatureTrack>) -> anyhow::Result<()> {
        println!("track_features: {} tracks actifs, clone_index={}", tracks.len(), clone_index);
        if tracks.is_empty() {
            let mut corners = Vector::<Point2f>::new();
            imgproc::good_features_to_track(prev, &mut corners, 200, 0.01, 10.0, &Mat::default(), 3, false, 0.04)?;
            println!("detection initiale: {} corners", corners.len());
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

        let n_lost = status.iter().filter(|s| *s == 0).count();
        println!("LK: {} tracked, {} lost", status.iter().filter(|s| *s == 1).count(), n_lost);

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
            self.process_lost_feature(track);
        }

        // Detect again if they are not enough features tracked
        if tracks.len() < 50 {
            self.detect_new_features(curr, clone_index, tracks)?;
        }
        Ok(())
    }

    fn triangulate(&self, track: &FeatureTrack) -> Option<Vector3<f64>> {
        if track.observations.len() < 3 {
            return None;
        }

        let obs: Vec<(SMatrix<f64, 2, 1>, usize)> = track.observations.iter()
            .map(|(clone_idx, pt)| {
                let z = SMatrix::<f64, 2, 1>::new(
                    (pt.x as f64 - CX) / FX,
                    (pt.y as f64 - CY) / FY,
                );
                (z, *clone_idx)
            })
            .collect();

        let c1_idx = obs[0].1;
        let (imu_p_c1, imu_q_c1) = self.state.saved[c1_idx];
        let r_c1 = r_cam_imu() * imu_q_c1.to_rotation_matrix().into_inner();
        let p_c1 = r_cam_imu() * imu_p_c1 + t_cam_imu();

        let mut alpha = obs[0].0[0];
        let mut beta  = obs[0].0[1];
        let mut rho   = 1.0;

        for _ in 0..10 {
            let mut hessian  = SMatrix::<f64, 3, 3>::zeros();
            let mut gradient = SMatrix::<f64, 3, 1>::zeros();

            for (z, clone_idx) in &obs {
                let (imu_p, imu_q) = self.state.saved[*clone_idx];
                let r_cam_world = r_cam_imu() * imu_q.to_rotation_matrix().into_inner();
                let p_cam = r_cam_imu() * imu_p + t_cam_imu();

                let r_rel = r_cam_world * r_c1.transpose();
                let t_rel = r_cam_world * (p_c1 - p_cam);

                let abr = Vector3::new(alpha, beta, 1.0);
                let h = r_rel * abr + t_rel * rho;
                let h1 = h[0]; let h2 = h[1]; let h3 = h[2];

                let z_hat = SMatrix::<f64, 2, 1>::new(h1/h3, h2/h3);
                let r_i = z - z_hat;

                let mut ji = SMatrix::<f64, 2, 3>::zeros();
                ji[(0,0)] = (r_rel[(0,0)] - (h1/h3)*r_rel[(2,0)]) / h3;
                ji[(0,1)] = (r_rel[(0,1)] - (h1/h3)*r_rel[(2,1)]) / h3;
                ji[(0,2)] = (t_rel[0]     - (h1/h3)*t_rel[2])     / h3;
                ji[(1,0)] = (r_rel[(1,0)] - (h2/h3)*r_rel[(2,0)]) / h3;
                ji[(1,1)] = (r_rel[(1,1)] - (h2/h3)*r_rel[(2,1)]) / h3;
                ji[(1,2)] = (t_rel[1]     - (h2/h3)*t_rel[2])     / h3;

                hessian  += ji.transpose() * ji;
                gradient += ji.transpose() * r_i;
            }

            let Some(h_inv) = hessian.try_inverse() else {
                return None;
            };
            let delta = h_inv * gradient;

            alpha += delta[0];
            beta  += delta[1];
            rho   += delta[2];

            if delta.norm() < 1e-6 {
                break;
            }
        }

        if rho <= 0.0 {
            return None;
        }

        Some(Vector3::new(alpha/rho, beta/rho, 1.0/rho))
    }

    fn compute_residuals_and_jacobians(
        &self,
        track: &FeatureTrack,
        p_f_c1: &Vector3<f64>, // triangulated pos
    ) -> Option<(
        DVector<f64>, // r (2M x 1)
        DMatrix<f64>, // H_f (2M x 3)
        DMatrix<f64>, // H_C (2M x STATE_SIZE)
    )> {
        let m = track.observations.len();

        let mut r   = DVector::<f64>::zeros(2 * m);
        let mut h_f = DMatrix::<f64>::zeros(2 * m, 3);
        let mut h_c = DMatrix::<f64>::zeros(2 * m, STATE_SIZE);

        let c1_idx = track.observations[0].0;
        let (imu_p_c1, imu_q_c1) = self.state.saved[c1_idx];
        let r_c1 = r_cam_imu() * imu_q_c1.to_rotation_matrix().into_inner();
        let p_c1 = r_cam_imu() * imu_p_c1 + t_cam_imu();

        for (i, (clone_idx, pt)) in track.observations.iter().enumerate() {
            let (imu_p, imu_q) = self.state.saved[*clone_idx];
            let r_ci = r_cam_imu() * imu_q.to_rotation_matrix().into_inner();
            let p_ci = r_cam_imu() * imu_p + t_cam_imu();

            let r_rel = r_ci * r_c1.transpose();
            let t_rel = r_ci * (p_c1 - p_ci);

            // feature position in C_i
            let p_f_ci = r_rel * p_f_c1 + t_rel;
            let px = p_f_ci[0];
            let py = p_f_ci[1];
            let pz = p_f_ci[2];

            if pz <= 0.0 {
                return None;
            }

            let z_obs = SMatrix::<f64, 2, 1>::new(
                (pt.x as f64 - CX) / FX,
                (pt.y as f64 - CY) / FY,
            );
            let z_hat = SMatrix::<f64, 2, 1>::new(px/pz, py/pz);
            r[2*i]   = z_obs[0] - z_hat[0];
            r[2*i+1] = z_obs[1] - z_hat[1];

            // Jacobian
            let j_h = SMatrix::<f64, 2, 3>::new(
                1.0/pz, 0.0, -px/(pz*pz),
                0.0, 1.0/pz, -py/(pz*pz),
            );

            // H_f_i = J_h * R_rel (2x3)
            let h_f_i = j_h * r_rel;
            h_f.rows_mut(2*i, 2).copy_from(&h_f_i);

            // H_C_i = J_h * [skew(p_f_ci) | -R_ci] (2x6)
            let mut h_c_i = SMatrix::<f64, 2, 6>::zeros();
            h_c_i.fixed_view_mut::<2, 3>(0, 0)
                .copy_from(&(j_h * skew(&p_f_ci)));
            h_c_i.fixed_view_mut::<2, 3>(0, 3)
                .copy_from(&(-j_h * r_ci));

            let col = 15 + 6 * clone_idx;
            h_c.view_mut((2*i, col), (2, 6)).copy_from(&h_c_i);
        }

        Some((r, h_f, h_c))
    }

    fn detect_new_features(
        &mut self,
        curr: &Mat,
        clone_index: usize,
        tracks: &mut Vec<FeatureTrack>,
    ) -> anyhow::Result<()> {
        let mut new_corners = Vector::<Point2f>::new();
        imgproc::good_features_to_track(
            curr, &mut new_corners, 200, 0.01, 10.0,
            &Mat::default(), 3, false, 0.04,
        )?;

        let existing_pts: Vec<Point2f> = tracks.iter()
            .filter_map(|t| t.observations.last())
            .map(|(_, p)| *p)
            .collect();

        for pt in new_corners.iter() {
            let too_close = existing_pts.iter().any(|ep| {
                let dx = ep.x - pt.x;
                let dy = ep.y - pt.y;
                (dx*dx + dy*dy).sqrt() < 10.0
            });

            if !too_close {
                tracks.push(FeatureTrack {
                    id: self.next_feature_id,
                    observations: vec![(clone_index, pt)],
                });
                self.next_feature_id += 1;
            }
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
    let g_world = Vector3::new(0.0, 0.0, 9.81);
    previous_state.p +
    previous_state.v.scale(delta_t_s) +
    (previous_state.q.to_rotation_matrix() * (imu.accel_msec2 - previous_state.ba) + g_world)
        .scale(0.5 * delta_t_s * delta_t_s)
}

#[inline(always)]
fn predict_vel(previous_state: &State, imu: &ImuMessage, delta_t_s: f64) -> Vector3<f64> {
    let g_world = Vector3::new(0.0, 0.0, 9.81);
    previous_state.v +
    (previous_state.q.to_rotation_matrix() * (imu.accel_msec2 - previous_state.ba) + g_world)
        .scale(delta_t_s)
}

#[inline(always)]
fn predict_quat(previous_state: &State, imu: &ImuMessage, delta_t_s: f64) -> UnitQuaternion<f64> {
    previous_state.q *
    UnitQuaternion::from_scaled_axis(0.5 * (imu.gyro_radsec - previous_state.bg).scale(delta_t_s))
}

