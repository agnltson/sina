use std::ffi::CString;
use std::os::raw::{c_char, c_int};

#[repr(C)]
struct ShimDetector {
    _private: [u8; 0],
}
#[repr(C)]
struct ShimDetections {
    _private: [u8; 0],
}

unsafe extern "C" {
    fn shim_detector_create(
        family_name: *const c_char,
        nthreads: c_int,
        quad_decimate: f32,
        quad_sigma: f32,
        refine_edges: c_int,
    ) -> *mut ShimDetector;

    fn shim_detector_destroy(det: *mut ShimDetector);

    fn shim_detect(
        det: *mut ShimDetector,
        buf: *const u8,
        width: i32,
        height: i32,
        stride: i32,
    ) -> *mut ShimDetections;

    fn shim_detections_count(dets: *const ShimDetections) -> i32;
    fn shim_detection_id(dets: *const ShimDetections, idx: i32) -> i32;
    fn shim_detection_center(dets: *const ShimDetections, idx: i32, out_center: *mut f64);
    fn shim_detections_destroy(dets: *mut ShimDetections);

    fn shim_estimate_pose_orthogonal(
        dets: *const ShimDetections,
        idx: i32,
        tagsize_m: f64,
        fx: f64,
        fy: f64,
        cx: f64,
        cy: f64,
        n_iters: c_int,
        out_err1: *mut f64,
        out_r1: *mut f64,
        out_t1: *mut f64,
        out_err2: *mut f64,
        out_r2: *mut f64,
        out_t2: *mut f64,
    ) -> c_int;
}

#[derive(Debug, Clone, Copy)]
pub struct TagParams {
    pub tagsize: f64,
    pub fx: f64,
    pub fy: f64,
    pub cx: f64,
    pub cy: f64,
}

#[derive(Debug, Clone, Copy)]
pub struct PoseCandidate {
    pub error: f64,
    pub rotation: [f64; 9],
    pub translation: [f64; 3],
}

pub struct Detector {
    ptr: *mut ShimDetector,
}

unsafe impl Send for Detector {}

impl Detector {
    pub fn new(
        family_name: &str,
        nthreads: u8,
        quad_decimate: f32,
        quad_sigma: f32,
        refine_edges: bool,
    ) -> Option<Self> {
        let cname = CString::new(family_name).ok()?;
        let ptr = unsafe {
            shim_detector_create(
                cname.as_ptr(),
                nthreads as c_int,
                quad_decimate,
                quad_sigma,
                refine_edges as c_int,
            )
        };
        if ptr.is_null() {
            None
        } else {
            Some(Self { ptr })
        }
    }

    pub fn detect(&mut self, buf: &[u8], width: i32, height: i32, stride: i32) -> Option<Detections> {
        let ptr = unsafe { shim_detect(self.ptr, buf.as_ptr(), width, height, stride) };
        if ptr.is_null() {
            None
        } else {
            Some(Detections { ptr })
        }
    }
}

impl Drop for Detector {
    fn drop(&mut self) {
        unsafe { shim_detector_destroy(self.ptr) };
    }
}

pub struct Detections {
    ptr: *mut ShimDetections,
}

impl Detections {
    pub fn len(&self) -> usize {
        unsafe { shim_detections_count(self.ptr) as usize }
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    pub fn get(&self, idx: usize) -> Option<Detection<'_>> {
        if idx < self.len() {
            Some(Detection {
                dets: self,
                idx: idx as i32,
            })
        } else {
            None
        }
    }

    pub fn iter(&self) -> impl Iterator<Item = Detection<'_>> {
        (0..self.len()).map(move |i| Detection {
            dets: self,
            idx: i as i32,
        })
    }
}

impl Drop for Detections {
    fn drop(&mut self) {
        unsafe { shim_detections_destroy(self.ptr) };
    }
}

pub struct Detection<'a> {
    dets: &'a Detections,
    idx: i32,
}

impl<'a> Detection<'a> {
    pub fn id(&self) -> i32 {
        unsafe { shim_detection_id(self.dets.ptr, self.idx) }
    }

    pub fn center(&self) -> [f64; 2] {
        let mut out = [0.0; 2];
        unsafe { shim_detection_center(self.dets.ptr, self.idx, out.as_mut_ptr()) };
        out
    }

    pub fn estimate_pose_orthogonal(&self, params: &TagParams, n_iters: i32) -> Option<PoseCandidate> {
        let mut err1 = 0.0f64;
        let mut err2 = -1.0f64;
        let mut r1 = [0.0f64; 9];
        let mut t1 = [0.0f64; 3];
        let mut r2 = [0.0f64; 9];
        let mut t2 = [0.0f64; 3];

        let rc = unsafe {
            shim_estimate_pose_orthogonal(
                self.dets.ptr,
                self.idx,
                params.tagsize,
                params.fx,
                params.fy,
                params.cx,
                params.cy,
                n_iters,
                &mut err1,
                r1.as_mut_ptr(),
                t1.as_mut_ptr(),
                &mut err2,
                r2.as_mut_ptr(),
                t2.as_mut_ptr(),
            )
        };
        if rc != 0 {
            return None;
        }

        let cand1 = PoseCandidate {
            error: err1,
            rotation: r1,
            translation: t1,
        };

        if err2 >= 0.0 {
            let cand2 = PoseCandidate {
                error: err2,
                rotation: r2,
                translation: t2,
            };
            Some(if cand1.error <= cand2.error { cand1 } else { cand2 })
        } else {
            Some(cand1)
        }
    }
}
