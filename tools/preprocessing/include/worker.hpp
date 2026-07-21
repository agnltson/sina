#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include "frame.hpp"
#include "pose.hpp"

namespace preprocessing {

struct TagObservation {
    int tag_id;
    double translation_world[3];
    double quaternion_world_wxyz[4];
    double reprojection_error;
};

struct CameraParams {
    double fx;
    double fy;
    double cx;
    double cy;
    double tag_size_m;
};

// Pulls frames off queue until it's closed and drained, runs AprilTag
// detection + pose estimation on each one, and returns every observation
// found. Meant to run on its own std::thread -- one call per worker,
// mirroring worker_loop() in the Rust version. Throws std::runtime_error
// if tag_family_str isn't a known family name.
//
// T_device_camera is the rigid transform from the RGB camera frame to the
// device frame (from AriaReader::T_device_camera()): tag poses come out of
// AprilTag in the *camera* frame, but csv_poses holds world_device poses,
// so composing world_tag = world_device * cam_tag directly (without going
// through the camera's mounting offset first) was producing wrong
// positions.
std::vector<TagObservation> worker_loop(
    BoundedQueue<DecodedFrame>& queue,
    const std::vector<PoseSample>& csv_poses,
    const CameraParams& camera,
    const Eigen::Isometry3d& T_device_camera,
    const std::string& tag_family_str,
    int max_hamming);

} // namespace preprocessing
