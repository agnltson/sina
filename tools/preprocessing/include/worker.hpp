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

std::vector<TagObservation> worker_loop(
    BoundedQueue<DecodedFrame>& queue,
    const std::vector<PoseSample>& csv_poses,
    const CameraParams& camera,
    const Eigen::Isometry3d& T_device_camera,
    const std::string& tag_family_str,
    int max_hamming);

} // namespace preprocessing
