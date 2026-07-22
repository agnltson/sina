#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Geometry>

namespace preprocessing {

struct PoseSample {
    int64_t timestamp_us;
    Eigen::Isometry3d isometry;
};

std::vector<PoseSample> load_poses(const std::string& path);

std::optional<Eigen::Isometry3d> interpolate_pose(
    const std::vector<PoseSample>& poses, int64_t target_us);

} // namespace preprocessing
