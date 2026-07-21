#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Geometry>

namespace preprocessing {

// Rust's nalgebra::Isometry3<f64> becomes Eigen::Isometry3d: a rigid
// transform (rotation + translation), the direct Eigen equivalent.
struct PoseSample {
    int64_t timestamp_us;
    Eigen::Isometry3d isometry;
};

// Reads closed_loop_trajectory.csv (comma-separated, header row with column
// names) and returns poses sorted by timestamp_us, mirroring load_poses()
// in the Rust version. Throws std::runtime_error if the file can't be
// opened or a required column is missing.
std::vector<PoseSample> load_poses(const std::string& path);

// Finds the two pose samples surrounding target_us and linearly
// interpolates translation / slerps rotation between them. Returns
// std::nullopt if target_us is before the first or after the last sample,
// mirroring interpolate_pose()'s Option<Isometry3<f64>> in Rust.
std::optional<Eigen::Isometry3d> interpolate_pose(
    const std::vector<PoseSample>& poses, int64_t target_us);

} // namespace preprocessing
