#include "pose.hpp"

#include <algorithm>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

namespace preprocessing {

namespace {

std::vector<std::string> split_csv_line(const std::string& line) {
    std::vector<std::string> fields;
    std::stringstream ss(line);
    std::string field;
    while (std::getline(ss, field, ',')) {
        fields.push_back(field);
    }
    return fields;
}

} // namespace

std::vector<PoseSample> load_poses(const std::string& path) {
    std::ifstream file(path);
    if (!file) {
        throw std::runtime_error("cannot open trajectory CSV: " + path);
    }

    std::string header_line;
    if (!std::getline(file, header_line)) {
        throw std::runtime_error("trajectory CSV is empty: " + path);
    }
    std::vector<std::string> headers = split_csv_line(header_line);
    std::unordered_map<std::string, size_t> col_index;
    for (size_t i = 0; i < headers.size(); ++i) {
        col_index[headers[i]] = i;
    }

    auto column = [&](const std::vector<std::string>& fields, const char* name) -> const std::string& {
        auto it = col_index.find(name);
        if (it == col_index.end()) {
            throw std::runtime_error(std::string("missing CSV column: ") + name);
        }
        return fields.at(it->second);
    };

    std::vector<PoseSample> poses;
    std::string line;
    while (std::getline(file, line)) {
        if (line.empty()) {
            continue;
        }
        std::vector<std::string> fields = split_csv_line(line);

        int64_t timestamp_us = std::stoll(column(fields, "tracking_timestamp_us"));
        double tx = std::stod(column(fields, "tx_world_device"));
        double ty = std::stod(column(fields, "ty_world_device"));
        double tz = std::stod(column(fields, "tz_world_device"));
        double qx = std::stod(column(fields, "qx_world_device"));
        double qy = std::stod(column(fields, "qy_world_device"));
        double qz = std::stod(column(fields, "qz_world_device"));
        double qw = std::stod(column(fields, "qw_world_device"));

        Eigen::Isometry3d isometry = Eigen::Isometry3d::Identity();
        isometry.translate(Eigen::Vector3d(tx, ty, tz));
        isometry.rotate(Eigen::Quaterniond(qw, qx, qy, qz).normalized());

        poses.push_back(PoseSample{timestamp_us, isometry});
    }

    std::sort(poses.begin(), poses.end(), [](const PoseSample& a, const PoseSample& b) {
        return a.timestamp_us < b.timestamp_us;
    });

    return poses;
}

std::optional<Eigen::Isometry3d> interpolate_pose(
    const std::vector<PoseSample>& poses, int64_t target_us) {
    auto it = std::lower_bound(
        poses.begin(), poses.end(), target_us,
        [](const PoseSample& sample, int64_t t) { return sample.timestamp_us < t; });

    if (it != poses.end() && it->timestamp_us == target_us) {
        return it->isometry;
    }

    size_t idx = static_cast<size_t>(it - poses.begin());
    if (idx == 0 || idx == poses.size()) {
        return std::nullopt;
    }

    const PoseSample& before = poses[idx - 1];
    const PoseSample& after = poses[idx];

    double span = static_cast<double>(after.timestamp_us - before.timestamp_us);
    if (span <= 0.0) {
        return before.isometry;
    }
    double t = static_cast<double>(target_us - before.timestamp_us) / span;

    Eigen::Vector3d translation =
        before.isometry.translation() + t * (after.isometry.translation() - before.isometry.translation());

    Eigen::Quaterniond rotation =
        Eigen::Quaterniond(before.isometry.rotation()).slerp(t, Eigen::Quaterniond(after.isometry.rotation()));

    Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
    result.translate(translation);
    result.rotate(rotation);
    return result;
}

} // namespace preprocessing
