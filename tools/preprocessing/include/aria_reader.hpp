#pragma once

#include <memory>
#include <string>

#include <Eigen/Geometry>

#include <calibration/CameraCalibration.h>
#include <data_provider/VrsDataProvider.h>
#include <vrs/StreamId.h>

#include "frame.hpp"

namespace preprocessing {

struct CameraIntrinsics {
    double fx;
    double fy;
    double cx;
    double cy;
};

class AriaReader {
public:
    explicit AriaReader(const std::string& vrs_path);

    CameraIntrinsics camera_intrinsics() const;

    const Eigen::Isometry3d& T_device_camera() const;

    void read_and_send(BoundedQueue<DecodedFrame>& queue, size_t skip_factor);

private:
    std::shared_ptr<projectaria::tools::data_provider::VrsDataProvider> provider_;
    vrs::StreamId rgb_stream_;
    projectaria::tools::calibration::CameraCalibration camera_calib_;
    projectaria::tools::calibration::CameraCalibration pinhole_calib_;
    Eigen::Isometry3d T_device_camera_;
};

} // namespace preprocessing
