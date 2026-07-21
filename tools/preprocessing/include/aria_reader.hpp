#pragma once

#include <memory>
#include <string>

#include <Eigen/Geometry>

#include <calibration/CameraCalibration.h>
#include <data_provider/VrsDataProvider.h>
#include <vrs/StreamId.h>

#include "frame.hpp"

namespace preprocessing {

// Undistorted pinhole intrinsics -- what estimate_tag_pose_orthogonal_iteration
// should be called with, since read_and_send() has already undistorted
// every frame to this model before pushing it onto the queue. Doesn't
// include tag_size_m: that's a config.toml value, not something the SDK
// can tell us, so preprocessing.cpp combines the two into a CameraParams
// (see worker.hpp) itself.
struct CameraIntrinsics {
    double fx;
    double fy;
    double cx;
    double cy;
};

// Reads Aria "camera-rgb" frames + calibration directly from a .vrs file
// via the Aria SDK (projectaria_tools), replacing the old ffmpeg mp4
// decoder entirely -- no more separate video.mp4 or
// mp4_to_vrs_time_ns.json, since the SDK gives us both images and their
// real capture timestamps straight from the recording.
class AriaReader {
public:
    explicit AriaReader(const std::string& vrs_path);

    // Undistorted intrinsics matching every frame produced by
    // read_and_send() (see camera_intrinsics_ construction in the .cpp:
    // it's the same pinhole model frames are undistorted to).
    CameraIntrinsics camera_intrinsics() const;

    // Rigid transform from the RGB camera frame to the device frame
    // (T_device_camera), read straight from the VRS calibration. This is
    // the piece that was missing before: tag poses are estimated in the
    // *camera* frame, but closed_loop_trajectory.csv gives world_device
    // poses, so world_tag must go through
    // world_device * T_device_camera * cam_tag, not world_device * cam_tag
    // directly.
    const Eigen::Isometry3d& T_device_camera() const;

    // Reads every skip_factor-th "camera-rgb" frame, debayers +
    // undistorts + grayscales it, and pushes it onto queue; closes queue
    // when done regardless of how it exits. Meant to run on its own
    // thread, mirroring decode_and_send() in the old ffmpeg version.
    void read_and_send(BoundedQueue<DecodedFrame>& queue, size_t skip_factor);

private:
    std::shared_ptr<projectaria::tools::data_provider::VrsDataProvider> provider_;
    vrs::StreamId rgb_stream_;
    projectaria::tools::calibration::CameraCalibration camera_calib_;
    projectaria::tools::calibration::CameraCalibration pinhole_calib_;
    Eigen::Isometry3d T_device_camera_;
};

} // namespace preprocessing
