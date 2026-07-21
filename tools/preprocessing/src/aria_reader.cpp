#include "aria_reader.hpp"

#include <iostream>
#include <optional>
#include <stdexcept>
#include <utility>
#include <variant>
#include <vector>

#include <calibration/utility/Distort.h>
#include <image/Image.h>
#include <image/ManagedImage.h>
#include <image/utility/Debayer.h>

namespace preprocessing {

AriaReader::AriaReader(const std::string& vrs_path) {
    using namespace projectaria::tools;

    // createVrsDataProvider returns a plain (possibly null) shared_ptr, not
    // an optional -- check against nullptr, not .has_value().
    provider_ = data_provider::createVrsDataProvider(vrs_path);
    if (!provider_) {
        throw std::runtime_error("cannot open VRS file: " + vrs_path);
    }

    std::optional<vrs::StreamId> stream_id = provider_->getStreamIdFromLabel("camera-rgb");
    if (!stream_id.has_value()) {
        throw std::runtime_error("no camera-rgb stream in VRS file: " + vrs_path);
    }
    rgb_stream_ = *stream_id;

    std::optional<calibration::DeviceCalibration> device_calib = provider_->getDeviceCalibration();
    if (!device_calib.has_value()) {
        throw std::runtime_error("VRS file has no device calibration: " + vrs_path);
    }

    std::optional<calibration::CameraCalibration> camera_calib =
        device_calib->getCameraCalib("camera-rgb");
    if (!camera_calib.has_value()) {
        throw std::runtime_error("no camera-rgb calibration in VRS file: " + vrs_path);
    }
    camera_calib_ = *camera_calib;

    // This is the device_camera transform that was missing before: the
    // physical rigid transform from the RGB camera frame to the device
    // frame, straight from the VRS calibration blob. Sophus::SE3d doesn't
    // interoperate directly with Eigen::Isometry3d, so we copy its
    // rotation matrix + translation into one ourselves.
    const Sophus::SE3d& t_device_camera = camera_calib_.getT_Device_Camera();
    T_device_camera_ = Eigen::Isometry3d::Identity();
    T_device_camera_.linear() = t_device_camera.rotationMatrix();
    T_device_camera_.translation() = t_device_camera.translation();

    std::cout << "\n=== T_DEVICE_CAMERA ===\n";
    std::cout << T_device_camera_.matrix() << "\n";

    // read_and_send() undistorts every frame to a plain pinhole model at
    // the camera's native resolution before pushing it onto the queue, so
    // AprilTag pose estimation never has to deal with the RGB camera's
    // native fisheye distortion. T_Device_Camera is carried over unchanged
    // since undistortion only changes the projection model, not where the
    // camera physically sits on the device.
    Eigen::Vector2i image_size = camera_calib_.getImageSize();
    double focal_length = camera_calib_.getFocalLengths().mean();
    pinhole_calib_ = calibration::getLinearCameraCalibration(
        image_size.x(), image_size.y(), focal_length, "camera-rgb", t_device_camera);
}

CameraIntrinsics AriaReader::camera_intrinsics() const {
    Eigen::Vector2d focal = pinhole_calib_.getFocalLengths();
    Eigen::Vector2d principal = pinhole_calib_.getPrincipalPoint();
    return CameraIntrinsics{focal.x(), focal.y(), principal.x(), principal.y()};
}

const Eigen::Isometry3d& AriaReader::T_device_camera() const {
    return T_device_camera_;
}

void AriaReader::read_and_send(BoundedQueue<DecodedFrame>& queue, size_t skip_factor) {
    using namespace projectaria::tools;

    size_t num_frames = provider_->getNumData(rgb_stream_);

    for (size_t i = 0; i < num_frames; i += skip_factor) {
        //std::cout << "Sending frame " << i << std::endl;
        auto image_data_and_record = provider_->getImageDataByIndex(rgb_stream_, static_cast<int>(i));
        const data_provider::ImageData& image_data = image_data_and_record.first;
        const data_provider::ImageDataRecord& record = image_data_and_record.second;

        std::optional<image::ImageVariant> variant = image_data.imageVariant();
        if (!variant.has_value()) {
            std::cerr << "frame " << i << ": no image data, frame skipped\n";
            continue;
        }

        // Depending on the recording's capture profile, "camera-rgb" is
        // either raw single-channel Bayer-mosaic (RGGB) data that must be
        // debayered before any resampling (resampling the raw mosaic
        // directly would produce garbage colors), or already demosaiced
        // RGB delivered straight by the player. Handle both rather than
        // assuming one.
        image::ManagedImage3U8 debayered; // only populated in the raw-Bayer branch
        image::ImageVariant rgb_variant;
        if (std::holds_alternative<image::ImageU8>(*variant)) {
            debayered = image::debayer(std::get<image::ImageU8>(*variant));
            rgb_variant = debayered;
        } else if (std::holds_alternative<image::Image3U8>(*variant)) {
            rgb_variant = *variant;
        } else {
            std::cerr << "frame " << i << ": unsupported pixel format "
                      << vrs::toString(image_data.getPixelFormat()) << ", frame skipped\n";
            continue;
        }

        image::ManagedImageVariant undistorted_variant =
            calibration::distortByCalibration(rgb_variant, pinhole_calib_, camera_calib_);
        const image::ManagedImage3U8& undistorted = std::get<image::ManagedImage3U8>(undistorted_variant);

        size_t width = undistorted.width();
        size_t height = undistorted.height();

        // AprilTag only needs a single grayscale channel; pack it tightly
        // (stride == width) so worker.cpp can wrap it as an apriltag
        // image_u8_t with no further copying.
        std::vector<uint8_t> gray_data(width * height);
        for (size_t y = 0; y < height; ++y) {
            for (size_t x = 0; x < width; ++x) {
                const Eigen::Matrix<uint8_t, 3, 1>& rgb =
                    undistorted(static_cast<int>(x), static_cast<int>(y));
                // Rec. 601 luma weights.
                gray_data[y * width + x] =
                    static_cast<uint8_t>(0.299 * rgb(0) + 0.587 * rgb(1) + 0.114 * rgb(2));
            }
        }

        queue.push(DecodedFrame{i, record.captureTimestampNs, width, height, std::move(gray_data)});
    }

    queue.close();
}

} // namespace preprocessing
