#include "worker.hpp"

#include <cmath>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <unordered_map>

#include <Eigen/Geometry>

extern "C" {
#include <apriltag/apriltag.h>
#include <apriltag/apriltag_pose.h>
#include <apriltag/common/matd.h>
#include <apriltag/tag16h5.h>
#include <apriltag/tag25h9.h>
#include <apriltag/tag36h11.h>
#include <apriltag/tagCircle21h7.h>
#include <apriltag/tagCircle49h12.h>
#include <apriltag/tagCustom48h12.h>
#include <apriltag/tagStandard41h12.h>
#include <apriltag/tagStandard52h13.h>
}

namespace preprocessing {

namespace {

constexpr int POSE_ESTIMATION_ITERS = 50;

// AprilTag ships one create()/destroy() pair per tag family (tag36h11_create,
// tagStandard41h12_create, ...) instead of a single "create by name"
// function -- this table is our lookup for that.
struct FamilyOps {
    apriltag_family_t* (*create)();
    void (*destroy)(apriltag_family_t*);
};

const std::unordered_map<std::string, FamilyOps>& family_table() {
    static const std::unordered_map<std::string, FamilyOps> table = {
        {"tag36h11", {tag36h11_create, tag36h11_destroy}},
        {"tag25h9", {tag25h9_create, tag25h9_destroy}},
        {"tag16h5", {tag16h5_create, tag16h5_destroy}},
        {"tagStandard41h12", {tagStandard41h12_create, tagStandard41h12_destroy}},
        {"tagStandard52h13", {tagStandard52h13_create, tagStandard52h13_destroy}},
        {"tagCircle21h7", {tagCircle21h7_create, tagCircle21h7_destroy}},
        {"tagCircle49h12", {tagCircle49h12_create, tagCircle49h12_destroy}},
        {"tagCustom48h12", {tagCustom48h12_create, tagCustom48h12_destroy}},
    };
    return table;
}

// Wraps an already tightly-packed (stride == width) grayscale buffer as an
// apriltag image_u8_t WITHOUT copying or allocating -- the pattern
// documented in the apriltag README for wrapping an existing buffer.
// AriaReader::read_and_send() already produces gray_data this way, so
// there's nothing to copy here.
image_u8_t wrap_gray_frame(const DecodedFrame& frame) {
    return image_u8_t{
        static_cast<int32_t>(frame.width),
        static_cast<int32_t>(frame.height),
        static_cast<int32_t>(frame.width), // stride == width: no padding
        const_cast<uint8_t*>(frame.gray_data.data()),
    };
}

// matd_t stores its data as a flat row-major array. MATD_EL(m, row, col)
// is the library's own indexing macro.
Eigen::Matrix3d matd_to_eigen_3x3(const matd_t* m) {
    Eigen::Matrix3d result;
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            result(row, col) = MATD_EL(m, row, col);
        }
    }
    return result;
}

Eigen::Vector3d matd_to_eigen_3x1(const matd_t* m) {
    return Eigen::Vector3d(MATD_EL(m, 0, 0), MATD_EL(m, 1, 0), MATD_EL(m, 2, 0));
}

} // namespace

std::vector<TagObservation> worker_loop(
    BoundedQueue<DecodedFrame>& queue,
    const std::vector<PoseSample>& csv_poses,
    const CameraParams& camera,
    const Eigen::Isometry3d& T_device_camera,
    const std::string& tag_family_str,
    int max_hamming) {
    auto family_it = family_table().find(tag_family_str);
    if (family_it == family_table().end()) {
        throw std::runtime_error("invalid family tag '" + tag_family_str + "'");
    }
    apriltag_family_t* tag_family = family_it->second.create();
    void (*family_destroy)(apriltag_family_t*) = family_it->second.destroy;

    apriltag_detector_t* detector = apriltag_detector_create();
    apriltag_detector_add_family_bits(detector, tag_family, max_hamming);

    std::vector<TagObservation> observations;
    DecodedFrame frame;
    while (queue.pop(frame)) {
        if (frame.width == 0 || frame.height == 0) {
            std::cerr << "frame " << frame.frame_index << ": degenerate dimensions ("
                      << frame.width << "x" << frame.height << "), frame skipped\n";
            continue;
        }
        if (frame.gray_data.size() != frame.width * frame.height) {
            std::cerr << "frame " << frame.frame_index << ": gray_data size "
                      << frame.gray_data.size() << " != " << frame.width << "x" << frame.height
                      << ", frame skipped\n";
            continue;
        }

        image_u8_t image = wrap_gray_frame(frame);
        zarray_t* detections = apriltag_detector_detect(detector, &image);

        if (zarray_size(detections) == 0) {
            apriltag_detections_destroy(detections);
            continue;
        }

        // frame.timestamp_ns comes straight from the VRS record now, so
        // there's no separate frame_timestamps_us lookup table to check
        // against anymore -- just convert to the microseconds
        // closed_loop_trajectory.csv poses are keyed on.
        int64_t target_us = frame.timestamp_ns / 1000;

        std::optional<Eigen::Isometry3d> world_device = interpolate_pose(csv_poses, target_us);
        if (!world_device.has_value()) {
            apriltag_detections_destroy(detections);
            continue;
        }

        for (int i = 0; i < zarray_size(detections); ++i) {
            apriltag_detection_t* detection = nullptr;
            zarray_get(detections, i, &detection);

            apriltag_detection_info_t info;
            info.det = detection;
            info.tagsize = camera.tag_size_m;
            info.fx = camera.fx;
            info.fy = camera.fy;
            info.cx = camera.cx;
            info.cy = camera.cy;

            // Zero-initialized so pose2.R stays a well-defined nullptr if
            // the library doesn't find a second local minimum.
            apriltag_pose_t pose1{};
            apriltag_pose_t pose2{};
            double err1 = 0.0;
            double err2 = 0.0;
            estimate_tag_pose_orthogonal_iteration(&info, &err1, &pose1, &err2, &pose2, POSE_ESTIMATION_ITERS);

            bool has_second = pose2.R != nullptr;
            apriltag_pose_t* best = &pose1;
            double best_error = err1;
            if (has_second && err2 < best_error) {
                best = &pose2;
                best_error = err2;
            }

            bool valid_shape = best->R != nullptr && best->t != nullptr &&
                                best->R->nrows == 3 && best->R->ncols == 3 &&
                                best->t->nrows == 3 && best->t->ncols == 1;

            if (valid_shape) {
                Eigen::Vector3d translation = matd_to_eigen_3x1(best->t);
                Eigen::Matrix3d rotation_matrix = matd_to_eigen_3x3(best->R);
                double determinant = rotation_matrix.determinant();

                bool translation_finite = translation.allFinite();
                bool rotation_valid = std::isfinite(determinant) && std::abs(determinant - 1.0) <= 0.1;

                if (translation_finite && rotation_valid) {
                    Eigen::Isometry3d cam_tag = Eigen::Isometry3d::Identity();
                    cam_tag.translate(translation);
                    cam_tag.rotate(Eigen::Quaterniond(rotation_matrix));

                    // The fix: tag poses come out of AprilTag in the
                    // camera frame, but csv_poses is world_device. Going
                    // straight from world_device to cam_tag skips the
                    // camera's physical mounting offset on the device and
                    // was producing wrong positions -- T_device_camera
                    // has to sit in the middle of the chain.
                    Eigen::Isometry3d world_tag = world_device.value() * T_device_camera * cam_tag;

                    /*Eigen::Isometry3d world_device_pose = world_device.value();
                    Eigen::Isometry3d world_tag_corrected = world_device_pose * T_device_camera * cam_tag;

                    // Logs de débogage
                    if (detection->id == 0) {
                        std::cout << "Frame " << frame.frame_index << ", Tag " << detection->id << "\n";
                        std::cout << "  world_device translation: " << world_device_pose.translation().transpose() << "\n";
                        std::cout << "  T_device_camera translation: " << T_device_camera.translation().transpose() << "\n";
                        std::cout << "  cam_tag translation (AprilTag): " << translation.transpose() << "\n";
                        
                        // Calcul intermédiaire : position du tag dans le repère device
                        Eigen::Vector3d device_tag = T_device_camera * cam_tag.translation();
                        std::cout << "  device_tag translation (T_device_camera * cam_tag): " << device_tag.transpose() << "\n";
                        
                        // Position finale
                        std::cout << "  world_tag translation (corrigé): " << world_tag_corrected.translation().transpose() << "\n";
                        std::cout << "  reprojection error: " << best_error << "\n\n";
                    }*/

                    Eigen::Quaterniond q(world_tag.rotation());

                    TagObservation obs;
                    obs.tag_id = detection->id;
                    obs.translation_world[0] = world_tag.translation().x();
                    obs.translation_world[1] = world_tag.translation().y();
                    obs.translation_world[2] = world_tag.translation().z();
                    obs.quaternion_world_wxyz[0] = q.w();
                    obs.quaternion_world_wxyz[1] = q.x();
                    obs.quaternion_world_wxyz[2] = q.y();
                    obs.quaternion_world_wxyz[3] = q.z();
                    obs.reprojection_error = best_error;
                    observations.push_back(obs);
                }
            }

            if (pose1.R) matd_destroy(pose1.R);
            if (pose1.t) matd_destroy(pose1.t);
            if (has_second) {
                if (pose2.R) matd_destroy(pose2.R);
                if (pose2.t) matd_destroy(pose2.t);
            }
        }

        apriltag_detections_destroy(detections);
    }

    apriltag_detector_destroy(detector);
    family_destroy(tag_family);

    return observations;
}

} // namespace preprocessing
