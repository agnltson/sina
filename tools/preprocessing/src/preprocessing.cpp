#include "preprocessing.hpp"

#include <iostream>
#include <stdexcept>
#include <thread>
#include <vector>

#include "aria_reader.hpp"
#include "pose.hpp"
#include "tags_output.hpp"
#include "worker.hpp"

namespace preprocessing {

namespace {

constexpr size_t DEFAULT_NUM_WORKERS = 2;
constexpr size_t DEFAULT_QUEUE_CAPACITY = 8;
constexpr size_t DEFAULT_MAX_HAMMING = 1;
constexpr size_t DEFAULT_MIN_OBSERVATION = 5;

} // namespace

void preprocess(
    const Config& config,
    const std::string& vrs_path,
    const std::string& trajectory_path) {
    std::cout << "Opening VRS: " << vrs_path << "\n";

    std::cout << "Loading trajectory CSV...\n";
    std::vector<PoseSample> csv_poses = load_poses(trajectory_path);
    std::cout << "  " << csv_poses.size() << " poses loaded.\n";

    AriaReader aria(vrs_path);
    CameraIntrinsics intrinsics = aria.camera_intrinsics();
    CameraParams camera{
        intrinsics.fx,
        intrinsics.fy,
        intrinsics.cx,
        intrinsics.cy,
        config.apriltag.tag_size_m,
    };
    Eigen::Isometry3d T_device_camera = aria.T_device_camera();

    size_t num_workers = config.preprocessor.num_workers.value_or([] {
        unsigned int n = std::thread::hardware_concurrency();
        return n > 0 ? static_cast<size_t>(n) : DEFAULT_NUM_WORKERS;
    }());
    std::cout << "Launching pipeline with " << num_workers << " workers...\n";

    size_t queue_capacity = config.preprocessor.queue_capacity.value_or(DEFAULT_QUEUE_CAPACITY);
    BoundedQueue<DecodedFrame> queue(queue_capacity);

    size_t skip_factor = config.preprocessor.skip_factor.value_or(1);

    std::string reader_error;
    std::thread reader_thread([&] {
        try {
            aria.read_and_send(queue, skip_factor);
        } catch (const std::exception& e) {
            reader_error = e.what();
            queue.close();
        }
    });

    int max_hamming = static_cast<int>(config.preprocessor.max_hamming.value_or(DEFAULT_MAX_HAMMING));

    std::vector<std::thread> worker_threads;
    std::vector<std::vector<TagObservation>> worker_results(num_workers);
    std::vector<std::string> worker_errors(num_workers);
    for (size_t worker_id = 0; worker_id < num_workers; ++worker_id) {
        worker_threads.emplace_back([&, worker_id] {
            try {
                worker_results[worker_id] = worker_loop(
                    queue, csv_poses, camera, T_device_camera,
                    config.apriltag.tag_family, max_hamming);
            } catch (const std::exception& e) {
                worker_errors[worker_id] = e.what();
            }
        });
    }

    reader_thread.join();
    for (auto& t : worker_threads) {
        t.join();
    }

    if (!reader_error.empty()) {
        throw std::runtime_error("Aria reader failed: " + reader_error);
    }
    for (size_t worker_id = 0; worker_id < num_workers; ++worker_id) {
        if (!worker_errors[worker_id].empty()) {
            throw std::runtime_error(
                "worker " + std::to_string(worker_id) + " failed: " + worker_errors[worker_id]);
        }
    }

    std::vector<TagObservation> all_observations;
    for (auto& result : worker_results) {
        all_observations.insert(all_observations.end(), result.begin(), result.end());
    }

    std::cout << all_observations.size() << " tags observed, aggregation...\n";
    size_t min_observation = config.preprocessor.min_observation.value_or(DEFAULT_MIN_OBSERVATION);
    write_tags_config(all_observations, "tags_world_config.json", min_observation);
    std::cout << "AprilTag config written in tags_world_config.json\n";
}

} // namespace preprocessing
