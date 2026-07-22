#pragma once

#include <cstddef>
#include <optional>
#include <string>

namespace preprocessing {

struct PreprocessorConfig {
    std::optional<size_t> skip_factor;
    std::optional<size_t> num_workers;
    std::optional<size_t> queue_capacity;
    std::optional<size_t> min_observation;
    std::optional<size_t> max_hamming;
    double fx = 0.0;
    double fy = 0.0;
    double cx = 0.0;
    double cy = 0.0;
};

struct StreamingConfig {
    std::string profile;
    std::string ip;
    double fx = 0.0;
    double fy = 0.0;
    double cx = 0.0;
    double cy = 0.0;
};

struct ApriltagConfig {
    std::string tag_family;
    double tag_size_m = 0.0;
};

struct Config {
    PreprocessorConfig preprocessor;
    StreamingConfig streaming;
    ApriltagConfig apriltag;
};

Config load_config(const std::string& path);

} // namespace preprocessing
