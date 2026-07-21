#pragma once

#include <cstddef>
#include <optional>
#include <string>

namespace preprocessing {

// Mirrors PreprocessorConfig in the Rust config.rs. Optional fields use
// std::optional the same way Rust used Option<T>.
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

// Parses a TOML file with [preprocessor]/[streaming]/[apriltag] tables into
// a Config, mirroring load_config() in the Rust version. Throws
// std::runtime_error if a required key is missing or has the wrong type,
// or toml::parse_error (itself a std::runtime_error) on a syntax error.
Config load_config(const std::string& path);

} // namespace preprocessing
