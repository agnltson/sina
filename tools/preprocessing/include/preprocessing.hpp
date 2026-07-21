#pragma once

#include <string>

#include "config.hpp"

namespace preprocessing {

// Runs the full pipeline straight off a .vrs recording: reads camera-rgb
// frames + calibration from vrs_path via the Aria SDK on one thread while
// config.preprocessor.num_workers threads detect AprilTags and estimate
// poses, then aggregates the results into tags_world_config.json.
void preprocess(
    const Config& config,
    const std::string& vrs_path,
    const std::string& trajectory_path);

} // namespace preprocessing
