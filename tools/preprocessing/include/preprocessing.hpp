#pragma once

#include <string>

#include "config.hpp"

namespace preprocessing {

void preprocess(
    const Config& config,
    const std::string& vrs_path,
    const std::string& trajectory_path);

} // namespace preprocessing
