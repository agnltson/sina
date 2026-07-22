#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "worker.hpp"

namespace preprocessing {

void write_tags_config(const std::vector<TagObservation>& observations,
                        const std::string& output_path,
                        size_t min_observations);

} // namespace preprocessing
