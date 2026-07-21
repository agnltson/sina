#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "worker.hpp"

namespace preprocessing {

// Aggregates observations per tag id (mean position, positional std-dev,
// best-error orientation) and writes them as pretty-printed JSON to
// output_path. Tags seen fewer than min_observations times are dropped.
//
// This is the C++ port of write_config() from config.rs -- renamed to
// avoid clashing with the TOML Config type in config.hpp/cpp.
void write_tags_config(const std::vector<TagObservation>& observations,
                        const std::string& output_path,
                        size_t min_observations);

} // namespace preprocessing
