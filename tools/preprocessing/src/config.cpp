#include "config.hpp"

#include <stdexcept>
#include <string>

#include <toml++/toml.h>

namespace preprocessing {

namespace {

const toml::table& require_table(const toml::table& t, std::string_view key) {
    const toml::node* node = t.get(key);
    if (!node || !node->is_table()) {
        throw std::runtime_error("missing table: " + std::string(key));
    }
    return *node->as_table();
}

double require_double(const toml::table& t, std::string_view key) {
    auto v = t[key].value<double>();
    if (!v) {
        throw std::runtime_error("missing or non-numeric key: " + std::string(key));
    }
    return *v;
}

std::string require_string(const toml::table& t, std::string_view key) {
    auto v = t[key].value<std::string>();
    if (!v) {
        throw std::runtime_error("missing or non-string key: " + std::string(key));
    }
    return *v;
}

std::optional<size_t> optional_size(const toml::table& t, std::string_view key) {
    auto v = t[key].value<int64_t>();
    if (!v) {
        return std::nullopt;
    }
    return static_cast<size_t>(*v);
}

} // namespace

Config load_config(const std::string& path) {
    toml::table root = toml::parse_file(path);

    Config config;

    const toml::table& pre = require_table(root, "preprocessor");
    config.preprocessor.skip_factor = optional_size(pre, "skip_factor");
    config.preprocessor.num_workers = optional_size(pre, "num_workers");
    config.preprocessor.queue_capacity = optional_size(pre, "queue_capacity");
    config.preprocessor.min_observation = optional_size(pre, "min_observation");
    config.preprocessor.max_hamming = optional_size(pre, "max_hamming");
    config.preprocessor.fx = require_double(pre, "fx");
    config.preprocessor.fy = require_double(pre, "fy");
    config.preprocessor.cx = require_double(pre, "cx");
    config.preprocessor.cy = require_double(pre, "cy");

    const toml::table& streaming = require_table(root, "streaming");
    config.streaming.profile = require_string(streaming, "profile");
    config.streaming.ip = require_string(streaming, "ip");
    config.streaming.fx = require_double(streaming, "fx");
    config.streaming.fy = require_double(streaming, "fy");
    config.streaming.cx = require_double(streaming, "cx");
    config.streaming.cy = require_double(streaming, "cy");

    const toml::table& at = require_table(root, "apriltag");
    config.apriltag.tag_family = require_string(at, "tag_family");
    config.apriltag.tag_size_m = require_double(at, "tag_size_m");

    return config;
}

} // namespace preprocessing
