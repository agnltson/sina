#include "tags_output.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <stdexcept>
#include <unordered_map>

#include <nlohmann/json.hpp>

namespace preprocessing {

void write_tags_config(const std::vector<TagObservation>& observations,
                        const std::string& output_path,
                        size_t min_observations) {
    std::unordered_map<int, std::vector<const TagObservation*>> by_tag;
    for (const auto& obs : observations) {
        by_tag[obs.tag_id].push_back(&obs);
    }

    nlohmann::json tags = nlohmann::json::array();

    for (const auto& [tag_id, obs_list] : by_tag) {
        if (obs_list.size() < min_observations) {
            continue;
        }

        double n = static_cast<double>(obs_list.size());
        double mean[3] = {0.0, 0.0, 0.0};
        for (const auto* obs : obs_list) {
            for (int axis = 0; axis < 3; ++axis) {
                mean[axis] += obs->translation_world[axis] / n;
            }
        }

        double variance[3] = {0.0, 0.0, 0.0};
        for (const auto* obs : obs_list) {
            for (int axis = 0; axis < 3; ++axis) {
                double diff = obs->translation_world[axis] - mean[axis];
                variance[axis] += diff * diff / n;
            }
        }
        double std_dev[3] = {std::sqrt(variance[0]), std::sqrt(variance[1]), std::sqrt(variance[2])};

        const TagObservation* best = *std::min_element(
            obs_list.begin(), obs_list.end(),
            [](const TagObservation* a, const TagObservation* b) {
                return a->reprojection_error < b->reprojection_error;
            });

        nlohmann::json entry;
        entry["id"] = tag_id;
        entry["position_world"] = {mean[0], mean[1], mean[2]};
        entry["orientation_world_wxyz"] = {
            best->quaternion_world_wxyz[0], best->quaternion_world_wxyz[1],
            best->quaternion_world_wxyz[2], best->quaternion_world_wxyz[3]};
        entry["num_observations"] = obs_list.size();
        entry["position_std_m"] = {std_dev[0], std_dev[1], std_dev[2]};
        entry["best_reprojection_error"] = best->reprojection_error;

        tags.push_back(std::move(entry));
    }

    std::sort(tags.begin(), tags.end(), [](const nlohmann::json& a, const nlohmann::json& b) {
        return a["id"].get<int>() < b["id"].get<int>();
    });

    nlohmann::json root;
    root["tags"] = tags;

    std::ofstream out(output_path);
    if (!out) {
        throw std::runtime_error("cannot open output file: " + output_path);
    }
    out << root.dump(2);
}

} // namespace preprocessing
