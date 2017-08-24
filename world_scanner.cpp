// Copyright (c) 2017 Baidu Inc. All Rights Reserved.

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "world_scanner.h"

namespace simulator {

void WorldScanner::init_scan() {
    std::vector<Entity> entities;
    game_->get_all_entities(entities);
    auto goal_counter = count_goals(entities);
    scan_agent(entities);
    scan_blocks(entities);
    scan_all_goals(entities);
    scan_unique_goals(entities, goal_counter);
    scan_color_goals(entities);
    scan_property_mappings(entities);
    world_size_ = game_->world_size();
}

std::unordered_map<std::string, int> WorldScanner::count_goals(
    std::vector<Entity>& entities) {
    std::unordered_map<std::string, int> goal_counter;
    for (auto& e : entities) {
        if (e.type == "goal") {
            if (goal_counter.count(e.property("name")) == 0) {
                goal_counter[e.property("name")] = 1;
            } else {
                goal_counter[e.property("name")]++;
            }
        }
    }
    return goal_counter;
}

bool WorldScanner::outside_world(const Vec3& point) {
    Entity e;
    e.location = point;
    return !game_->entity_valid(e);
}

void WorldScanner::scan_agent(const std::vector<Entity>& entities) {
    for (const auto& e : entities) {
        if (e.type == "agent") {
            agent_ = e;
            return;
        }
    }
}

void WorldScanner::scan_agent() {
    std::vector<Entity> entities;
    game_->get_all_entities(entities);
    scan_agent(entities);
}

void WorldScanner::scan_blocks(const std::vector<Entity>& entities) {
    blocks_.clear();
    for (const auto& e : entities) {
        if (e.type == "block") {
            blocks_.push_back(e);
        }
    }
}

void WorldScanner::scan_all_goals(const std::vector<Entity>& entities) {
    goals_.clear();
    for (const auto& e : entities) {
        if (e.type == "goal") {
            goals_.push_back(e);
        }
    }
}

void WorldScanner::scan_unique_goals(
    std::vector<Entity>& entities,
    std::unordered_map<std::string, int>& goal_counter) {
    unique_goals_.clear();
    for (auto& e : entities) {
        if (e.type == "goal" && goal_counter[e.property("name")] == 1) {
            unique_goals_.push_back(e);
        }
    }
}

void WorldScanner::scan_color_goals(std::vector<Entity>& entities) {
    color_goals_.clear();
    for (auto& e : entities) {
        if (e.type == "goal" && game_->color_defined(e.property("color"))) {
            color_goals_.push_back(e);
        }
    }
}

void WorldScanner::scan_property_mappings(std::vector<Entity>& entities) {
    property_mappings_.clear();
    for (auto& e : entities) {
        if (e.type == "goal") {
            auto keys = e.all_properties();
            for (const auto& k1 : keys) {
                for (const auto& k2 : keys) {
                    if (k1 != k2) {
                        std::string key = k1 + "|" + k2;
                        auto& prop_mapping = property_mappings_[key];
                        std::string v1 = e.property(k1);
                        std::string v2 = e.property(k2);
                        prop_mapping[v1].push_back(v2);
                    }
                }
            }
        }
    }
}

PropMap WorldScanner::get_property_mapping(std::string k1, std::string k2) {
    std::string key = k1 + "|" + k2;
    CHECK_GT(property_mappings_.count(key), 0);
    return property_mappings_[key];
}

}  // namespace simulator
