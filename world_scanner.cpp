// MIT License

// Copyright (c) 2017 Baidu Inc. All rights reserved.

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
/////////////////////////////////////////////////////////////////////////////////

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

} // namespace simulator
