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

#pragma once
#include <unordered_map>
#include "simulator_entity.h"

namespace simulator {

typedef std::unordered_map<std::string, std::vector<std::string>> PropMap;

// A class that scans and preprocesses entity information in a world
// It applies to any game that involves interactions between entities in a space
class WorldScanner {
  public:
    WorldScanner(TeachingEnvPtr game) : game_(game) { init_scan(); }

    void scan_agent();

    std::vector<Entity> goals_;   // all goals existing in the env
    std::vector<Entity> blocks_;  // all blocks existing in the env
    std::vector<Entity>
        unique_goals_;  // goals in the env that have a unique name
    std::vector<Entity>
        color_goals_;  // goals that have properly defined colors
    Entity agent_;
    size_t world_size_;

    // Initial scan of the world
    void init_scan();

    // whether a point is outside of a world
    bool outside_world(const Vec3& point);

    // Given two properties k1 and k2, return a property mapping
    // For example,
    // PropMap pm = get_property_mapping("name", "color");
    // std::vector<std::string> apple_colors = pm["apple"];
    // std::vector<std::string> red_names = pm["red"];
    PropMap get_property_mapping(std::string k1, std::string k2);

    std::string scan_agent_sent_from_env() {
        return game_->get_agent_sent_from_buffer();
    }

    void scan_agent(const std::vector<Entity>& entities);

    void scan_blocks(const std::vector<Entity>& entities);

    void scan_all_goals(const std::vector<Entity>& entities);

    void scan_unique_goals(std::vector<Entity>& entities,
                           std::unordered_map<std::string, int>& goal_counter);

    void scan_color_goals(std::vector<Entity>& entities);

    void scan_property_mappings(std::vector<Entity>& entities);

  protected:
    std::unordered_map<std::string, int> count_goals(
        std::vector<Entity>& entities);

  private:
    std::unordered_map<std::string, PropMap> property_mappings_;
    TeachingEnvPtr game_;
};

typedef std::shared_ptr<WorldScanner> ScannerPtr;

}  // namespace simulator
