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

    std::vector<Entity> goals_;        // all goals existing in the env
    std::vector<Entity> blocks_;       // all blocks existing in the env
    std::vector<Entity> unique_goals_; // goals in the env that have a unique name
    std::vector<Entity> color_goals_;  // goals that have properly defined colors
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

} // namespace simulator
