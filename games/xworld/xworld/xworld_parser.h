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
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include "simulator_entity.h"
#include "xitem.h"

namespace simulator {
namespace xwd {

namespace pt = boost::property_tree;

// class that parses an xworld configuration file
class XWorldParser {
  public:
    XWorldParser(bool print_xworld_config,
                 const std::string& world_config,
                 int curriculum_learning = 0);

    void reset_config(bool print = false) { reset_config(world_conf_, print); }

    // this enables the teacher to change the env during training
    void reset_config(const std::string& world_config, bool print = false);

    void get_configuration(WorldUnitList& ul, int& height, int& width) {
        ul = ul_;
        height = height_;
        width = width_;
    }

    void get_all_possible_objects(std::vector<Entity>& objects);

    ItemInfo get_next_agent();

    // generate a number between low and high according to the curriculum
    int curriculum_number(int low,
                          int high,
                          int num_games,
                          int curriculum_games);

  private:
    int curriculum_learning_;
    int total_items_;
    int num_games_so_far_;  // how many times the parser has been called
                            // used for curriculum learning, increasing the game
                            // difficulty gradually
    std::unordered_map<std::string, ItemType> item_types_;
    std::unordered_map<std::string, ItemInfo> ul_map_;
    std::vector<ItemInfo> agent_que_;  // record agents
    size_t agent_que_idx_;
    WorldUnitList ul_;
    std::vector<ItemInfo> all_objects_;
    int height_;
    int width_;
    size_t levels_;  // how many levels the map has
                     // suppose the map is n x n, then the first level has
                     // an empty ground of (n-levels_+1) x (n-levels_+1)
                     // the empty ground is surrouned by blocks
    std::string world_conf_;
    std::unique_ptr<pt::ptree> conf_root_;

    std::vector<int> read_json_vector(const pt::ptree::value_type& node);

    std::vector<std::vector<int>> read_json_matrix(
        const pt::ptree::value_type& node);

    int select_number_from_range(int low, int high, bool curriculum_learning);

    int get_total_num_of_items(const pt::ptree::value_type& item);

    void record_all_possible_objects(
        const std::vector<std::string>& all_valid_classes);

    // lower level has more surrounding blocks so that the agent's movement is
    // constrained
    // for curriculum learning
    void set_map_level();

    std::string sample_schedule(const pt::ptree::value_type& item);

    void select_item_classes(std::string type,
                             const std::vector<std::string>& categories,
                             std::vector<std::string>& all_classes);

    void instantiate_items(size_t total,
                           const pt::ptree::value_type& item,
                           bool print = false);

    // For a small chance we put all the items in a row
    // This will only change the locations of the items
    void put_items_in_a_row(double prob, ItemType t);
};
}
}  // namespace simulator::xwd
