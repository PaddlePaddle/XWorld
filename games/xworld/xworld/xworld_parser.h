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
#include <unordered_set>
#include <queue>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include "xitem.h"
#include "simulator_entity.h"

namespace simulator { namespace xwd {

namespace pt = boost::property_tree;

// class that parses an xworld configuration file
class XWorldParser {
  public:
    XWorldParser(bool print_xworld_config,
                 const std::string& world_config, int curriculum_learning = 0);

    void reset_config(bool print = false) {
        reset_config(world_conf_, print);
    }

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
    int curriculum_number(int low, int high, int num_games, int curriculum_games);

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
    size_t levels_;         // how many levels the map has
                            // suppose the map is n x n, then the first level has
                            // an empty ground of (n-levels_+1) x (n-levels_+1)
                            // the empty ground is surrouned by blocks
    std::string world_conf_;
    std::unique_ptr<pt::ptree> conf_root_;

    std::vector<int> read_json_vector(const pt::ptree::value_type& node);

    std::vector<std::vector<int>> read_json_matrix(const pt::ptree::value_type& node);

    int select_number_from_range(int low, int high, bool curriculum_learning);

    int get_total_num_of_items(const pt::ptree::value_type& item);

    void record_all_possible_objects(const std::vector<std::string>& all_valid_classes);

    // lower level has more surrounding blocks so that the agent's movement is constrained
    // for curriculum learning
    void set_map_level();

    std::string sample_schedule(const pt::ptree::value_type& item);

    void select_item_classes(std::string type, const std::vector<std::string>& categories,
                             std::vector<std::string>& all_classes);

    void instantiate_items(
        size_t total, const pt::ptree::value_type& item, bool print = false);

    // For a small chance we put all the items in a row
    // This will only change the locations of the items
    void put_items_in_a_row(double prob, ItemType t);
};

}} // namespace simulator::xwd
