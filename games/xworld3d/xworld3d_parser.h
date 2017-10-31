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
#include <map>
#include <memory>
#include <queue>

#include "x3item.h"

namespace simulator {
namespace xworld3d {

namespace pt = boost::property_tree;

struct CurriculumInfo {
    CurriculumInfo() : current_level(0),
                       curriculum_learning(0),
                       num_games_so_far(0),
                       success_rate_at_current_level(0) {}

    template<typename T>
    T select_number_from_range(T lo, T hi) {
        if (num_games_so_far >= curriculum_learning) {
            return hi;
        }

        return (double(num_games_so_far)/curriculum_learning * (hi-lo) + lo);
    }

    int current_level;
    int curriculum_learning;
    int num_games_so_far;
    float success_rate_at_current_level;
};

class X3Parser {
public:
    X3Parser(const std::string& world_config,
             const std::string& model_dir,
             bool print_xworld_config,
             int curriculum_learning = 0);

    void reset_config(bool print = false) { reset_config(world_conf_, print); }

    // this enables the teacher to change the env during training
    void reset_config(const std::string& world_config, bool print = false);

    void generate_map(std::vector<X3ItemInfo>& item_list);

    int height() { return height_; }
    int width() { return width_; }

    std::string get_model_file(std::string name) {
        if (item_repo_.find(name) != item_repo_.end()) {
            return item_repo_[name];
        } else {
            return std::string("");
        }
    }

    std::string assign_agent();

private:
    CurriculumInfo curriculum_;
    std::vector<X3ItemInfo> item_list_;
    int height_;
    int width_;
    std::string model_root_dir_;
    std::string world_conf_;
    std::unique_ptr<pt::ptree> conf_root_;
    std::map<std::string, X3EntityType> item_types_;
    std::map<std::string, std::string> item_repo_;
    std::deque<X3ItemInfo> agent_list_;
    int total_items_;

    void build_world_boundaries(std::vector<X3ItemInfo>& list);

    int get_total_num_of_items(const pt::ptree::value_type& item);

    void instantiate_items(size_t total,
                           const pt::ptree::value_type& item);

    std::vector<int> read_json_vector(const pt::ptree::value_type& node);
};

}} // namespace simulator::xworld3d