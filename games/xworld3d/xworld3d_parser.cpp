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

#include <boost/exception/all.hpp>
#include <unordered_set>
#include "xworld3d_parser.h"

namespace simulator {
namespace xworld3d {

using simulator::util::path_join;

X3Parser::X3Parser(const std::string& world_config,
                   const std::string& model_dir,
                   bool print_xworld_config,
                   int curriculum_learning) :
        curriculum_(),
        model_root_dir_(model_dir),
        world_conf_(world_config),
        conf_root_(nullptr) {
    curriculum_.curriculum_learning = curriculum_learning;
    item_types_["block"] = X3EntityType::BLOCK;
    item_types_["agent"] = X3EntityType::AGENT;
    item_types_["goal"] = X3EntityType::GOAL;

    // TODO: use a class to pre-load these
    item_repo_["ground"] = path_join({model_root_dir_, "floor.xml"});
    item_repo_["stadium"] = path_join({model_root_dir_, "stadium/stadium.obj"});
    item_repo_["block"] = path_join({model_root_dir_, "block/block.urdf"});
    item_repo_["agent"] = path_join({model_root_dir_, "ball/ball.urdf"});
    item_repo_["goal"] = path_join({model_root_dir_, "apple/apple.urdf"});
}

void X3Parser::reset_config(const std::string& world_config, bool print) {
    curriculum_.num_games_so_far++;
    world_conf_ = world_config;
    item_list_.clear();
    agent_list_.clear();
    total_items_ = 0;

    // only read the config once
    if (!conf_root_) {
        // print the config file
        if (print) {
            std::ifstream infile(world_config);
            std::string line;
            while (std::getline(infile, line)) {
                LOG(INFO) << line;
            }
        }

        conf_root_.reset(new pt::ptree);
        try {
            pt::read_json(world_config, *(conf_root_.get()));
        } catch (const boost::exception& ex) {
            LOG(FATAL) << "world config file error: "
                       << boost::diagnostic_information(ex);
        }
    }

    CHECK_GT(conf_root_->count("xworld3d"), 0);
    // get_child is overloadded with different return types
    pt::ptree xwd3 = conf_root_->get_child("xworld3d");

    height_ = xwd3.get<int>("dimensions.height");
    width_ = xwd3.get<int>("dimensions.width");

    for (const auto& t : xwd3.get_child("items")) {
        int total = get_total_num_of_items(t);
        if (item_types_.find(t.first) == item_types_.end()) {
            LOG(FATAL) << "JSON file contains unsupported item type: "
                       << t.first;
        }
        instantiate_items(total, t);
    }
    CHECK_GT(item_list_.size(), 0);
    CHECK_GT(agent_list_.size(), 0);
}

void X3Parser::build_world_boundaries(std::vector<X3ItemInfo>& list) {
    for (int i = -1; i <= width_; ++i) {
        list.emplace_back(
            "block" + std::to_string(total_items_++),
            X3EntityType::BLOCK,
            item_repo_["block"],
            Vec3(i, -1.0f, 0.0f)
        );
        list.emplace_back(
            "block" + std::to_string(total_items_++),
            X3EntityType::BLOCK,
            item_repo_["block"],
            Vec3(i, height_, 0.0f)
        );
    }
    for (int i = 0; i < height_; ++i) {
        list.emplace_back(
            "block" + std::to_string(total_items_++),
            X3EntityType::BLOCK,
            item_repo_["block"],
            Vec3(-1.0f, i, 0.0f)
        );
        list.emplace_back(
            "block" + std::to_string(total_items_++),
            X3EntityType::BLOCK,
            item_repo_["block"],
            Vec3(width_, i, 0.0f)
        );
    }
}

void X3Parser::generate_map(std::vector<X3ItemInfo>& list) {
    reset_config();
    list = item_list_;

    std::unordered_set<Vec3> occupied;
    for (auto const& it : list) {
        if (it.loc.defined()) {
            occupied.insert(it.loc);
        }
    }
    for (auto& it : list) {
        if (!it.loc.defined()) {
            Vec3 l;
            l.random_loc(width_, height_);
            while (occupied.find(l) != occupied.end()) {
                l.random_loc(width_, height_);
            }
            it.loc = l;
            occupied.insert(l);
        }
    }

    build_world_boundaries(list);
}

std::string X3Parser::assign_agent() {
    CHECK_GT(agent_list_.size(), 0);
    std::string agent_name = agent_list_.front().name;
    agent_list_.pop_front();
    return agent_name;
}

int X3Parser::get_total_num_of_items(const pt::ptree::value_type& item) {
    // randomly select the value for total in the number range
    std::vector<int> num_range;
    for (const auto& n : item.second.get_child("number")) {
        num_range.push_back(n.second.get_value<int>());
    }
    CHECK_LE(num_range.size(), 2);
    if (num_range.size() == 1) {
        num_range.push_back(num_range[0]);
    }
    return curriculum_.select_number_from_range(num_range[0], num_range[1]);
}

void X3Parser::instantiate_items(size_t total,
                                 const pt::ptree::value_type& item) {
    if (total == 0) { return; }

    size_t cnt = 0;
    if (item.second.count("instances") > 0) {
        for (const auto& node : item.second.get_child("instances")) {
            if (cnt == total) {
                break;
            }

            auto v = read_json_vector(node);
            Vec3 loc(v[0], v[1], v[2]);
            if (!loc.in_boundary(width_, height_)) {
                continue;
            }

            X3ItemInfo i(item.first + std::to_string(total_items_),
                         item_types_[item.first],
                         item_repo_[item.first],
                         loc);
            item_list_.push_back(i);
            if (i.type == X3EntityType::AGENT) {
                agent_list_.push_back(i);
            }
            total_items_++;
            cnt++;
        }
    }

    ///////////////// then initialize some random items //////////////////
    for (size_t k = 0; k < total - cnt; k++) {
        X3ItemInfo i(item.first + std::to_string(total_items_),
                     item_types_[item.first],
                     item_repo_[item.first]);
        item_list_.push_back(i);
        if (i.type == X3EntityType::AGENT) {
            agent_list_.push_back(i);
        }
        total_items_++;
    }
}

std::vector<int> X3Parser::read_json_vector(
    const pt::ptree::value_type& node) {
    std::vector<int> vec;
    for (const auto& x : node.second) {
        vec.push_back(x.second.get_value<int>());
    }
    return vec;
}

}} // simulator::xworld3d
