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

#include "xworld_parser.h"
#include <algorithm>
#include <fstream>
#include <boost/algorithm/string.hpp>
#include <boost/exception/all.hpp>
#include <unordered_set>

namespace simulator { namespace xwd {

// called only ONCE across all the games
XWorldParser::XWorldParser(bool print_xworld_config,
                           const std::string& world_config,
                           int curriculum_learning)
        : curriculum_learning_(curriculum_learning), num_games_so_far_(0),
          world_conf_(world_config), conf_root_(nullptr) {
    item_types_["goal"] = GOAL;
    item_types_["agent"] = AGENT;
    item_types_["block"] = BLOCK;
    item_types_["dummy"] = DUMMY;
    reset_config(print_xworld_config);
}

std::vector<std::vector<int>> XWorldParser::read_json_matrix(const pt::ptree::value_type& node) {
    std::vector<std::vector<int>> matrix;
    for (const auto& r : node.second) {
        matrix.push_back(read_json_vector(r));
    }
    return matrix;
}

std::vector<int> XWorldParser::read_json_vector(const pt::ptree::value_type& node) {
    std::vector<int> vec;
    for (const auto& x : node.second) {
        vec.push_back(x.second.get_value<int>());
    }
    return vec;
}

int XWorldParser::get_total_num_of_items(const pt::ptree::value_type& item) {
    // randomly select the value for total in the number range
    std::vector<int> num_range;
    for (const auto& n : item.second.get_child("number")) {
        num_range.push_back(n.second.get_value<int>());
    }
    CHECK_LE(num_range.size(), 2);
    if (num_range.size() == 1) {
        num_range.push_back(num_range[0]);
    }
    return select_number_from_range(num_range[0], num_range[1],
                                    curriculum_learning_);
}

int XWorldParser::select_number_from_range(int low, int high, bool curriculum_learning) {
    CHECK_GE(high, low);
    int ret = low;
    if (curriculum_learning) {
        ret = curriculum_number(low, high, num_games_so_far_, curriculum_learning_);
    } else {
        ret = util::get_rand_ind(high - low + 1) + low; // [low, high]
    }
    return ret;
}

void XWorldParser::set_map_level() {
    std::vector<std::string> classes;
    select_item_classes("block", std::vector<std::string>(), classes);

    auto get_block_unit = [&] (int x, int y) {
        ItemInfo i;
        i.type = item_types_["block"];
        i.img_path = XItem::item_rep_.item_tree_->retrieve_item_path(classes[0]);
        i.name = "brick_" + std::to_string(total_items_++);
        i.location = Loc(x, y);
        return i;
    };

    std::unordered_set<Loc> visited;

    auto put_block_unit = [&] (int x, int y) {
        // corners will be duplicate
        if (visited.find(Loc{x,y}) == visited.end()) {
            ItemInfo i = get_block_unit(x, y);
            ul_.push_back(i);
            ul_map_[i.name] = i;
            visited.insert(Loc{x,y});
        }
    };

    int level = select_number_from_range(1, levels_, curriculum_learning_);
    int wall_thickness = levels_ - level;
    int x = 0;
    int y = 0;
    int current_size = height_;
    if (wall_thickness % 2) {
        // randomly select a corner for the extra wall padding
        int location = util::get_rand_ind(4);
        int wx = (location % 2) * (current_size - 1);
        int wy = (location / 2) * (current_size - 1);
        // put two walls starting (wx, wy)
        for (int k = 0; k < current_size; k ++) {
            std::vector<int> wxs = {wx, k};
            std::vector<int> wys = {k, wy};
            for (int j = 0; j < 2; j ++) {
                put_block_unit(wxs[j], wys[j]);
            }
        }
        if (wx == 0) {
            x ++;
        }
        if (wy == 0) {
            y ++;
        }
        current_size --;
    }

    wall_thickness /= 2;
    for (int i = 0; i < wall_thickness; i ++) {
        std::vector<int> wxs = {x, x + current_size - 1, x + current_size - 1, x};
        std::vector<int> wys = {y, y, y + current_size - 1, y + current_size - 1};
        std::vector<int> dx = {1, 0, -1, 0};
        std::vector<int> dy = {0, 1, 0, -1};
        for (int j = 0; j < 4; j ++) {
            for (int k = 0; k < current_size; k ++) {
                put_block_unit(wxs[j] + dx[j] * k, wys[j] + dy[j] * k);
            }
        }
        x ++;
        y ++;
        current_size -= 2;
        CHECK_GE(current_size, 2) << "too many map levels";
    }
}

int XWorldParser::curriculum_number(int low, int high, int num_games, int curriculum_games) {
    CHECK_GT(curriculum_games, 0);
    // gradually increase the difficulty
    // we assume the bigger the number, the more difficulty there is (goals, blocks)
    double progress = std::min(double(num_games) / curriculum_games, 1 - 1e-5);
    return low + int((high - low + 1) * progress);
}

void XWorldParser::get_all_possible_objects(std::vector<Entity>& objects) {
    CHECK_GT(all_objects_.size(), 0);
    objects.clear();
    for (const auto& o : all_objects_) {
        Entity e;
        e.set_property("name", o.name);  // (no id)
        e.set_property("color", o.color);
        objects.push_back(e);
    }
}

std::string XWorldParser::sample_schedule(const pt::ptree::value_type& item) {
    std::vector<std::pair<std::string,double>> schedules;
    double total_prob = 0;
    for (const auto& node : item.second.get_child("schedule")) {
        double p = node.second.get_value<double>();
        schedules.push_back(std::make_pair(node.first, p));
        total_prob += p;
    }

    // sample a schedule according to the weights
    for (size_t i = 1; i < schedules.size(); i ++) {
        schedules[i].second += schedules[i-1].second;
    }
    double sample = util::get_rand_range_val(total_prob);
    std::string schedule = "";
    // assume that we have only a few schedules
    // so we use linear scan
    for (const auto& s : schedules) {
        if (s.second > sample) {
            schedule = s.first;
            break;
        }
    }
    CHECK_NE(schedule, "");
    return schedule;
}

ItemInfo XWorldParser::get_next_agent() {
    CHECK(agent_que_idx_ < agent_que_.size()) << "Agent number exceeds!";
    return agent_que_[agent_que_idx_ ++];
}

// class: "goal/fruit/apple"
void XWorldParser::select_item_classes(std::string type,
                                       const std::vector<std::string>& categories,
                                       std::vector<std::string>& all_classes) {
    // get all the icon classes of the type
    std::string dir = type + "/";
    if (categories.empty()) {
        XItem::item_rep_.item_tree_->retrieve_item_classes(dir, all_classes);
    } else {
        std::vector<std::string> tmp;
        for (const auto& c : categories) {
            XItem::item_rep_.item_tree_->retrieve_item_classes(
                dir + c + "/", tmp);
            all_classes.insert(all_classes.end(), tmp.begin(), tmp.end());
        }
    }
    std::sort(all_classes.begin(), all_classes.end());
    CHECK_GT(all_classes.size(), 0);
}

void XWorldParser::record_all_possible_objects(const std::vector<std::string>& all_classes) {
    all_objects_.clear();
    for (const auto& c : all_classes) {
        std::string class_name = c.substr(c.find_last_of("/") + 1);
        std::vector<std::string> item_paths = XItem::item_rep_.item_tree_->retrieve_item_paths(c);
        for (const auto& p : item_paths) {
            std::string color = XItem::item_rep_.retrieve_property(p, "color");
            ItemInfo item;
            item.color = color;
            item.name = class_name;  // (no id)
            all_objects_.push_back(item);
        }
    }
}

void XWorldParser::instantiate_items(
    size_t total, const pt::ptree::value_type& item, bool print) {
    if (total == 0) {
        return;
    }
    if (item_types_.find(item.first) == item_types_.end())
        LOG(FATAL) << "JSON file contains unsupported item type: " << item.first;

    ///////////////// get all the icon classes of the type ////////////////
    std::vector<std::string> categories;
    if (item.second.count("category") > 0) {
        for (const auto& c : item.second.get_child("category")) {
            categories.push_back(c.second.get_value<std::string>());
        }
    }
    std::vector<std::string> all_classes;
    select_item_classes(item.first, categories, all_classes);

    ///////////////// record all possible goals ///////////////////
    //// only do this for the first game
    if (item.first == "goal" && num_games_so_far_ == 1) {
        record_all_possible_objects(all_classes);
    }

    /////////// gradually increase the number of items on the map //////////
    //////// sorting makes the order of the items the same for the next game /////////
    int size = all_classes.size();
    if (//false &&                  // random class number
        curriculum_learning_) {     // increasing class number
        size = curriculum_number(std::max(1, int(all_classes.size()/10)),
                                 all_classes.size(),
                                 num_games_so_far_, curriculum_learning_);
    }
    std::vector<std::string> classes(all_classes.begin(), all_classes.begin() + size);
    util::random_shuffle(classes);

    auto get_class_name = [] (const std::string& full_class) {
        int idx = full_class.find_last_of("/") + 1;
        return full_class.substr(idx);
    };

    auto classes_match = [get_class_name] (const std::string& full_class,
                             const std::string& class_name) {
        return get_class_name(full_class) == class_name;
    };

    size_t cnt = 0;
    if (item.second.count("instances") > 0) {
        for (const auto& node : item.second.get_child("instances")) {
            if (cnt == total) {
                break;
            }
            ItemInfo i;
            i.type = item_types_[item.first];

            // "x" represents a randomly sampled novel class with specified fixed location
            if (node.first == "x") {
                CHECK_GT(classes.size(), 0);
                size_t idx = util::get_rand_ind(classes.size());
                std::string class_name = classes[idx];
                classes.erase(classes.begin() + idx); // ensure unique objects are sampled
                i.img_path = XItem::item_rep_.item_tree_->retrieve_item_path(class_name);
                i.name = get_class_name(class_name) + "_" + std::to_string(total_items_);
            } else {
                std::string class_name = "";
                for (auto n : classes) {
                    if (classes_match(n, node.first)) {
                        class_name = n;
                        break;
                    }
                }
                CHECK(!class_name.empty());
                i.img_path = XItem::item_rep_.item_tree_->retrieve_item_path(class_name);
                i.name = node.first + "_" + std::to_string(total_items_);
            }
            std::vector<int> loc = read_json_vector(node);
            i.location = Loc(loc);
            if (item.first == "agent") {
                agent_que_.push_back(i);
            }
            ul_.push_back(i);
            ul_map_[i.name] = i;
            total_items_++;
            cnt ++;
        }
    }

    ///////////////// then initialize some random items //////////////////
    for (size_t k = 0; k < total - cnt; k ++) {
        ItemInfo i;
        i.type = item_types_[item.first];
        int idx = util::get_rand_ind(classes.size());
        // automatically name the item by its type and id
        std::string c = classes[idx];
        std::string class_name = c.substr(c.find_last_of("/") + 1);
        i.img_path = XItem::item_rep_.item_tree_->retrieve_item_path(c);
        i.name = class_name + "_" + std::to_string(total_items_);
        i.location = Loc();  // undefined location
        if (item.first == "agent") {
            agent_que_.push_back(i);
        }
        ul_.push_back(i);
        ul_map_[i.name] = i;
        total_items_++;
    }
}

void XWorldParser::put_items_in_a_row(double prob, ItemType t) {
    Loc anchor;
    if (util::get_rand_range_val(1) < prob) {
        std::unordered_set<Loc> defined;
        for (const auto& u : ul_) {
            if (u.location.defined()) {
                defined.insert(u.location);
            }
        }
        for (auto& u : ul_) {
            if (u.type == t) {
                if (!anchor.defined()) {
                    anchor.random_loc(width_, height_);
                    u.location = anchor;
                } else {
                    anchor = anchor + Loc(1, 0);
                    if (anchor.in_boundary(width_, height_)) {
                        u.location = anchor;
                    }
                }
                // already occupied
                if (defined.count(u.location) > 0) {
                    u.location = Loc();
                }
            }
        }
    }
}

void XWorldParser::reset_config(const std::string& world_config, bool print) {
    num_games_so_far_ ++;
    world_conf_ = world_config;
    total_items_ = 0;
    agent_que_idx_ = 0;
    ul_.clear();
    ul_map_.clear();
    agent_que_.clear();

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

    CHECK_GT(conf_root_->count("xworld"), 0);
    // get_child is overloadded with different return types
    pt::ptree xwd = conf_root_->get_child("xworld");

    height_ = xwd.get<int>("dimensions.height");
    width_ = xwd.get<int>("dimensions.width");
    CHECK_EQ(height_, width_);

    levels_ = 1;

    // levels only matter when curriculum learning
    if (xwd.count("levels") > 0 && curriculum_learning_) {
        levels_ = xwd.get<int>("levels");
        CHECK_GE(levels_, 1) << "level number incorrect";
    }

    // first surround the map with blocks according to levels_
    // so that later the generator can adjust the unit locations
    set_map_level();

    for (const auto& t : xwd.get_child("items")) {
        size_t total = get_total_num_of_items(t);
        instantiate_items(total, t, print);
        if (t.second.count("in_a_row_prob") > 0) {
            put_items_in_a_row(t.second.get<double>("in_a_row_prob"),
                               item_types_[t.first]);
        }
    }
    CHECK_GT(ul_map_.size(), 0);
}

}} // namespace simulator::xwd
