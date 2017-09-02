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

#include "xworld.h"
#include <gflags/gflags.h>
#include <algorithm>
#include <boost/algorithm/string.hpp>
#include <limits>
#include <utility>

DECLARE_bool(perturb_goal);

namespace simulator {
namespace xwd {

XWorld::~XWorld() {
    for (auto i : item_list_) {
        delete i;
    }
}

std::vector<std::string> XWorld::get_target_goal_names(
    std::vector<int> goal_inds) {
    int item_num = item_list_.size();
    int goal_cnt = 0;
    size_t cnt = 0;
    std::vector<std::string> goal_names;
    std::sort(goal_inds.begin(), goal_inds.end());
    for (int i = 0; i < item_num && cnt < goal_inds.size(); i++) {
        if (item_list_[i]->get_item_type() == GOAL) {
            if (goal_cnt == goal_inds[cnt]) {  // found the specified goal
                goal_names.push_back(item_list_[i]->get_item_name());
                cnt++;
            }
            goal_cnt++;
        }
    }
    return goal_names;
}

void XWorld::get_all_items(std::vector<Entity>& entities) {
    auto convert_item_type = [](ItemType t) {
        if (t == GOAL) {
            return "goal";
        } else if (t == AGENT) {
            return "agent";
        } else if (t == BLOCK) {
            return "block";
        } else if (t == DUMMY) {
            return "dummy";
        } else {
            LOG(FATAL) << "item type error";
        }
    };

    entities.clear();
    for (unsigned int i = 0; i < item_list_.size(); i++) {
        Entity e;
        auto loc = item_list_[i]->get_item_location();
        e.location = Vec3(loc.x, loc.y, 0);
        e.type = convert_item_type(item_list_[i]->get_item_type());
        e.id = item_list_[i]->get_item_name();
        e.set_property("name", util::remove_instance_id(e.id));
        e.set_property("color", item_list_[i]->get_item_color());
        entities.push_back(e);
    }
}

void XWorld::get_all_possible_objects(std::vector<Entity>& objects) {
    xwp_.get_all_possible_objects(objects);
}

unsigned int XWorld::get_number_of_goals() { return total_goal_num_; }

std::vector<Loc> XWorld::get_goal_locations() {
    int item_num = item_list_.size();
    std::vector<Loc> loc_set;
    Loc loc;
    for (int i = 0; i < item_num; i++) {
        if (item_list_[i]->get_item_type() == GOAL) {
            loc = item_list_[i]->get_item_location();
            loc_set.push_back(loc);
        }
    }
    return loc_set;
}

int XWorld::get_num_actions() {
    if (agent_list_.size() > 0) {
        return agent_list_[0]->get_num_of_actions();
    } else {
        XAgent* agent_ptr = new XAgent("", Loc(0, 0), &(map_), "");
        return agent_ptr->get_num_of_actions();
    }
}

XAgent* XWorld::add_agent(std::string agent_name) {
    ItemInfo a = get_next_agent();
    // the agent get a snap shot of the map with all the existing items
    XAgent* agent_ptr = new XAgent(agent_name, a.location, &(map_), a.img_path);
    agent_list_.push_back(agent_ptr);
    agent_remaining_steps_.push_back(max_steps_);
    item_list_.push_back(agent_ptr);
    map_.add_item(item_list_.back());
    return agent_ptr;
}

ItemInfo XWorld::get_next_agent() {
    ItemInfo a = xwp_.get_next_agent();
    if (!a.location.defined()) {
        a.location = agent_units_[util::get_rand_ind(agent_units_.size())];
    }
    // agents can start with the same location, so no need to update
    // agent_units_
    return a;
}

void XWorld::reset() {
    // keep the agent list
    for (auto i : item_list_) {
        if (i->get_item_type() != AGENT) {
            delete i;
        }
    }
    std::fill(agent_remaining_steps_.begin(),
              agent_remaining_steps_.end(),
              max_steps_);

    init();

    CHECK_GT(agent_list_.size(), 0);
    for (auto a : agent_list_) {
        auto i = get_next_agent();
        a->set_item_location(i.location.x, i.location.y);
        item_list_.push_back(a);
        map_.add_item(a);
    }
}

XWorld::XWorld(bool print_xworld_config,
               const std::string& conf,
               int curriculum_learning)
    : max_steps_(0),
      xwp_(print_xworld_config, conf, curriculum_learning) {
    init();
}

bool XWorld::test_valid_map(const std::vector<Loc>& blocks_candidates,
                            const std::vector<Loc>& reachable_candidates,
                            const std::vector<Loc>& agent_candidates) {
    std::unordered_map<Loc, size_t> impassable;
    for (const auto& b : blocks_candidates) {
        impassable[b] = 1 << 0;
    }
    for (size_t r = 1; r <= reachable_candidates.size(); r++) {
        const auto& l = reachable_candidates[r - 1];
        if (impassable.find(l) == impassable.end()) {
            impassable[l] = 1 << r;
        } else {
            impassable[l] += 1 << r;
        }
    }
    for (size_t a = 0; a < agent_candidates.size(); a++) {
        const auto& l = agent_candidates[a];
        // dfs to check reachable through *only* ground
        for (size_t r = 1; r <= reachable_candidates.size(); r++) {
            auto tmp = impassable;
            if (!check_reachable(
                    1 << (reachable_candidates.size() + 1), l, 1 << r, tmp)) {
                return false;
            }
        }
    }
    return true;
}

void XWorld::sample_excluded(const std::unordered_set<Loc>& exclude,
                             std::vector<Loc>& sampled_grids) {
    sampled_grids.clear();
    for (int w = 0; w < width_; w++) {
        for (int h = 0; h < height_; h++) {
            if (exclude.find(Loc(w, h)) == exclude.end()) {
                sampled_grids.push_back(Loc(w, h));
            }
        }
    }
    util::random_shuffle(sampled_grids);
}

void XWorld::randomize_valid_navigation_conf(
    int blocks_n,
    int reachable_n,
    int agents_n,
    const std::unordered_set<Loc>& user_defined_block_locs,
    const std::unordered_set<Loc>& user_defined_reachable_locs,
    const std::unordered_set<Loc>& user_defined_agent_locs,
    std::vector<Loc>& blocks_candidates,
    std::vector<Loc>& reachable_candidates,
    std::vector<Loc>& agent_candidates) {
    // SIMULATOR_TIMER(randomize_valid_xworld_map);

    CHECK_LT(reachable_n, 32U) << "this function is no longer efficient";

    auto assign_grids = [](std::vector<Loc>& candidates,
                           int& idx,
                           int remaining,
                           const std::unordered_set<Loc>& user_defined_locs,
                           const std::vector<Loc>& sampled_grids) {
        CHECK_LE(idx + remaining, sampled_grids.size());
        candidates = std::vector<Loc>(user_defined_locs.begin(),
                                      user_defined_locs.end());
        for (int i = 0; i < remaining; i++) {
            candidates.push_back(sampled_grids[idx++]);
        }
    };

    size_t total = blocks_n - user_defined_block_locs.size() + reachable_n -
                   user_defined_reachable_locs.size() + agents_n -
                   user_defined_agent_locs.size();
    std::unordered_set<Loc> exclude;
    exclude.insert(user_defined_block_locs.begin(),
                   user_defined_block_locs.end());
    exclude.insert(user_defined_reachable_locs.begin(),
                   user_defined_reachable_locs.end());
    exclude.insert(user_defined_agent_locs.begin(),
                   user_defined_agent_locs.end());

    const int maximal_trials = 1000;
    int trials = 0;
    while (true) {
        if (trials++ > maximal_trials) {
            LOG(FATAL) << "No valid map is found. "
                       << "Check your configuration file. (too many blocks?)";
        }
        std::vector<Loc> sampled_grids;
        sample_excluded(exclude, sampled_grids);
        CHECK_GE(sampled_grids.size(), total);
        int idx = 0;
        // assign sampled_grids to blocks, reachable, and agents
        assign_grids(blocks_candidates,
                     idx,
                     blocks_n - user_defined_block_locs.size(),
                     user_defined_block_locs,
                     sampled_grids);
        assign_grids(reachable_candidates,
                     idx,
                     reachable_n - user_defined_reachable_locs.size(),
                     user_defined_reachable_locs,
                     sampled_grids);
        assign_grids(agent_candidates,
                     idx,
                     agents_n - user_defined_agent_locs.size(),
                     user_defined_agent_locs,
                     sampled_grids);
        // test valid
        if (test_valid_map(
                blocks_candidates, reachable_candidates, agent_candidates)) {
            break;
        }
    }
}

bool XWorld::check_reachable(size_t visited,
                             const Loc& s,
                             size_t r,
                             std::unordered_map<Loc, size_t>& impassable) {
    if (impassable.find(s) != impassable.end()) {
        bool ret = (impassable[s] == r);
        impassable[s] += visited;  // mark visited
        return ret;
    } else {                      // empty grid
        impassable[s] = visited;  // mark visited
        std::vector<Loc> around = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
        for (const auto& d : around) {
            Loc s_ = s + d;
            if (s_.in_boundary(width_, height_)) {
                if (impassable.find(s_) == impassable.end() ||
                    (impassable[s_] & visited) == 0) {
                    if (check_reachable(visited, s_, r, impassable)) {
                        return true;
                    }
                }
            }
        }
    }
    return false;
}

void XWorld::init() {
    item_list_.clear();
    WorldUnitList ul;
    xwp_.reset_config();
    xwp_.get_configuration(ul, height_, width_);

    map_ = XMap(height_, width_);

    std::unordered_set<Loc> user_defined_block_locs;
    std::unordered_set<Loc> user_defined_reachable_locs;
    std::unordered_set<Loc> user_defined_agent_locs;
    int n_blocks = 0;
    int n_reachable = 0;
    int n_agents = 0;
    for (const auto& u : ul) {
        if (u.type == BLOCK) {
            n_blocks++;
            if (u.location.defined()) {
                user_defined_block_locs.insert(u.location);
            }
        } else if (u.type == AGENT) {
            n_agents++;
            if (u.location.defined()) {
                user_defined_agent_locs.insert(u.location);
            }
        } else {  // anything except block and agent should be reachable
            n_reachable++;
            if (u.location.defined()) {
                user_defined_reachable_locs.insert(u.location);
            }
        }
    }
    CHECK_LE(n_blocks + n_reachable + n_agents, height_ * width_)
        << "n_blocks=" << n_blocks << " n_reachable=" << n_reachable
        << "n_agents=" << n_agents
        << ": too many units for the map; check your conf";

    std::vector<Loc> blocks_candidates;
    std::vector<Loc> reachable_candidates;
    std::vector<Loc> agent_candidates;
    randomize_valid_navigation_conf(n_blocks,
                                    n_reachable,
                                    n_agents,
                                    user_defined_block_locs,
                                    user_defined_reachable_locs,
                                    user_defined_agent_locs,
                                    blocks_candidates,
                                    reachable_candidates,
                                    agent_candidates);

    std::unordered_set<std::string> existing_names;
    int idx_reachable = 0;
    int idx_blocked = 0;
    int* idx = NULL;
    std::vector<Loc>* lst = nullptr;
    std::unordered_set<Loc>* defined_locs;
    for (const auto& u : ul) {
        // agents will be added later after the map is generated
        if (u.type == AGENT) {
            continue;
        }
        // make sure no duplicate names
        CHECK(existing_names.find(u.name) == existing_names.end());
        existing_names.insert(u.name);

        if (u.type == BLOCK) {
            idx = &idx_blocked;
            lst = &blocks_candidates;
            defined_locs = &user_defined_block_locs;
        } else {
            idx = &idx_reachable;
            lst = &reachable_candidates;
            defined_locs = &user_defined_reachable_locs;
        }

        Loc loc;
        // user predefined location
        if (u.location.defined()) {
            loc = u.location;
        } else {
            do {  // skip all user defined locations
                CHECK_LT(*idx, int(lst->size()));
                loc = (*lst)[*idx];
                *idx = *idx + 1;
            } while (defined_locs->count(loc) > 0);
        }
        add_item_to_world(u.type, u.name, u.img_path, loc, u.active);
    };

    total_goal_num_ = get_goal_locations().size();

    map_.add_items(item_list_);
    // keep for adding agents later
    // unlike goals, agents will be identical
    // so the mapping from which agent to which user-defined location
    // is not important
    agent_units_ = agent_candidates;
}

void XWorld::add_item_to_world(ItemType type,
                               const std::string& name,
                               const std::string& img_path,
                               Loc loc,
                               bool active) {
    switch (type) {
        case BLOCK:
        case DUMMY:
            item_list_.push_back(new XItem(type, name, loc, img_path, active));
            break;
        case GOAL:
            item_list_.push_back(new XItem(
                type, name, loc, img_path, active, FLAGS_perturb_goal));
            break;
        case AGENT:
            break;
        default:
            LOG(FATAL) << "unsupported item type for initialization";
    }
}

cv::Mat XWorld::to_image(bool flag_item_centric /* false */,
                         std::string item_name /* "" */,
                         int pad_size /* 0 */,
                         bool flag_illustration /* true */,
                         int success,
                         int visible_radius_unit /*0*/,
                         bool flag_crop_receiptive_field /*false*/) {
    std::vector<std::vector<std::string>> goal_names;
    cv::Mat img = map_.to_image(goal_names,
                                flag_item_centric,
                                item_name,
                                flag_illustration,
                                success,
                                visible_radius_unit,
                                flag_crop_receiptive_field);
    copyMakeBorder(img,
                   img,
                   pad_size,
                   pad_size,
                   pad_size,
                   pad_size,
                   cv::BORDER_CONSTANT,
                   cv::Scalar(0, 0, 0));
    return img;
}

bool XWorld::act(XAgent* agent_ptr, int action_id) {
    map_.remove_item(agent_ptr);
    auto success_action_flag = agent_ptr->act(action_id);
    map_.add_item(agent_ptr);
    return success_action_flag;
}

void XWorld::act_random(XAgent* agent_ptr, std::vector<int> action_list) {
    // sample a random action from the action list for the current agent
    agent_ptr->act(util::get_rand_ind(agent_ptr->get_num_of_actions()));
}
}
}  // namespace simulator::xwd
