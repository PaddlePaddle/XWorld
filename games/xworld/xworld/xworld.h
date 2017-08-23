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
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <sstream>
#include <unordered_set>
#include <memory>

#include "xitem.h"
#include "xmap.h"
#include "xagent.h"
#include "xworld_parser.h"

namespace simulator { namespace xwd {
/*enum ItemType  -> Defined in XItem.h
{
    GOAL,
    AGENT,
    BLOCK,
    DUMMY
};

typedef struct {
    ItemType type;       // item type
    std::string name;    // item name (unique id)
    std::string img_path;// item image path on the disk
    Loc location;        // item location
    std::string color;   // item color:
    bool active;          // this int stores the active information for an item
} ItemInfo;

typedef std::vector<ItemInfo> WorldUnitList;
*/

/**
*  class that implements the environment for agent learning
*/

class XWorld {
public:
    XWorld(bool print_xworld_config, const std::string& conf, int curriculum_learning);

    void reset();

    //destructor
    virtual ~XWorld();

    // get an image representation of the world
    // flag_item_centric=true: get an egocentric view image representation of the world for the specified item
    // flag_active_goal=true: generate an image with the specified goal highlighted
    cv::Mat to_image(bool flag_item_centric = false, std::string item_name = "",
                     int pad_size = 0, bool flag_illustration = true, int success = 0,
                     int visible_radius_unit = 0, bool flag_crop_receiptive_field = false);

    // add an agent to the world and get a pointer to the agent
    XAgent* add_agent(std::string agent_name);

    // agent specified by agent_ptr takes action action_id
    void act(XAgent* agent_ptr, int action_id);

    // agent specified by agent_ptr take random action
    void act_random(XAgent* agent_ptr, std::vector<int> action_list);

    Loc get_agent_location(int agent_id) {
        return agent_list_[agent_id]->get_item_location();
    }

    std::vector<Loc> get_goal_locations();

    // get all items in the world with fields including type, name, location
    void get_all_items(std::vector<Entity>& entities);

    void get_all_possible_objects(std::vector<Entity>& objects);

    std::vector<std::string> get_target_goal_names(std::vector<int> goal_inds);

    unsigned int get_number_of_goals();

    //get number of actions
    int get_num_actions();

    int height() { return height_; }

    int width() { return width_; }

    // set the max steps for each agent, called by xworld_simulator
    // this variable will be recorded for each agent, so that when an agent
    // reaches its max steps, it may get a TIMEOUT_REWARD
    // see agent_remaining_steps_
    void set_max_steps(int max_steps) { max_steps_ = max_steps; }

private:

    XMap map_;
    int height_;
    int width_;
    int max_steps_;                           // the maximum steps each agent has
    XWorldParser xwp_;
    std::vector<Loc> agent_units_;            // keep the remaining of the reachable slots after the map
                                              // is generated; the agents will be placed on these slots

    bool success_action_flag_;                // whether the action is successfully accomplished

    std::vector<XItem*> item_list_;           // list of all items, including agents
    std::vector<XAgent*> agent_list_;         // list of all agents,
    std::vector<int> agent_remaining_steps_;  // how many steps remained for each agent

    unsigned int total_goal_num_;

    void init();

    ItemInfo get_next_agent();

    // get all the grids on the map that are not in exclude
    void sample_excluded(const std::unordered_set<Loc>& exclude,
                         std::vector<Loc>& sampled_grids);

    // generate a valid map so that all the reachable grids
    // can be reached by all the agents
    void randomize_valid_navigation_conf(
        int blocks_n, int reachable_n, int agents_n,
        const std::unordered_set<Loc>& user_defined_block_locs,
        const std::unordered_set<Loc>& user_defined_reachable_locs,
        const std::unordered_set<Loc>& user_defined_agent_locs,
        std::vector<Loc>& blocks_candidates,
        std::vector<Loc>& reachable_candidates,
        std::vector<Loc>& agent_candidates);

    // test if a map is valid
    bool test_valid_map(const std::vector<Loc>& blocks_candidates,
                        const std::vector<Loc>& reachable_candidates,
                        const std::vector<Loc>& agent_candidates);

    // Given a starting location s, a target label r, and a set of obstacles
    // impassable, determine if r is reachable from s
    // A location might have several labels
    // When reaching r, other labels r' cannot be stepped on
    bool check_reachable(size_t visited, const Loc& s, size_t r,
                         std::unordered_map<Loc, size_t>& impassable);

    void add_item_to_world(ItemType type, const std::string& name,
                           const std::string& img_path, Loc loc, bool active);

    static constexpr float TIMEOUT_REWARD = -1.0f;        // (negative reward) for timeout
    static constexpr float MOVE_REWARD = -0.1f;           // (negative) reward for a single action
    static constexpr float DEST_REWARD = 1.0f;            // reward when complete the task
    static constexpr float FAIL_REWARD = -0.2f;           // (negative) reward for a failure action
    static constexpr float WRONG_GOAL_REWARD = -1.0f;     // failed action
    static constexpr float SPEAK_CORRECT_REWARD = 0.5f;   // reward for correct sentence response
};

}} // namespace simulator::xwd
