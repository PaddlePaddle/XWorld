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

#include <functional>
#include "xagent.h"

namespace simulator { namespace xwd {

XAgent::XAgent(std::string name, Loc loc, const XMap* map, std::string img_name)
        : XItem(AGENT, name, loc, img_name, true), map_(map) {
    this->register_actions();
}

std::string XAgent::speak(const std::string& command) {
    return command;
}

std::string XAgent::listen(const std::string& command) {
    return command;
}

void XAgent::update_location(int x, int y) {
    this->set_item_location(x, y);
}

unsigned int XAgent::get_num_of_actions() {
    return this->action_callbacks_.size();
}

void XAgent::register_actions() {
    // all actions better defined as lambda functions
    action_callbacks_.push_back([this] { return move_up(); });
    action_callbacks_.push_back([this] { return move_down(); });
    action_callbacks_.push_back([this] { return move_left(); });
    action_callbacks_.push_back([this] { return move_right(); });

	//add more self-defined action here
//    action_callbacks_.push_back(std::bind(&XAgent::move_up_two_steps, this));
//    action_callbacks_.push_back(std::bind(&XAgent::move_down_two_steps, this));
//    action_callbacks_.push_back(std::bind(&XAgent::move_left_two_steps, this));
//    action_callbacks_.push_back(std::bind(&XAgent::move_right_two_steps, this));
}


bool XAgent::move_up() {
    bool success_action_flag = false;
    Loc curr_loc = this->get_item_location();
    if (this->map_->is_reachable(curr_loc.x, curr_loc.y-1)) {
    	this->update_location(curr_loc.x, curr_loc.y-1);
        success_action_flag = true;
    }
    return success_action_flag;
}

bool XAgent::move_down() {
    bool success_action_flag = false;
    Loc curr_loc = this->get_item_location();
    if (this->map_->is_reachable(curr_loc.x, curr_loc.y+1)) {
        this->update_location(curr_loc.x, curr_loc.y+1);
        success_action_flag = true;
    }
    return success_action_flag;
}

bool XAgent::move_left() {
    bool success_action_flag = false;
    Loc curr_loc = this->get_item_location();
    if (this->map_->is_reachable(curr_loc.x-1, curr_loc.y)) {
        this->update_location(curr_loc.x-1, curr_loc.y);
        success_action_flag = true;
    }
    return success_action_flag;
}

bool XAgent::move_right() {
    bool success_action_flag = false;
    Loc curr_loc = this->get_item_location();
    if (this->map_->is_reachable(curr_loc.x+1, curr_loc.y)) {
        this->update_location(curr_loc.x+1, curr_loc.y);
        success_action_flag = true;
    }
    return success_action_flag;
}

bool XAgent::move_up_two_steps() {
    return move_up() && move_up();
}

bool XAgent::move_down_two_steps() {
    return move_down() && move_down();
}

bool XAgent::move_left_two_steps() {
    return move_left() && move_left();
}

bool XAgent::move_right_two_steps() {
    return move_right() && move_right();
}

bool XAgent::act(int action_id) {
    return action_callbacks_[action_id]();
}

}} //namespace simulator::xwd
