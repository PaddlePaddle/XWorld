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
