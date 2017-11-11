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

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <map>
#include <memory>

#include "x3item.h"
#include "xworld3d_flags.h"

namespace simulator {
namespace xworld3d {

#define STEP_PENALTY -0.1f
#define COLLECT_GOAL 1.0f
#define WRONG_OP -0.2f

enum X3NavAction {
    MOVE_FORWARD = 0,
    TURN_LEFT = 1,
    TURN_RIGHT = 2,
    JUMP = 3,
    COLLECT = 4
};

enum X3Event {
    NOTHING = 0,
    CORRECT_GOAL = 1,
    WRONG_GOAL = 2,
    WRONG_ACTION = 3
};

class X3Scene {
public:
    X3Scene(float gravity, float time_step, int frame_skip, bool big_screen);

    void clear_scene() {
        blocks_.clear();
        agents_.clear();
        goals_.clear();
        camera_->detach();
        world_.clean_everything();
    }

    int img_height() const { return img_height_; }
    int img_width() const { return img_width_; }

    void build_scene(const std::vector<X3ItemInfo>& infos,
                     const std::string& floor_file,
                     const std::string& stadium_file);

    void add_item(const X3ItemInfo& info);

    void act(std::string agent_name, int a, float& reward, int& event) {
        auto it = agents_.find(agent_name);
        CHECK(it != agents_.end()) << "Agent " << agent_name << " not found.";
        apply_action(it->second, a, reward, event);
        step();
    }

    roboschool::RenderResult render(std::string agent_name, bool debug);

private:
    void apply_action(const X3AgentPtr& agent, int a, float& reward, int& event);

    void remove_goal(X3Goal* goal);

    X3Goal* collect_goal(const X3AgentPtr& agent);

    void step();

    int frame_skip_;
    int img_height_;     // size for opengl rendering (used for show and debug)
    int img_width_;      // size for opengl rendering (used for show and debug)

    // large sizes will crash in camera render
    static const int IMG_HEIGHT_SHOW = 512;
    static const int IMG_WIDTH_SHOW = 512;

    World world_;
    std::map<std::string, X3BlockPtr> blocks_;
    std::map<std::string, X3GoalPtr> goals_;
    std::map<std::string, X3AgentPtr> agents_;
    std::unique_ptr<X3Camera> camera_;
    Thingy floor_;
};

}} // simulator::xworld_3d
