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

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <math.h>
#include <random>

#include "x3scene.h"

namespace simulator {
namespace xworld3d {

using simulator::util::path_join;

X3Scene::X3Scene(float gravity, float time_step, int frame_skip):
        frame_skip_(frame_skip),
        img_height_(FLAGS_x3_img_height),
        img_width_(FLAGS_x3_img_width),
        world_(gravity, time_step) {
    dt_ = time_step * frame_skip_;
    world_.set_glsl_path(FLAGS_x3_glsl_path);
    camera_ = std::unique_ptr<X3Camera>(
            new X3Camera(world_, img_height_, img_width_));
    blocks_.clear();
    agents_.clear();
    goals_.clear();
}

void X3Scene::build_scene(const std::vector<X3ItemInfo>& infos,
                          const std::string& floor_file,
                          const std::string& stadium_file) {
    clear_scene();

    std::vector<Object> olist = world_.load_mjcf(floor_file);
    for (auto& o : olist) {
        o.query_position();
    }

    Pose stadium_pose(0.0f, 0.0f, -0.05f);
    auto t = world_.load_thingy(stadium_file, stadium_pose,
                                0.01f, 0.0f, 0xffffff, true);

    for (auto const& i : infos) {
        add_item(i);
    }
    CHECK_GT(agents_.size(), 0) << "There should be at least one agent.";
}

void X3Scene::add_item(const X3ItemInfo& info) {
    if (info.type == AGENT) {
        CHECK(agents_.find(info.name) == agents_.end())
                << "An agent with name " << info.name << " exists.";
        auto agent = std::make_shared<X3Agent>(info, world_,
                                               FLAGS_x3_speed_norm,
                                               FLAGS_x3_orientation_bins,
                                               FLAGS_x3_reaching_distance);
        agents_[info.name] = agent;
    } else if (info.type == GOAL) {
        CHECK(goals_.find(info.name) == goals_.end())
                << "A goal with name " << info.name << " exists.";
        auto goal = std::make_shared<X3Goal>(info, world_);
        goals_[info.name] = goal;
    } else if (info.type == BLOCK) {
        CHECK(blocks_.find(info.name) == blocks_.end())
                << "An item with name " << info.name << " exists.";
        auto block = std::make_shared<X3Block>(info, world_);
        blocks_[info.name] = block;
    } else {
        LOG(ERROR) << "unknown item type: " << info.type;
    }
}

void X3Scene::apply_action(const X3AgentPtr& agent, int a,
                           float& reward, int& event) {
    reward = STEP_PENALTY;
    event = X3Event::NOTHING;

    switch (a) {
        case X3NavAction::MOVE_FORWARD:
            agent->move_forward();
            break;
        case X3NavAction::TURN_LEFT:
            agent->turn_left();
            break;
        case X3NavAction::TURN_RIGHT:
            agent->turn_right();
            break;
        case X3NavAction::JUMP:
            agent->jump();
            break;
        case X3NavAction::COLLECT:
            {
                X3Goal* goal = collect_goal(agent);
                if (goal) {
                    remove_goal(goal);
                    reward = COLLECT_GOAL;
                    event = X3Event::CORRECT_GOAL;
                } else {
                    reward = WRONG_OP;
                    event = X3Event::WRONG_ACTION;
                }
            }
            break;
        default:
            LOG(ERROR) << "unknown action id: " << a;
            event = X3Event::WRONG_ACTION;
    }
}

roboschool::RenderResult X3Scene::render(std::string agent_name, bool debug) {
    auto it = agents_.find(agent_name);
    CHECK(it != agents_.end())
            << "agent " << agent_name << " not found.";
    return camera_->render(it->second.get(), debug);
}

void X3Scene::remove_goal(X3Goal* goal) {
    auto it = goals_.find(goal->name());
    if (it != goals_.end()) {
        auto& g = it->second;
        world_.remove_object(g->object());
        g->destroy();
        goals_.erase(it);
    }
}

X3Goal* X3Scene::collect_goal(const X3AgentPtr& agent) {
    X3Goal* goal = NULL;
    float best_score = 0.5;
    float score;
    for (auto& it : goals_) {
        score = agent->reach_test(it.second->pose());
        if (score > best_score) {
            goal = it.second.get();
            best_score = score;
        }
    }

    return goal;
}

void X3Scene::step() {
    world_.step(frame_skip_);
}

}} // simulator::xworld3d
