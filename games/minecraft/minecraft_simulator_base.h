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

#include <AgentHost.h>

#include "minecraft_simulator.h"

namespace simulator {
namespace mcw {

class MinecraftSimulatorBase : public MinecraftSimulator {
  public:
    MinecraftSimulatorBase(const std::string& conf);

    virtual ~MinecraftSimulatorBase() {}

    // reset game and start from scratch
    virtual void reset_game() override;

    // game over when either reaching goal or passing the number of max steps
    // return: the game_over code
    virtual int game_over() override;

    // get the number of actions the agent (learner) possesses
    virtual int get_num_actions() override { return action_names_.size(); }

    // get the number of lives
    virtual int get_lives() override { return 1; }

    // visualize the current screen
    virtual void show_screen(float reward) override;

    // take a certain action and return the reward
    virtual float take_action(const StatePacket& actions) override;

    // read the current screen
    virtual void get_screen(StatePacket& screen) override;

    virtual void get_screen_out_dimensions(size_t& height,
                                           size_t& width,
                                           size_t& channels) override {
        height = mission_->getVideoHeight(0);
        width = mission_->getVideoWidth(0);
        channels = 3;  // hardcoded as color
    }

    virtual void define_state_specs(StatePacket& state) override;

    static MinecraftSimulatorBase* create(const std::string& mission,
                                          const std::string& conf_file);

  protected:
    // Create a new mission in mission_. Called when reset_game is called
    virtual void new_mission() = 0;
    // Update internal states. Called when tack_action is called
    virtual void update_state(int action_id) = 0;

    // called by observe(). Should check if desired state after action is
    // reached
    virtual bool reached_desired_state() { return true; }

  protected:
    // get an valid observation to world_state_ and return the reward
    float observe();

    malmo::ClientPool client_pool_;
    malmo::AgentHost agent_host_;
    std::unique_ptr<malmo::MissionSpec> mission_;
    malmo::WorldState world_state_;
    std::vector<std::string> action_names_;

    bool quited_;  // whether "quit" command is sent already
};

class MinecraftSimulatorDemo : public MinecraftSimulatorBase {
  public:
    MinecraftSimulatorDemo(const std::string& conf_file)
        : MinecraftSimulatorBase(conf_file) {}

  protected:
    virtual void new_mission() override;
    virtual void update_state(int action_id) override;
    virtual bool reached_desired_state() override;

  private:
    static bool approximately_equal(float x, float y) {
        return std::abs(x - y) < 0.01;
    }
    float current_x_;
    float current_z_;
    float current_yaw_;
    bool got_reward_;
    bool reached_location_;
};
}
}  // namespace simulator::mcw
