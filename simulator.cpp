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

#include "simulator.h"
#include <sstream>
#include <thread>

// the total number of channels for one forward should be context * (color? 3:
// 1)
DEFINE_int32(context, 1, "Use so many frames as input");
DEFINE_int32(max_steps, 0, "maxinum steps for the game");
DEFINE_bool(lock_step, false, "step-by-step inspection");
DEFINE_bool(color, false, "whether the screen is grayscale or color");
DEFINE_bool(pause_screen, false, "show screen continuously or paused by hand");
DEFINE_string(curriculum_stamp, "", "the file that records the curriculum progress (an int)");

namespace simulator {

std::mutex GameSimulator::s_display_mutex_;

GameSimulator::GameSimulator()
        : num_steps_(0), last_action_(""), last_action_success_(true) {}

void GameSimulator::init_context_screens(bool is_uint8) {
    StatePacket screen;
    get_screen(screen);
    CHECK_EQ(screen.size(), 1);
    auto single_screen_size = screen.get_buffer("screen")->get_value_width();
    screens_ = StatePacket();
    if (is_uint8) {
        std::vector<uint8_t> contexts(FLAGS_context * single_screen_size, 0);
        screens_.add_buffer_value("screen", contexts);
    } else {
        std::vector<float> contexts(FLAGS_context * single_screen_size, 0);
        screens_.add_buffer_value("screen", contexts);
    }
}

template <typename T>
void GameSimulator::shift_context(BufferPtr cur_buf,
                                  BufferPtr context_buf,
                                  int n_contexts,
                                  size_t screen_sz) {
    auto data = context_buf->get_value<T>();
    memmove(data, data + screen_sz, sizeof(T) * (n_contexts - 1) * screen_sz);
    data += (n_contexts - 1) * screen_sz;
    cur_buf->copy_value(data, data + screen_sz);
}

void GameSimulator::make_context_screens() {
    StatePacket curr_screen;
    get_screen(curr_screen);
    CHECK_EQ(curr_screen.size(), 1);
    auto curr_screen_buffer = curr_screen.get_buffer("screen");

    bool is_uint8 = curr_screen_buffer->get_value()->is_uint8();
    if (screens_.size() < 1) {
        // init for the first time
        init_context_screens(is_uint8);
    }
    auto screens_buffer = screens_.get_buffer("screen");

    size_t sz = curr_screen_buffer->get_value_size();
    CHECK_EQ(screens_buffer->get_value_size(), FLAGS_context * sz);

    if (is_uint8) {
        shift_context<uint8_t>(
            curr_screen_buffer, screens_buffer, FLAGS_context, sz);
    } else {
        shift_context<float>(
            curr_screen_buffer, screens_buffer, FLAGS_context, sz);
    }
}

void GameSimulator::fill_in_reward_and_screen(float reward,
                                              StatePacket& state) {
    // The state should have been allocated in the derived get_state_data()
    CHECK_GE(state.size(), GameSimulator::N_BUFFERS);
    // insert the cumulative reward
    std::vector<float> vec = {reward};
    state.get_buffer("reward")->set_value(vec.begin(), vec.end());
    // copy screens
    state.copy_from_by_key(screens_, "screen");
}

float GameSimulator::take_actions(const StatePacket& actions, int actrep, bool screen, float acc_reward) {
    float reward = 0;
    num_steps_++;
    for (int i = 0; i < actrep; i++) {
        if (screen) {
            show_screen(acc_reward + reward);
        }
        reward += take_action(actions);
    }
    return reward;
}

void GameSimulator::init_screen() {
    screens_ = StatePacket();
    make_context_screens();
}

void GameSimulator::reset_game() {
    num_steps_ = 0;
}

std::string GameSimulator::get_screen_name() {
    std::stringstream ss;
    ss << std::this_thread::get_id();
    return ss.str();
}

std::string GameSimulator::decode_game_over_code(int code) {
    if (code == 0) {
        return "alive";
    }
    std::string code_str = "";
    if (code & MAX_STEP) {
        code_str += "max_step|";
    }
    if (code & DEAD) {
        code_str += "dead|";
    }
    if (code & SUCCESS) {
        code_str += "success|";
    }
    if (code & LOST_LIFE) {
        code_str += "lost_life|";
    }
    CHECK(!code_str.empty());
    return code_str.substr(0, code_str.length() - 1);
}

// adaptor class: interface for agent-specific simulator
AgentSpecificSimulator::AgentSpecificSimulator(SimulatorMultiPtr simulator_ptr,
                                               int agent_id /*= 0*/)
    : simulator_ptr_(simulator_ptr), agent_id_(agent_id) {
}

void AgentSpecificSimulator::reset_game() {
    activate_my_agent();
    simulator_ptr_->reset_game();
    GameSimulator::reset_game();
}

int AgentSpecificSimulator::game_over() {
    activate_my_agent();
    return GameSimulator::game_over() | simulator_ptr_->game_over();
}

int AgentSpecificSimulator::get_num_actions() {
    activate_my_agent();
    return simulator_ptr_->get_num_actions();
}

float AgentSpecificSimulator::take_action(const StatePacket& actions) {
    activate_my_agent();
    return simulator_ptr_->take_action(actions);
}

void AgentSpecificSimulator::get_screen(StatePacket& screen) {
    activate_my_agent();
    simulator_ptr_->get_screen(screen);
}

void AgentSpecificSimulator::define_state_specs(StatePacket& state) {
    activate_my_agent();
    simulator_ptr_->define_state_specs(state);
}

void AgentSpecificSimulator::get_extra_info(std::string& info) {
    activate_my_agent();
    simulator_ptr_->get_extra_info(info);
}

void AgentSpecificSimulator::get_screen_out_dimensions(size_t& height,
                                                       size_t& width,
                                                       size_t& channels) {
    simulator_ptr_->get_screen_out_dimensions(height, width, channels);
}

void AgentSpecificSimulator::show_screen(float reward) {
    activate_my_agent();
    simulator_ptr_->show_screen(reward);
}

int AgentSpecificSimulator::get_lives() {
    activate_my_agent();
    return simulator_ptr_->get_lives();
}

}  // namespace simulator
