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

#include "simulator.h"
#include <sstream>
#include <thread>

// the total number of channels for one forward should be context * (color? 3: 1)
DEFINE_int32(context, 1, "Use so many frames as input");
DEFINE_int32(max_steps, 0, "maxinum steps for the game");
DEFINE_bool(lock_step, false, "step-by-step inspection");
DEFINE_bool(color, false, "whether the screen is grayscale or color");
DEFINE_bool(pause_screen, false, "show screen continuously or paused by hand");

namespace simulator {

std::mutex GameSimulator::s_display_mutex_;

GameSimulator::GameSimulator() : max_steps_(FLAGS_max_steps),
                                 num_steps_(0),
                                 last_action_("") {
}

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
    memmove(data, data + screen_sz, sizeof(T) * (n_contexts-1) * screen_sz);
    data += (n_contexts-1) * screen_sz;
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

void GameSimulator::fill_in_reward_and_screen(float reward, StatePacket& state) {
    // The state should have been allocated in the derived get_state_data()
    CHECK_GE(state.size(), GameSimulator::N_BUFFERS);
    // insert the cumulative reward
    std::vector<float> vec = {reward};
    state.get_buffer("reward")->set_value(vec.begin(), vec.end());
    // copy screens
    state.copy_from_by_key(screens_, "screen");
}

float GameSimulator::take_actions(const StatePacket& actions, int actrep) {
    float reward = 0;
    num_steps_ ++;
    for (int i = 0; i < actrep; i ++) {
        reward += take_action(actions);
    }
    make_context_screens();
    return reward;
}

void GameSimulator::init_screen() {
    screens_ = StatePacket();
    make_context_screens();
}

void GameSimulator::reset_game() {
    num_steps_ = 0;
    init_screen();
}

std::string GameSimulator::get_screen_name() {
    std::stringstream ss;
    ss << std::this_thread::get_id();
    return ss.str();
}

// adaptor class: interface for agent-specific simulator
AgentSpecificSimulator::AgentSpecificSimulator(SimulatorMultiPtr simulator_ptr, int agent_id /*= 0*/)
                                             : simulator_ptr_(simulator_ptr), agent_id_(agent_id) {
    max_steps_ = simulator_ptr_->get_max_steps();
}

void AgentSpecificSimulator::reset_game() {
    activate_my_agent();
    simulator_ptr_->reset_game();
    GameSimulator::reset_game();
}

int AgentSpecificSimulator::game_over() {
    activate_my_agent();
    return GameSimulator::game_over()
           | simulator_ptr_->game_over();
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

void AgentSpecificSimulator::get_extra_info(std::unordered_map<std::string, std::string>& info) {
    activate_my_agent();
    simulator_ptr_->get_extra_info(info);
}

void AgentSpecificSimulator::get_screen_out_dimensions(size_t& height, size_t& width, size_t& channels) {
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

std::string AgentSpecificSimulator::last_action() {
    // last action taken by any agent
    return simulator_ptr_->last_action();
}

} // namespace simulator
