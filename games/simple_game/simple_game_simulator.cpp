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

#include "simple_game_simulator.h"
#include <memory>
#include <vector>

namespace simulator { namespace simple_game {

SimpleGameEngine::SimpleGameEngine(size_t array_size) :
        _array_size(array_size),
        _state_vec(array_size, 0),
        _rewards(array_size, 0.0f) {
    reset_game();
}

void SimpleGameEngine::reset_game() {
    _cur_pos = _array_size / 2;
    std::fill(_state_vec.begin(), _state_vec.end(), 0);
    _state_vec[_cur_pos] = 1;
    std::fill(_rewards.begin(), _rewards.end(), 0);
    _rewards[_array_size - 1] = DEST_REWARD / 2;
    _rewards[0] = DEST_REWARD;
}

bool SimpleGameEngine::game_over() {
    return _cur_pos <= 0
           || _cur_pos >= static_cast<int>(_state_vec.size() - 1);
}

float SimpleGameEngine::act(Action a) {
    if (game_over()) return get_reward();
    int action_id = (int)a;
    switch (action_id) {
        case 0: // move left
            _state_vec[_cur_pos] = 0;
            --_cur_pos;
            break;
        case 1:
            _state_vec[_cur_pos] = 0;
            ++_cur_pos;
            break;
        default:
            LOG(FATAL) << "undefined action_id: " << action_id;
    }
    if (valid_range()) {
        _state_vec[_cur_pos] = 1;
    }
    return get_reward();
}

void SimpleGameEngine::get_screen(GameFrame &state_vec) {
    state_vec = _state_vec;
}

float SimpleGameEngine::get_reward() {
    float reward = MOVE_REWARD;
    if (valid_range() && _rewards[_cur_pos] != 0.0) {
        reward = _rewards[_cur_pos];
        _rewards[_cur_pos] = 0.0;
    }
    return reward;
}

SimpleGameEngine::ActionVect SimpleGameEngine::get_action_set() {
    return ActionVect({0,1});
}

SimpleGame::SimpleGame(size_t array_size) :
        _game(array_size) {
    _legal_actions = _game.get_action_set();
    _game.reset_game();
}

void SimpleGame::reset_game() {
    _game.reset_game();
    GameSimulator::reset_game();
}

int SimpleGame::game_over() {
    return GameSimulator::game_over()
            | (_game.game_over()? SUCCESS: ALIVE);
}

float SimpleGame::take_action(const StatePacket& actions) {
    CHECK_EQ(actions.size(), 1);
    int action_id = *(actions.get_buffer("action")->get_id());
    CHECK_LT(action_id, _legal_actions.size());
    SimpleGameEngine::Action a = _legal_actions[action_id];
    return _game.act(a);
}

void SimpleGame::get_screen(StatePacket &screen) {
    GameFrame screen_vec;
    _game.get_screen(screen_vec);
    screen = StatePacket();
    screen.add_buffer_value("screen", screen_vec);
}

void SimpleGame::define_state_specs(StatePacket& state) {
    state = StatePacket();
    state.add_key("reward");
    state.add_key("screen");
}

void SimpleGame::get_screen_out_dimensions(size_t& height, size_t& width, size_t& channels) {
    height = 1;
    width = _array_size;
    channels = 1;
}

void SimpleGame::show_screen(float reward) {
    std::stringstream ss;
    GameFrame state_vec;
    _game.get_screen(state_vec);
    for (size_t i = 0; i < state_vec.size(); i++) {
        ss << (int)state_vec[i] << " ";
    }
    ss << std::endl;
    LOG(INFO) << "state_vec: " << ss.str();
}

int SimpleGame::get_lives() {
    return game_over()? 0 : 1;
}

}} // namespace simulator::simple_game
