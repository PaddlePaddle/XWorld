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
#include <memory>
#include <vector>
#include "simulator.h"

namespace simulator { namespace simple_game {

class SimpleGameEngine {
public:
    typedef int Action;
    typedef std::vector<Action> ActionVect;

    SimpleGameEngine(size_t array_size);

    void reset_game();

    bool game_over();

    float act(Action a);

    void get_screen(GameFrame &state_vec);

    float get_reward();

    ActionVect get_action_set();

    int get_num_actions() {
        return NUM_ACTIONS;
    }

private:
    bool valid_range() {
        return _cur_pos >= 0 && _cur_pos < static_cast<int>(_state_vec.size());
    }

    static const int NUM_ACTIONS = 2;
    static constexpr float MOVE_REWARD = -0.1f;
    static constexpr float DEST_REWARD = 4.0f;

    size_t _array_size;
    GameFrame _state_vec;
    std::vector<float> _rewards;
    int _cur_pos;
};

// A synthetic simple game.
// The agent moves along a line. The agent starts at the middle of the line.
// It have two acions in each step: move left of right.
// Each move has a reward determined by MOVE_REWARD (-0.1).
// There are two rewards at both ends of the line.
// The left end has reward  DEST_REWARD (4.0).
// The right end has reward DEST_REWARD/2 (2.0).
// The game ends if the agent arrives at either end.
// The optimal policy is to always move the left.
class SimpleGame : public GameSimulator {
public:
    SimpleGame(size_t array_size = ARRAY_SIZE);

    virtual void reset_game() override;

    virtual int game_over() override;

    virtual int get_num_actions() override {
        return _legal_actions.size();
    }

    virtual int get_lives() override;

    virtual void show_screen(float reward) override;

    virtual float take_action(const StatePacket& actions) override;

    virtual void get_screen(StatePacket &screen) override;

    void define_state_specs(StatePacket& state);

    virtual void get_screen_out_dimensions(size_t& height, size_t& width, size_t& channels) override;

private:
    static const size_t ARRAY_SIZE = 16;

    float get_reward();

    size_t _array_size;

    SimpleGameEngine _game;
    SimpleGameEngine::ActionVect _legal_actions;
};

}} // namespace simulator::simple_game
