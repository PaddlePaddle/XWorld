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
#include <vector>
#include <map>
#include <boost/python.hpp>
#include <Python.h>
#include "simulator.h"

namespace simulator { namespace robo_simulator {

class RoboSimulator : public GameSimulator {
public:
    RoboSimulator(bool human_control = false);

    virtual ~RoboSimulator() {Py_Finalize();}

    virtual void reset_game() override;

    virtual int game_over() override;

    virtual int get_num_actions() override;

    virtual void show_screen(float reward) override;

    virtual int get_lives() override;

    virtual float take_action(const StatePacket& actions) override;

    virtual void get_screen(StatePacket& screen) override;

    virtual void get_screen_out_dimensions(size_t& height, size_t& width,
        size_t& channels) override;

    void define_state_specs(StatePacket& state);

private:
    std::vector<uint8_t> _screen_vec;
    int _img_height;
    int _img_width;
    boost::python::object py_space;
    bool _human_control;

    const int _max_lives = 1;
    bool _reset_init = false;
    std::vector<char> _action_space;
    std::map<char, int> _action_to_id;
};

}} // namespace simulator::robo_simulator
