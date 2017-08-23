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
