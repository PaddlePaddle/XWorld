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

#include <deque>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <memory>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "simulator.h"
#include "xworld3d_flags.h"

namespace simulator {
namespace xworld3d {

class X3SimulatorImpl;

class X3Simulator : public GameSimulator {
public:
    X3Simulator(bool print_xworld_config,
                float gravity,
                float time_step,
                int frame_skip,
                bool big_screen = false);  // big_screen used for debugging

    void reset_game() override;

    int game_over() override;

    int get_num_actions() override;

    int get_lives() override;

    void show_screen(float reward) override;

    float take_action(const StatePacket& actions) override;

    void get_screen(StatePacket& screen) override;

    void define_state_specs(StatePacket& state) override;

    void get_screen_out_dimensions(size_t& height,
                                   size_t& width,
                                   size_t& channels) override;

private:
    std::shared_ptr<X3SimulatorImpl> impl_;
};

}} // simulator::xworld3d
