// Copyright (c) 2017 Baidu Inc. All Rights Reserved.

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software // distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <deque>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "simulator.h"
#include "x3scene.h"
#include "xworld3d_flags.h"

namespace simulator {
namespace xworld3d {

/**
*  Implementation class that actually interacts with X3Scene
*/
class X3SimulatorImpl {
    friend class X3Simulator;
public:
    X3SimulatorImpl(std::string name,
                    float gravity, float time_step, int frame_skip);

    ~X3SimulatorImpl() {}

private:
    void reset_game();

    // game over when either reaching goal or passing the number of max steps
    int game_over();

    // called by AgentSpecificSimulator
    float take_action(const StatePacket& actions);

    // get the number of actions the agent (learner) possesses
    int get_num_actions() { return legal_actions_.size(); }

    // get the number of lives
    int get_lives();

    // called by AgentSpecificSimulator
    void get_screen(StatePacket& screen);

    // visualize the current screen
    void show_screen(float reward);

    void get_screen_rgb();  // get the current screenshot (color)

    // downsample and convert to grays
    // resize the input image to the size of [IMG_HEIGHT_OUT, IMG_WIDTH_OUT]
    void down_sample_image(GameFrame& screen_out, bool color = false);

    cv::Mat get_current_screen();

    // called by AgentSpecificSimulator
    void define_state_specs(StatePacket& state);


    int world_size() { return world_size_; }

    // these are the dimensions that are used by the CNN
    void get_screen_out_dimensions(size_t& img_height_out,
                                   size_t& img_width_out,
                                   size_t& channels);

    void get_world_dimensions(size_t& height, size_t& width);

    void generate_scene(std::vector<X3EntityInfo>& entity_infos);

    X3Scene x3scene_;  // the environment for all the agents
    std::vector<X3NavAction> legal_actions_;
    std::string agent_name_;
    size_t world_size_;  // size of the world (in terms of building blocks)
    int img_height_;      // original image size
    int img_width_;       // original image size
    size_t img_height_out_;  // resized image size
    size_t img_width_out_;   // resized image size
    int event_of_last_action_;
    GameFrame curr_screen_;

    static const int GRID_SIZE = 12;
};

}} /* namespace simulator::xwd */
