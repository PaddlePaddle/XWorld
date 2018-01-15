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
#include <set>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <memory>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "simulator.h"
#include "teacher.h"
#include "xworld3d_flags.h"

namespace simulator {
namespace xworld3d {

class X3SimulatorImpl;

class X3Simulator : public GameSimulatorMulti, public TeachingEnvironment {
public:
    X3Simulator(bool print_xworld_config, bool big_screen/*for debug*/ = false);

    ~X3Simulator();

    void reset_game() override;

    int game_over() override;

    int get_num_actions() override;

    int get_lives() override;

    void show_screen(float reward) override;

    float take_action(const StatePacket& actions) override;

    void get_screen(StatePacket& screen) override;

    void define_state_specs(StatePacket& state) override;

    void get_extra_info(std::string& info) override;

    void get_screen_out_dimensions(size_t& height,
                                   size_t& width,
                                   size_t& channels) override;

    int add_agent() override;

    //// TeachingEnvironment
    void apply_teacher_actions() override;

    void get_world_dimensions(double& X, double& Y, double& Z) override;

    std::string get_events_of_game() override;

    // get the information for all the entities
    void get_all_entities(std::vector<Entity>& entities) override;

    boost::python::object get_py_env() override;

    void update_environment() override;

    std::string conf_file();

private:
    void record_collision_events(const std::set<std::string>& collision_list);

    cv::Mat get_reward_image(float reward);  // get the sub-image for reward

    cv::Mat get_message_image(
        std::deque<std::string>& messages);  // get history msg images

    //// image process
    void update_message_box_on_screen();  // reward msg box on the screen with
                                          // updated msgs

    void resize_image_to_frame(const cv::Mat& img,
                               GameFrame& frame,
                               size_t img_height_out,
                               size_t img_width_out,
                               bool color);

    cv::Mat concat_images(cv::Mat img1, cv::Mat img2, bool is_vertical);

    std::unique_ptr<X3SimulatorImpl> impl_;

    std::vector<size_t> legal_actions_;
    size_t height_;
    size_t width_;
    size_t img_height_out_;  // training input image size
    size_t img_width_out_;   // training input image size
    bool bird_view_;
    cv::Mat prev_screen_;  // previous screen for display
    cv::Mat screen_;

    std::vector<std::string> agent_received_sentences_;
    std::vector<Vec3> agent_init_positions_;  // the init position of the current session
    std::vector<size_t> agent_prev_actions_;  // the agent's action in the previous time step
    static const int n_history_ =
        25;  // how many history messages to display in the message box
             // that is next to the xworld map screen
             // this variable won't affect the game and the training
    std::deque<std::string>
        history_messages_;  // history message buffer for showing in the GUI
    std::string game_events_;
};

}} // simulator::xworld3d
