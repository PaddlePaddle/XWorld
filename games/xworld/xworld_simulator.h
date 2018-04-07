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
#include "xworld/xworld.h"

#include <deque>
#include "simulator.h"
#include "teacher.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

DECLARE_string(xwd_conf_path);
DECLARE_int32(visible_radius);
DECLARE_string(task_mode);
DECLARE_bool(task_groups_exclusive);
DECLARE_bool(log_hist);

namespace simulator {
namespace xwd {

/**
*  class that acts as the interface for interacting with XWorld
*  using the same interface as GameSimulatorMulti
*/

class XWorldSimulator : public GameSimulatorMulti, public TeachingEnvironment {
  public:
    XWorldSimulator(bool print_xworld_config);
    ~XWorldSimulator() {}
    virtual void reset_game() override;

    // game over when either reaching goal or passing the number of max steps
    virtual int game_over() override;

    // called by AgentSpecificSimulator
    virtual float take_action(const StatePacket& actions) override;

    // get the number of actions the agent (learner) possesses
    virtual int get_num_actions() override;

    // get the number of lives
    virtual int get_lives() override;

    // called by AgentSpecificSimulator
    virtual void get_screen(StatePacket& screen) override;

    // visualize the current screen
    virtual void show_screen(float reward) override;

    // called by AgentSpecificSimulator
    void define_state_specs(StatePacket& state);

    // called by AgentSpecificsimulator
    virtual void get_extra_info(std::string& info) override;

    // add an agent to the XWorld with name as agent_name (should be unique)
    virtual int add_agent() override;

    void apply_teacher_actions() override;

    void get_world_dimensions(double& X, double& Y, double& Z) override;

    // get the information for all the entities
    void get_all_entities(std::vector<Entity>& entities) override;

    boost::python::object get_py_env() override;

    void update_environment() override;

    // these are the dimensions that are used by the CNN
    void get_screen_out_dimensions(size_t& img_height_out,
                                   size_t& img_width_out,
                                   size_t& channels);

    std::string conf_file() { return xworld_.conf_file(); }

  private:
    void init();

    std::string get_teacher_sentence_for_agent();

    void get_screen_rgb(GameFrame& rgbs);  // get the current screenshot (color)

    void update_message_box_on_screen();  // reward msg box on the screen with
                                          // updated msgs

    cv::Mat get_command_image(const std::string& cmd);

    cv::Mat get_reward_image(float reward);  // get the sub-image for reward

    cv::Mat get_message_image(
        std::deque<std::string>& messages);  // get history msg images

    // concatenate two images vertically if is_vertical is true and horizontally
    // otherwise
    cv::Mat concat_images(cv::Mat img1, cv::Mat img2, bool is_vertical);

    // downsample and convert to grays
    // resize the input image to the size of [IMG_HEIGHT_OUT, IMG_WIDTH_OUT]
    void down_sample_image(const GameFrame& screen,
                           GameFrame& screen_out,
                           bool color = false);

    xwd::XWorld xworld_;  // the environment for all the agents
    std::vector<std::string> agent_received_sentences_;

    int height_;  // size of the world (in terms of building blocks)
    int width_;
    int img_height_;      // original image size
    int img_width_;       // original image size
    int img_height_out_;  // resized image size
    int img_width_out_;   // resized image size

    static const int n_history_ =
        25;  // how many history messages to display in the message box
             // that is next to the xworld map screen
             // this variable won't affect the game and the training

    std::deque<std::string>
        history_messages_;  // history message buffer for showing in the GUI

    cv::Mat prev_screen_;  // previous screen for display
    cv::Mat screen_;
    int keyboard_action_;
};
}
} /* namespace simulator::xwd */
