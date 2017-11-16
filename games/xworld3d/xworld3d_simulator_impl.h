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

#include "xworld3d.h"
#include "xworld3d_simulator.h"

namespace simulator {
namespace xworld3d {

/**
*  Implementation class that actually interacts with X3World
*/
class X3SimulatorImpl {
public:
    X3SimulatorImpl(const std::string& conf, bool print, bool big_screen = false);

    ~X3SimulatorImpl() {}

    void reset_game();

    // get the number of actions the agent (learner) possesses
    int get_num_actions() { return legal_actions_.size(); }

    // called by AgentSpecificSimulator
    void define_state_specs(StatePacket& state);

    void get_world_dimensions(double& X, double& Y, double& Z);

    void get_all_entities(std::vector<Entity>& entities);

    boost::python::object get_py_env();

    size_t height() const { return height_; }

    size_t width() const { return width_; }

    void update_environment();

    bool act(const size_t agent_id, const int a);

    void get_screen_rgb(const size_t agent_id,
                        cv::Mat& img,
                        bool debug = false);  // get the current screenshot (color)

private:
    X3SimulatorImpl(const X3SimulatorImpl&) = delete;

    X3World xworld3d_;       // the environment for all the agents
    std::vector<X3NavAction> legal_actions_;
    size_t height_;          // unit: grid
    size_t width_;           // unit: grid
};

X3SimulatorImpl::X3SimulatorImpl(const std::string& conf,
                                 bool print,
                                 bool big_screen) :
        xworld3d_(conf, print, big_screen),
        legal_actions_({MOVE_FORWARD, TURN_LEFT, TURN_RIGHT, JUMP, COLLECT}),
        height_(0), width_(0) {}

void X3SimulatorImpl::reset_game() {
    xworld3d_.reset_world(true/* reset map */);

    height_ = xworld3d_.height();
    width_ = xworld3d_.width();
}

inline void X3SimulatorImpl::get_all_entities(std::vector<Entity>& entities) {
    xworld3d_.get_entities(entities);
}

inline boost::python::object X3SimulatorImpl::get_py_env() {
    return xworld3d_.get_py_env();
}

inline void X3SimulatorImpl::update_environment() {
    // do not reset the map, only do a minor update
    // according to the teacher
    xworld3d_.reset_world(false);
}

inline bool X3SimulatorImpl::act(const size_t agent_id, const int a) {
    return xworld3d_.act(agent_id, a);
}

void X3SimulatorImpl::get_screen_rgb(
        const size_t agent_id, cv::Mat& screen, bool debug) {
    roboschool::RenderResult render_result =
            xworld3d_.render(agent_id, debug);
    std::string render_str = std::get<0>(render_result);
    int img_height = std::get<4>(render_result);
    int img_width = std::get<5>(render_result);

    screen.create(img_height, img_width, CV_8UC3);
    // TODO: get rid of for loop
    for (size_t h = 0, p = 0; h < img_height; ++h) {
        for (size_t w = 0; w < img_width; ++w, p += 3) {
            cv::Vec3b& color = screen.at<cv::Vec3b>(cv::Point(w, h));
            color[0] = render_str[p + 2];
            color[1] = render_str[p + 1];
            color[2] = render_str[p];
        }
    }
}

}} // namespace simulator::xworld3d
