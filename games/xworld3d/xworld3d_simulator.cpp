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

#include "x3scene.h"
#include "xworld3d_parser.h"
#include "xworld3d_simulator.h"

DECLARE_bool(pause_screen);
DECLARE_bool(color);

using std::make_shared;

namespace simulator {
namespace xworld3d {

/**
*  Implementation class that actually interacts with X3Scene
*/
class X3SimulatorImpl {
public:
    X3SimulatorImpl(const std::string& world_config,
                    const std::string& model_dir,
                    bool print, int curriculum,
                    float gravity, float time_step, int frame_skip);

    ~X3SimulatorImpl() {}

    std::string name() { return agent_name_; }

    void reset_game();

    // game over when either reaching goal or passing the number of max steps
    int game_over();

    int max_steps() {
        return (world_height_ * world_width_);
    }

    // called by AgentSpecificSimulator
    float take_action(int action_idx);

    // get the number of actions the agent (learner) possesses
    int get_num_actions() { return legal_actions_.size(); }

    // get the number of lives
    int get_lives();

    // called by AgentSpecificSimulator
    void get_screen(StatePacket& screen);

    // visualize the current screen
    cv::Mat show_screen(float reward);

    // called by AgentSpecificSimulator
    void define_state_specs(StatePacket& state);

    // these are the dimensions that are used by the CNN
    void get_screen_out_dimensions(size_t& img_height_out,
                                   size_t& img_width_out,
                                   size_t& channels);

    void get_world_dimensions(size_t& height, size_t& width);

private:
    void get_screen_rgb(GameFrame& screen, bool debug = false);  // get the current screenshot (color)

    void get_screen_rgb(cv::Mat& img, bool debug = false);  // get the current screenshot (color)

    // downsample and convert to grays
    // resize the input image to the size of [IMG_HEIGHT_OUT, IMG_WIDTH_OUT]
    void resize_image(const GameFrame& screen_in,
                      GameFrame& screen_out,
                      int img_height_out, int img_width_out,
                      bool color = false);

    cv::Mat concat_images(cv::Mat img1, cv::Mat img2,
                                           bool is_vertical);

    X3Scene x3scene_;  // the environment for all the agents
    X3Parser x3parser_;
    std::vector<X3NavAction> legal_actions_;
    std::string agent_name_;
    int world_height_;
    int world_width_;
    int img_height_;      // original image size
    int img_width_;       // original image size
    size_t img_height_out_;  // resized image size
    size_t img_width_out_;   // resized image size
    int event_of_last_action_;
    GameFrame curr_screen_;

    static const int GRID_SIZE = 32;
};

X3SimulatorImpl::X3SimulatorImpl(const std::string& world_config,
                                 const std::string& model_dir,
                                 bool print, int curriculum,
                                 float gravity,
                                 float time_step,
                                 int frame_skip) :
        x3scene_(gravity, time_step, frame_skip),
        x3parser_(world_config, model_dir, print, curriculum),
        legal_actions_({MOVE_FORWARD, TURN_LEFT, TURN_RIGHT, COLLECT}),
        agent_name_(""),
        img_height_(0), img_width_(0),
        event_of_last_action_(X3Event::NOTHING) {}

void X3SimulatorImpl::reset_game() {
    x3scene_.clear_scene();

    std::vector<X3ItemInfo> item_list;
    x3parser_.generate_map(item_list);
    x3scene_.build_scene(item_list,
                         x3parser_.get_model_file("ground"),
                         x3parser_.get_model_file("stadium"));

    world_height_ = x3parser_.height();
    world_width_ = x3parser_.width();
    img_height_ = x3scene_.img_height();
    img_width_ = x3scene_.img_width();
    img_height_out_ = world_height_ * GRID_SIZE;
    img_width_out_ = world_width_ * GRID_SIZE;
    curr_screen_.resize(3 * img_width_ * img_height_, 0);

    agent_name_ = x3parser_.assign_agent();
    event_of_last_action_ = X3Event::NOTHING;
}

int X3SimulatorImpl::game_over() {
    int status = ALIVE;

    if (event_of_last_action_ == CORRECT_GOAL) {
        status = SUCCESS;
    } else if (event_of_last_action_ == WRONG_GOAL) {
        status = DEAD;
    }

    return status;
}

float X3SimulatorImpl::take_action(int action_idx) {
    CHECK_LT(action_idx, get_num_actions());

    float reward = 0;
    x3scene_.act(agent_name_, legal_actions_[action_idx],
                 reward, event_of_last_action_);

    return reward;
}

inline int X3SimulatorImpl::get_lives() {
    return game_over() | DEAD ? 0 : 1;
}

void X3SimulatorImpl::get_screen(StatePacket& screen) {
    GameFrame screen_vec;
    get_screen_rgb(curr_screen_);
    resize_image(curr_screen_,
                 screen_vec,
                 img_height_out_, img_width_out_,
                 FLAGS_color);
    screen = StatePacket();
    screen.add_buffer_value("screen", screen_vec);
}

void X3SimulatorImpl::get_screen_out_dimensions(size_t& img_height_out,
                                                size_t& img_width_out,
                                                size_t& channels) {
    img_height_out = img_height_out_;
    img_width_out = img_width_out_;
    channels = (FLAGS_color ? 3 : 1);
}

inline void X3SimulatorImpl::get_world_dimensions(size_t& height, size_t& width) {
    height = world_height_;
    width = world_width_;
}

void X3SimulatorImpl::get_screen_rgb(GameFrame& screen, bool debug) {
    roboschool::RenderResult render_result =
            x3scene_.render(agent_name_, debug);
    std::string render_str = std::get<0>(render_result);
    std::memcpy(screen.data(),
                render_str.c_str(),
                screen.size() * sizeof(uchar));
}

void X3SimulatorImpl::get_screen_rgb(cv::Mat& screen, bool debug) {
    roboschool::RenderResult render_result =
            x3scene_.render(agent_name_, debug);
    std::string render_str = std::get<0>(render_result);
    int img_height = std::get<4>(render_result);
    int img_width = std::get<5>(render_result);

    screen.create(img_height, img_width, CV_8UC3);
    for (size_t h = 0, p = 0; h < img_height_; ++h) {
        for (size_t w = 0; w < img_width_; ++w, p += 3) {
            cv::Vec3b& color = screen.at<cv::Vec3b>(cv::Point(w, h));
            color[0] = render_str[p + 2];
            color[1] = render_str[p + 1];
            color[2] = render_str[p];
        }
    }
}

void X3SimulatorImpl::resize_image(const GameFrame& screen_in,
                                   GameFrame& screen_out,
                                   int img_height_out, int img_width_out,
                                   bool color /* =false */) {
    cv::Mat img(img_height_, img_width_, CV_8UC3);
    for (size_t h = 0, p = 0; h < img_height_; ++h) {
        for (size_t w = 0; w < img_width_; ++w, p += 3) {
            cv::Vec3b& color = img.at<cv::Vec3b>(cv::Point(w, h));
            color[0] = screen_in[p + 2];
            color[1] = screen_in[p + 1];
            color[2] = screen_in[p];
        }
    }
    cv::Mat img_out(img_height_out, img_width_out, CV_8UC3);
    cv::resize(img, img_out, img_out.size(), cv::INTER_LINEAR);

    size_t n_channels = (color ? 3 : 1);
    if (screen_out.size() != img_height_out * img_width_out * n_channels) {
        screen_out.resize(img_height_out * img_width_out * n_channels);
    }

    if (!color) {
        cv::cvtColor(img_out, img_out, cv::COLOR_BGR2GRAY);
    }

    for (int h = 0; h < img_height_out; ++h) {
        for (int w = 0; w < img_width_out; ++w) {
            for (size_t c = 0; c < n_channels; c++) {
                // normalize the pixel vals to [0,1]
                int offset = c * img_width_out * img_height_out +
                             h * img_width_out + w;
                screen_out[offset] =
                    (color ? img_out.at<cv::Vec3b>(cv::Point(w, h)).val[c]
                           : img_out.at<uchar>(cv::Point(w, h)));
            }
        }
    }
}

cv::Mat X3SimulatorImpl::concat_images(cv::Mat img1,
                                       cv::Mat img2,
                                       bool is_vertical) {
    cv::Mat img_concat(
        !is_vertical ? std::max(img1.rows, img2.rows) : img1.rows + img2.rows,
        is_vertical ? std::max(img1.cols, img2.cols) : img1.cols + img2.cols,
        CV_8UC3,
        cv::Scalar(0, 0, 0));
    cv::Mat ROI1(img_concat, cv::Rect(0, 0, img1.cols, img1.rows));
    cv::Mat ROI2(img_concat,
                 cv::Rect(is_vertical ? 0 : img1.cols,
                          is_vertical ? img1.rows : 0,
                          img2.cols,
                          img2.rows));
    img1.copyTo(ROI1);
    img2.copyTo(ROI2);
    return img_concat;
}

cv::Mat X3SimulatorImpl::show_screen(float reward) {
    cv::Mat img;
    cv::Mat img1(img_height_out_, img_width_out_, CV_8UC3);
    cv::Mat img2(img_height_out_, img_width_out_, CV_8UC3);

    get_screen_rgb(img, false);
    cv::resize(img, img1, img1.size(), cv::INTER_LINEAR);
    get_screen_rgb(img, true);
    cv::resize(img, img2, img2.size(), cv::INTER_LINEAR);

    return concat_images(img1, img2, false);

    // cv::Mat img;
    // get_screen_rgb(img);
    //
    // return img;
}

X3Simulator::X3Simulator(const std::string& world_config,
                         const std::string& model_dir,
                         bool print, int curriculum,
                         float gravity,
                         float time_step,
                         int frame_skip) : GameSimulator() {
    impl_ = std::make_shared<X3SimulatorImpl>(
            world_config, model_dir, print, curriculum,
            gravity, time_step, frame_skip);
}

void X3Simulator::reset_game() {
    impl_->reset_game();
    max_steps_ = impl_->max_steps();
    GameSimulator::reset_game();
}

int X3Simulator::game_over() {
    return GameSimulator::game_over() | impl_->game_over();
}

int X3Simulator::get_num_actions() {
    return impl_->get_num_actions();
}

int X3Simulator::get_lives() {
    return impl_->get_lives();
}

void X3Simulator::show_screen(float reward) {
    cv::Mat img = impl_->show_screen(reward);

    // show screen
    std::lock_guard<std::mutex> guard(GameSimulator::s_display_mutex_);
    std::string window_name = get_screen_name();
    cv::namedWindow(window_name);
    cv::imshow(window_name, img);
    if (FLAGS_pause_screen) {
        // The screen will pause at every step waiting for keyboard
        cv::waitKey(-1);
    } else {
        // Default mode: the screen will display continuously
        cv::waitKey(1);
    }
}

float X3Simulator::take_action(const StatePacket& actions) {
    CHECK(actions.contain_key("action"))
            << "The agent has to take the move action.";
    int action_idx = *(actions.get_buffer("action")->get_id());
    last_action_ = std::to_string(action_idx);
    if (FLAGS_lock_step) {
        int key = cv::waitKey(0) % 256;
        switch (key) {
            case 'w':
                action_idx = 0;
                break;
            case 'a':
                action_idx = 1;
                break;
            case 'd':
                action_idx = 2;
                break;
            case 'c':
                action_idx = 3;
                break;
            default:
                break;
        }
    }
    return impl_->take_action(action_idx);
}

void X3Simulator::get_screen(StatePacket& screen) {
    impl_->get_screen(screen);
}

void X3Simulator::define_state_specs(StatePacket& state) {
    state = StatePacket();
    state.add_key("reward");
    state.add_key("screen");
}

void X3Simulator::get_screen_out_dimensions(size_t& height,
                                            size_t& width,
                                            size_t& channels) {
    impl_->get_screen_out_dimensions(height, width, channels);
}

}} // namespace simulator::xworld3d
