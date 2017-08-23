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

#include "deepmind_lab_game.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <glog/logging.h>
#include <thread>

DEFINE_string(runfiles_path, "", "Path of run files.");
DEFINE_string(level_script, "tests/demo_map", "DeepMind Lab level script.");

namespace simulator { namespace deepmind_lab_game {

std::mutex DeepmindLabGame::s_dmlab_mutex_;

DeepmindLabSimulatorBase* DeepmindLabSimulatorBase::create() {
    return new DeepmindLabGame();
}

DeepmindLabGame::DeepmindLabGame()
    : uniform_int_dist_(INT_MIN, INT_MAX),
      context_(NULL),
      episode_(0) {
    init();
}

void DeepmindLabGame::init() {
    CHECK(!FLAGS_runfiles_path.empty());

    env_status_ = EnvCApi_EnvironmentStatus_Terminated;
    // Environment initialization.
    DeepMindLabLaunchParams params;
    params.runfiles_path = FLAGS_runfiles_path.c_str();
    CHECK_EQ(dmlab_connect(&params, &env_c_api_, &context_), 0)
            << "Failed to connect RL API";
    CHECK_EQ(env_c_api_.setting(context_, "width", std::to_string(IMG_WIDTH).c_str()), 0)
            << "Failed to apply default 'width' setting.";
    CHECK_EQ(env_c_api_.setting(context_, "height", std::to_string(IMG_HEIGHT).c_str()), 0)
            << "Failed to apply default 'height' setting.";
    CHECK_EQ(env_c_api_.setting(context_, "controls", "external"), 0)
            << "Failed to apply 'controls' setting.";
    CHECK_EQ(env_c_api_.setting(context_, "appendCommand", " +set com_maxfps \"250\"") , 0)
            << "Failed to apply 'appendCommand' setting.";
    CHECK_EQ(env_c_api_.setting(context_, "levelName", FLAGS_level_script.c_str()),
             0) << "Invalid levelName flag " << FLAGS_level_script;

    CHECK_EQ(env_c_api_.init(context_), 0) << "Failed to init RL API";

    // Check spec
    CHECK_EQ(strcmp(env_c_api_.observation_name(context_, 0), "RGB_INTERLACED"), 0);
    CHECK_EQ(env_c_api_.action_discrete_count(context_), N_ORIGINAL_ACTIONS);

    EnvCApi_ObservationSpec spec;
    env_c_api_.observation_spec(context_, 0, &spec);
    CHECK_EQ(3, spec.dims);
    CHECK_EQ(IMG_HEIGHT, spec.shape[0]);
    CHECK_EQ(IMG_WIDTH, spec.shape[1]);
    CHECK_EQ(IMG_DEPTH, spec.shape[2]);
}

DeepmindLabGame::~DeepmindLabGame() {
    if (context_)
        env_c_api_.release_context(context_);
        context_ = nullptr;
}

void DeepmindLabGame::reset_game() {
    // without this lock, there is chance of segment fault when calling
    // `env_c_api_.start`
    std::lock_guard<std::mutex> guard(s_dmlab_mutex_);
    env_status_ = EnvCApi_EnvironmentStatus_Running;

    auto& reng = util::thread_local_reng();
    CHECK_EQ(env_c_api_.start(context_, episode_, uniform_int_dist_(reng)),
             0) << "Failed to start environment.";

    LOG(INFO) << "Episode: " << episode_;
    ++episode_;
    GameSimulator::reset_game();
}

int DeepmindLabGame::game_over() {
    return GameSimulator::game_over() |
           (env_status_ == EnvCApi_EnvironmentStatus_Running ? ALIVE : DEAD);
}

int DeepmindLabGame::get_num_actions() {
    CHECK_EQ(env_c_api_.action_discrete_count(context_), N_ORIGINAL_ACTIONS);
    if (actions_discrete_.empty()) {
        set_discrete_actions(&actions_discrete_);
    }
    CHECK(!actions_discrete_.empty());
    return actions_discrete_.size();
}

void DeepmindLabGame::show_screen(float reward) {
    cv::Mat img(IMG_HEIGHT, IMG_WIDTH, CV_8UC3);

    for (int i = 0; i < IMG_HEIGHT; ++i) {
        for (int j = 0; j < IMG_WIDTH; ++j) {
            cv::Vec3b& color = img.at<cv::Vec3b>(cv::Point(j, i));

            for (int k = 0; k < IMG_DEPTH; k++) {
                int index_src = j + i * IMG_WIDTH + k * IMG_WIDTH * IMG_HEIGHT;
                // cv::Vec3b has color order of B, G, R
                int index_dst = IMG_DEPTH - k - 1;
                color[index_dst] = screen_vec_[index_src];
            }
        }
    }

    const cv::Mat imgLarge(IMG_HEIGHT * 4 , IMG_WIDTH * 4, CV_8UC3);
    cv::resize(img, imgLarge, imgLarge.size());

    std::lock_guard<std::mutex> guard(GameSimulator::s_display_mutex_);
    cv::imshow(get_screen_name(), imgLarge);
    cv::waitKey(1);
}

int DeepmindLabGame::get_lives() {
    // life is not defined in this game.
    return 1;
}

float DeepmindLabGame::take_action(const StatePacket& actions) {
    CHECK_EQ(actions.size(), 1);
    int action_id = *(actions.get_buffer("action")->get_id());

    env_c_api_.act(context_, actions_discrete_[action_id].get(), NULL);
    // Advance by "actrep" step.
    // TODO: do we need actrep here? GameSimulator::take_actions
    // has already hanlded the action repetition.
    double reward = 0.0;
    env_status_ = env_c_api_.advance(context_, /*FLAGS_actrep*/1, &reward);
    return reward;
}

void DeepmindLabGame::get_screen(StatePacket& screen) {
    // Index 0 should be "RGB_INTERLACED"
    EnvCApi_Observation observation;
    env_c_api_.observation(context_, 0, &observation);

    CHECK_EQ(observation.spec.type, EnvCApi_ObservationBytes);
    const uint8_t* data = (const uint8_t*)observation.payload.bytes;

    screen_vec_.resize(IMG_DEPTH * IMG_WIDTH * IMG_HEIGHT);

    for (int i = 0; i < IMG_HEIGHT; ++i) {
        for (int j = 0; j < IMG_WIDTH; ++j) {
            for (int k = 0; k < IMG_DEPTH; ++k) {
                int index_src = k + j * IMG_DEPTH + i * IMG_WIDTH * IMG_DEPTH;
                int index_dst = j + i * IMG_WIDTH + k * IMG_WIDTH * IMG_HEIGHT;
                screen_vec_[index_dst] = data[index_src];
            }
        }
    }

    screen = StatePacket();
    screen.add_buffer_value("screen", screen_vec_);
}

void DeepmindLabGame::define_state_specs(StatePacket& state) {
    state = StatePacket();
    state.add_key("reward");
    state.add_key("screen");
}

void DeepmindLabGame::get_screen_out_dimensions(size_t& height, size_t& width, size_t& channels) {
    height = IMG_HEIGHT;
    width = IMG_WIDTH;
    channels = IMG_DEPTH;
}

void DeepmindLabGame::set_discrete_actions(std::vector<std::unique_ptr<int[]>>* actions_discrete) {

    const int N_DISCRETE_ACTIONS = 11;
    const int actions_discrete_definition[N_DISCRETE_ACTIONS][N_ORIGINAL_ACTIONS] = {
        { -20, 0, 0, 0, 0, 0, 0}, // look_left
        {20, 0, 0, 0, 0, 0, 0},   // look_right
        {0, -10, 0, 0, 0, 0, 0},   // look_up
        {0, 10, 0, 0, 0, 0, 0},  // look_down
        {0, 0, -1, 0, 0, 0, 0},   // strafe_left
        {0, 0, 1, 0, 0, 0, 0},    // strafe_right
        {0, 0, 0, 1, 0, 0, 0},    // forward
        {0, 0, 0, -1, 0, 0, 0},   // backward
        {0, 0, 0, 0, 1, 0, 0},    // fire
        {0, 0, 0, 0, 0, 1, 0},    // jump
        {0, 0, 0, 0, 0, 0, 1}     // crouch
    };

    actions_discrete->resize(N_DISCRETE_ACTIONS);

    for (int i = 0; i < N_DISCRETE_ACTIONS; i++) {
        (*actions_discrete)[i].reset(new int[N_ORIGINAL_ACTIONS]);

        for (int j = 0; j < N_ORIGINAL_ACTIONS; j++) {
            (*actions_discrete)[i][j] = actions_discrete_definition[i][j];
        }
    }
}

}} //  namespace simulator::deepmind_lab_game
