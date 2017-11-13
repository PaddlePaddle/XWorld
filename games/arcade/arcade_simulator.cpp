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

#include <glog/logging.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "arcade.h"

DECLARE_bool(pause_screen);
DEFINE_string(ale_rom, "", "atari rom file");
DEFINE_int32(ale_random_starts,
             30,
             "play the action 0 N times at the beginning"
             " where N is randomly selected between"
             " 1 and random_starts.");

namespace simulator {
namespace arcade_game {

using namespace ale;

ArcadeGame* ArcadeGame::create() {
    LOG(INFO) << FLAGS_ale_rom;
    return new Arcade(FLAGS_ale_rom);
}

void Arcade::show_screen(float reward) {
    cv::Mat img(IMG_HEIGHT, IMG_WIDTH, CV_8UC3);
    for (size_t h = 0, base = 0; h < IMG_HEIGHT; ++h) {
        for (size_t w = 0; w < IMG_WIDTH; ++w, base += 3) {
            cv::Vec3b& color = img.at<cv::Vec3b>(cv::Point(w, h));
            color[0] = screen_ale_[base + 2];
            color[1] = screen_ale_[base + 1];
            color[2] = screen_ale_[base];
        }
    }
    const cv::Mat imgLarge(IMG_HEIGHT * 4, IMG_WIDTH * 4, CV_8UC3);
    cv::resize(img, imgLarge, imgLarge.size());

    // imshow is not thread safe, so we use lock here.
    std::lock_guard<std::mutex> guard(GameSimulator::s_display_mutex_);
    cv::imshow(get_screen_name(), imgLarge);
    if (FLAGS_pause_screen) {
        cv::waitKey(-1);
    } else {
        cv::waitKey(20);
    }
}

Arcade::Arcade(const std::string& ale_rom) : ale_(false) {
    ale_.setFloat("repeat_action_probability", 0.0f);
    ale_.setBool("color_averaging", true);
    ale_.loadROM(ale_rom);
    const ALEScreen& screen = ale_.getScreen();
    CHECK_EQ(screen.height(), IMG_HEIGHT);
    CHECK_EQ(screen.width(), IMG_WIDTH);
    screen_rgb_.resize(IMG_HEIGHT * IMG_WIDTH * 3);
    screen_ale_.resize(IMG_HEIGHT * IMG_WIDTH * 3);
    CHECK_GT(FLAGS_ale_random_starts, 0);
    minimal_actions_ = ale_.getMinimalActionSet();
    VLOG(1) << "number of available actions: " << minimal_actions_.size();
}

void Arcade::reset_game() {
    // If lost all lives, reset the atari game. Otherwise, just initialize
    // screens_.
    if (ale_.game_over()) {
        ale_.reset_game();
    }
    random_start(FLAGS_ale_random_starts);
    lives_ = get_lives();
    GameSimulator::reset_game();
}

int Arcade::game_over() {
    int code = GameSimulator::game_over();
    if (ale_.game_over()) {
        code |= DEAD;
    }
    if (lives_ > get_lives()) {
        code |= LOST_LIFE;
    }
    return code;
}

float Arcade::take_action(const StatePacket& actions) {
    CHECK_EQ(actions.size(), 1);
    int action_id = *(actions.get_buffer("action")->get_id());
    CHECK_LT(static_cast<size_t>(action_id), minimal_actions_.size());
    if (FLAGS_lock_step) {
        int key = cv::waitKey(0) % 256;
        switch (key) {
            case 'h':
                action_id = 0;
                break;
            case 'j':
                action_id = 1;
                break;
            case 'k':
                action_id = 3;
                break;
            case 'l':
                action_id = 2;
                break;
            default:
                break;
        }
    }
    Action a = minimal_actions_[action_id];
    last_action_ = std::to_string(int(a));
    float reward = ale_.act(a);
    return reward;
}

int Arcade::get_num_actions() { return minimal_actions_.size(); }

void Arcade::get_screen_rgb() {
    ale_.getScreenRGB(screen_ale_);
    for (size_t h = 0, base = 0; h < IMG_HEIGHT; ++h) {
        for (size_t w = 0; w < IMG_WIDTH; ++w, base += 3) {
            screen_rgb_[h * IMG_WIDTH + w] = screen_ale_[base];
            screen_rgb_[IMG_HEIGHT * IMG_WIDTH + h * IMG_WIDTH + w] =
                screen_ale_[base + 1];
            screen_rgb_[2 * IMG_HEIGHT * IMG_WIDTH + h * IMG_WIDTH + w] =
                screen_ale_[base + 2];
        }
    }
}

void Arcade::down_sample_image(const GameFrame& screen, GameFrame& screen_out) {
    cv::Mat img(IMG_HEIGHT, IMG_WIDTH, CV_8UC3);
    for (size_t h = 0; h < IMG_HEIGHT; ++h) {
        for (size_t w = 0; w < IMG_WIDTH; ++w) {
            cv::Vec3b& color = img.at<cv::Vec3b>(cv::Point(w, h));
            color[0] = screen[h * IMG_WIDTH + w];
            color[1] = screen[IMG_WIDTH * IMG_HEIGHT + h * IMG_WIDTH + w];
            color[2] = screen[2 * IMG_WIDTH * IMG_HEIGHT + h * IMG_WIDTH + w];
        }
    }
    const int IMG_HEIGHT_TMP = 110, IMG_WIDTH_TMP = 84;
    const int border_y = (IMG_HEIGHT_TMP - IMG_HEIGHT_OUT);
    const int border_x = (IMG_WIDTH_TMP - IMG_WIDTH_OUT) / 2;
    cv::Mat img_tmp(IMG_HEIGHT_TMP, IMG_WIDTH_TMP, CV_8UC3);
    cv::Mat img_out(IMG_HEIGHT_OUT, IMG_WIDTH_OUT, CV_8UC3);
    cv::resize(img, img_tmp, img_tmp.size());
    cv::Mat cropped =
        img_tmp(cv::Rect(border_x, border_y, IMG_HEIGHT_OUT, IMG_WIDTH_OUT));
    cropped.copyTo(img_out);
    if (screen_out.size() != IMG_HEIGHT_OUT * IMG_WIDTH_OUT) {
        screen_out.resize(IMG_HEIGHT_OUT * IMG_WIDTH_OUT);
    }
    for (size_t h = 0; h < IMG_HEIGHT_OUT; ++h) {
        for (size_t w = 0; w < IMG_WIDTH_OUT; ++w) {
            cv::Vec3b& color = img_out.at<cv::Vec3b>(cv::Point(w, h));
            int y = rgb2y(color[0], color[1], color[2]);
            CHECK_GE(y, 0);
            CHECK_LE(y, 255);
            screen_out[h * IMG_WIDTH_OUT + w] = y;
        }
    }
}

int Arcade::get_lives() { return ale_.lives(); }

void Arcade::get_screen(StatePacket& screen) {
    GameFrame screen_vec;
    get_screen_rgb();
    down_sample_image(screen_rgb_, screen_vec);
    screen = StatePacket();
    screen.add_buffer_value("screen", screen_vec);
}

void Arcade::define_state_specs(StatePacket& state) {
    state = StatePacket();
    state.add_key("reward");
    state.add_key("screen");
}

void Arcade::get_screen_out_dimensions(size_t& height,
                                       size_t& width,
                                       size_t& channels) {
    height = IMG_HEIGHT_OUT;
    width = IMG_WIDTH_OUT;
    channels = 1;  // hardcoded grayscale
}
}
}  //  namespace simulator::arcade_game
