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

#include "xworld_simulator.h"
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <algorithm>
#include <functional>
#include "xworld/xworld.h"

DEFINE_string(
    xwd_conf_path,
    "./xworld/confs/navigation.json",
    "the map and task configure file");
DEFINE_int32(
    visible_radius_unit,
    0,
    "the radius of visible range in terms of unit (0 for fully observe)");
DECLARE_int32(max_steps);
DECLARE_bool(pause_screen);
DECLARE_bool(color);
DEFINE_bool(ego_centric, true, "whether have ego-centric view");
DEFINE_string(
    task_mode,
    "one_channel",
    "arxiv_lang_acquisition|arxiv_interactive|one_channel");

namespace simulator {
namespace xwd {

XWorldSimulator::XWorldSimulator(bool print_xworld_config) :
        xworld_(FLAGS_xwd_conf_path, print_xworld_config) {
    init();
}

void XWorldSimulator::init() {
    height_ = xworld_.height();
    width_ = xworld_.width();
    img_height_ = height_ * XItem::item_size_;
    img_width_ = width_ * XItem::item_size_;
    if (FLAGS_ego_centric) {
        img_height_out_ = (height_ * 2 - 1) * block_size_;
        img_width_out_ = (width_ * 2 - 1) * block_size_;
    } else {
        img_height_out_ = height_ * block_size_;
        img_width_out_ = width_ * block_size_;
    }
    // default
    if (FLAGS_task_mode == "arxiv_lang_acquisition") {
        FLAGS_max_steps = (height_ + width_ ) * 2;
    } else {
        FLAGS_max_steps = height_ * width_ * 2;
    }
}

int XWorldSimulator::add_agent() {
    agent_received_sentences_.push_back("");
    return GameSimulatorMulti::add_agent();
}

void XWorldSimulator::apply_teacher_actions() {
    auto sentence = get_teacher_sent_from_buffer();
    auto type = get_teacher_sent_type_from_buffer();
    // change empty sentence to "-"
    if (sentence.empty()) {
        sentence = "-";
        type = "Silence";
    }
    agent_received_sentences_[active_agent_id_] = sentence;
    auto message = "[" + type + "] Teacher: " + sentence;
    history_messages_.push_back(message);
    if (history_messages_.size() > n_history_) {
        history_messages_.pop_front();
    }
}

void XWorldSimulator::get_world_dimensions(double& X, double& Y, double& Z) {
    X = width_;
    Y = height_;
    Z = 0;
}

void XWorldSimulator::get_screen_out_dimensions(size_t& img_height_out,
                                                size_t& img_width_out,
                                                size_t& channels) {
    img_height_out = img_height_out_;
    img_width_out = img_width_out_;
    channels = (FLAGS_color ? 3 : 1);
}

void XWorldSimulator::get_all_entities(std::vector<Entity>& entities) {
    xworld_.get_entities(entities);
}

boost::python::object XWorldSimulator::get_py_env() {
    return xworld_.get_py_env();
}

void XWorldSimulator::reset_game() {
    TeachingEnvironment::reset_game();
    xworld_.reset();
    init();  // update dimensions
    history_messages_.clear();
    history_messages_.push_back("--------------- New Game --------------");
    if (history_messages_.size() > n_history_) {
        history_messages_.pop_front();
    }
}

void XWorldSimulator::update_environment() {
    // do not reset the map, only do a minor update
    // according to the teacher
    xworld_.reset(false);
}

int XWorldSimulator::game_over() {
    if (FLAGS_task_mode == "arxiv_lang_acquisition") {
        // Each session has a navigation task, during which some questions are
        // asked
        // The answer is appended after each question
        auto event = get_event_from_buffer();
        if (event == "correct_goal") {
            return SUCCESS;
        }
    } else if (FLAGS_task_mode == "arxiv_interactive") {
        // Each session has a language task; there is no navigation
        auto event = get_event_from_buffer();
        if (event == "correct_reply") {
            return SUCCESS;
        } else if (event == "wrong_reply") {
            return DEAD;
        }
    } else if (FLAGS_task_mode == "one_channel") {
        // Each session has all tasks until the max steps
    } else {
        LOG(FATAL) << "unsupported task mode: " << FLAGS_task_mode;
    }
    return ALIVE;
}

float XWorldSimulator::take_action(const StatePacket& actions) {
    TeachingEnvironment::take_action(actions);
    last_action_ = "";

    //// speak
    if (FLAGS_task_mode == "arxiv_interactive" ||
        FLAGS_task_mode == "one_channel") {
        CHECK(actions.contain_key("pred_sentence"))
            << "The agent has to take the speak action.";
        std::string agent_sent =
            *(actions.get_buffer("pred_sentence")->get_str());
        record_agent_sent_in_buffer(agent_sent);
        last_action_ += "speak(" + agent_sent + ")";
        // update message box
        history_messages_.push_back("[Reply] Learner: " +
                                    agent_sent);  // add token
        update_message_box_on_screen();
    }

    //// move
    if (FLAGS_task_mode == "arxiv_lang_acquisition" ||
        FLAGS_task_mode == "one_channel") {
        CHECK(actions.contain_key("action"))
            << "The agent has to take the move action.";
        int action_idx = *(actions.get_buffer("action")->get_id());
        CHECK_LT(action_idx, get_num_actions());
        // take one step in the game
        last_action_success_ = xworld_.act(active_agent_id_, action_idx);
        last_action_ += std::to_string(action_idx);
        record_agent_action_successful_in_buffer(last_action_success_);
    }
    return 0;  // xworld rewards are given by the teacher
}

// interface to get_state_data
std::string XWorldSimulator::get_teacher_sentence_for_agent() {
    return agent_received_sentences_[active_agent_id_];
}

int XWorldSimulator::get_num_actions() { return xworld_.get_num_actions(); }

void XWorldSimulator::get_screen(StatePacket& screen) {
    GameFrame screen_rgb;
    GameFrame screen_vec;
    get_screen_rgb(screen_rgb);
    down_sample_image(screen_rgb, screen_vec, FLAGS_color);
    screen = StatePacket();
    screen.add_buffer_value("screen", screen_vec);
}

void XWorldSimulator::get_screen_rgb(GameFrame& rgbs) {
    int pad_size = 0;
    // ego-centric view
    cv::Mat screen = xworld_.to_image(
        FLAGS_ego_centric,
        active_agent_id_,
        pad_size,
        /* flag_illustration= */ false,
        /* success= */ 0,
        FLAGS_visible_radius_unit,
        /*flag_crop_receiptive_field*/ true);

    cv::Mat screen_resized(img_height_, img_width_, CV_8UC3);
    cv::resize(screen, screen_resized, screen_resized.size(), cv::INTER_LINEAR);
    screen = screen_resized;
    unsigned int h = img_height_;
    unsigned int w = img_width_;
    rgbs.resize(3 * w * h, 0);
    for (size_t i = 0; i < h; ++i) {
        for (size_t j = 0; j < w; ++j) {
            cv::Vec3b& color = screen.at<cv::Vec3b>(cv::Point(j, i));
            rgbs[i * w + j] = color[0];
            rgbs[w * h + i * w + j] = color[1];
            rgbs[2 * w * h + i * w + j] = color[2];
        }
    }
}

cv::Mat XWorldSimulator::concat_images(cv::Mat img1,
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

cv::Mat XWorldSimulator::get_reward_image(float reward) {
    cv::Mat canvas(200, 200, CV_8UC3, cv::Scalar(0, 0, 0));
    std::stringstream stream;
    std::string sign = reward >= 0 ? "+" : "-";
    stream << std::fixed << std::setprecision(2) << std::fabs(reward);
    cv::putText(canvas,
                sign + stream.str(),
                cv::Point(20, 30),
                cv::FONT_HERSHEY_SIMPLEX,
                1,
                cv::Scalar(255, 255, 255),
                2,
                CV_AA);
    return canvas;
}

cv::Mat XWorldSimulator::get_message_image(std::deque<std::string>& messages) {
    if (messages.size() > n_history_) {
        messages.pop_front();
    }
    cv::Scalar white(200, 200, 200);
    cv::Scalar black(0, 0, 0);
    cv::Scalar green(0, 255, 0);
    cv::Scalar red(0, 0, 255);
    cv::Scalar yellow(0, 255, 255);
    cv::Scalar blue(255, 0, 0);
    cv::Scalar magenta(255, 0, 255);
    cv::Scalar cyan(255, 255, 0);
    cv::Scalar pink(255, 200, 200);

    cv::Mat message_image(500, 1000, CV_8UC3, black);
    int line_height = message_image.rows / n_history_;

    auto get_message_color = [&](std::string type) {
        if (type == "Silence" || type == "Reply") {
            return black;
        } else if (type.find("XWorldNav") == 0) {
            return green;
        } else if (type == "XWorldRecColorToObject" ||
                   type == "XWorldRecObjectToColor") {
            return red;
        } else if (type == "XWorldRecDirectionToObject" ||
                   type == "XWorldRecObjectToDirection") {
            return yellow;
        } else if (type == "XWorldRecDirectionToColor" ||
                   type == "XWorldRecColorToDirection") {
            return blue;
        } else if (type == "XWorldRecColorAndObject") {
            return magenta;
        } else if (type.find("XWorldRecDirectionAndObject") == 0) {
            return cyan;
        } else if (type.find("XWorldRecBetween") == 0) {
            return pink;
        } else if (type.find("XWorldLan") == 0) {
            return white;
        } else {
            LOG(FATAL) << "unrecognized message type: " + type;
        }
        return white;
    };

    for (size_t i = 1; i <= history_messages_.size(); i++) {
        if (history_messages_[i - 1][0] == '[') {
            int idx = history_messages_[i - 1].find(']');
            std::string type = history_messages_[i - 1].substr(1, idx - 1);
            std::string content = history_messages_[i - 1].substr(idx + 1);
            auto color = get_message_color(type);
            // message type
            cv::putText(message_image,
                        type + ":",
                        cv::Point(10, i * line_height - 5),
                        cv::FONT_HERSHEY_SIMPLEX,
                        0.4,
                        color,
                        1,
                        CV_AA);
            // message content
            cv::putText(message_image,
                        content,
                        cv::Point(300, i * line_height - 5),
                        cv::FONT_HERSHEY_SIMPLEX,
                        0.4,
                        white,
                        1,
                        CV_AA);
        } else {
            cv::putText(message_image,
                        history_messages_[i - 1],
                        cv::Point(10, i * line_height - 5),
                        cv::FONT_HERSHEY_SIMPLEX,
                        0.4,
                        white,
                        1,
                        CV_AA);
        }
    }

    return message_image;
}

void XWorldSimulator::update_message_box_on_screen() {
    if (!prev_screen_.empty()) {
        cv::Mat img_all = concat_images(
            prev_screen_, get_message_image(history_messages_), false);
        cv::waitKey(1);
        cv::imshow("XWorld Game", img_all);
    }
}

void XWorldSimulator::show_screen(float reward) {
    int pad_size = 0;
    int success = 0;
    // non-ego centric
    cv::Mat img =
        xworld_.to_image(false,
                         active_agent_id_,
                         pad_size,
                         true,
                         success,
                         0,
                         false);
    if (FLAGS_visible_radius_unit > 0) {
        cv::Mat img_partial =
            xworld_.to_image(false,
                             active_agent_id_,
                             pad_size,
                             true,
                             success,
                             FLAGS_visible_radius_unit,
                             false);
        float alpha = 0.5;
        float beta = (1.0 - alpha);
        cv::addWeighted(img, alpha, img_partial, beta, 0.0, img);
    }

    // ego-centric
    const cv::Mat img_ego =
        xworld_.to_image(true,
                         active_agent_id_,
                         pad_size,
                         true,
                         success,
                         FLAGS_visible_radius_unit,
                         false);
    cv::Mat ego_img(
        int(floor(img.rows / 2.0)), int(floor(img.cols / 2.0)), CV_8UC3);
    cv::resize(img_ego, ego_img, ego_img.size(), cv::INTER_LINEAR);

    cv::Mat img_wo_msg = concat_images(img, get_reward_image(reward), true);
    prev_screen_ = img_wo_msg;
    screen_ =
        concat_images(img_wo_msg, get_message_image(history_messages_), false);

    // show screen
    if (FLAGS_pause_screen) {
        // The screen will pause at every step waiting for keyboard
        cv::waitKey(-1);
    } else {
        // Default mode: the screen will display continuously
        cv::waitKey(success ? 1000 : 200);
    }
    cv::namedWindow("XWorld Game");
    // screen_ must still be valid after the end of this function
    // so that it can be saved in save_screen
    cv::setMouseCallback("XWorld Game", util::save_screen, &screen_);
    cv::imshow("XWorld Game", screen_);
}

void XWorldSimulator::define_state_specs(StatePacket& state) {
    state = StatePacket();
    state.add_key("reward");
    state.add_key("screen");
    state.add_key("sentence");
    // set the teacher's sentence
    state.get_buffer("sentence")->set_str(get_teacher_sentence_for_agent());
}

void XWorldSimulator::get_extra_info(
        std::unordered_map<std::string, std::string>& info) {
    info.clear();
    auto type =
        get_teacher_sent_type_from_buffer();  // task type related to a sentence
    auto event = get_event_from_buffer();     // current event happending in env
    info["task"] = type;
    info["event"] = event;
}

int XWorldSimulator::get_lives() { return game_over() ? 0 : 1; }

void XWorldSimulator::down_sample_image(const GameFrame& screen,
                                        GameFrame& screen_out,
                                        bool color /* =false */) {
    cv::Mat img(img_height_, img_width_, CV_8UC3);
    for (int h = 0; h < img_height_; ++h) {
        for (int w = 0; w < img_width_; ++w) {
            cv::Vec3b& color = img.at<cv::Vec3b>(cv::Point(w, h));
            color[0] = screen[h * img_width_ + w];
            color[1] = screen[img_width_ * img_height_ + h * img_width_ + w];
            color[2] =
                screen[2 * img_width_ * img_height_ + h * img_width_ + w];
        }
    }
    cv::Mat img_out(img_height_out_, img_width_out_, CV_8UC3);
    cv::resize(img, img_out, img_out.size(), cv::INTER_LINEAR);

    size_t n_channels = (color ? 3 : 1);
    if (screen_out.size() != img_height_out_ * img_width_out_ * n_channels) {
        screen_out.resize(img_height_out_ * img_width_out_ * n_channels);
    }

    if (!color) {
        cv::cvtColor(img_out, img_out, cv::COLOR_BGR2GRAY);
    }

    for (int h = 0; h < img_height_out_; ++h) {
        for (int w = 0; w < img_width_out_; ++w) {
            for (size_t c = 0; c < n_channels; c++) {
                // normalize the pixel vals to [0,1]
                int offset = c * img_width_out_ * img_height_out_ +
                             h * img_width_out_ + w;
                screen_out[offset] =
                    (color ? img_out.at<cv::Vec3b>(cv::Point(w, h)).val[c]
                           : img_out.at<uchar>(cv::Point(w, h)));
            }
        }
    }
}
}
} /* namespace simulator::xwd */
