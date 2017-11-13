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

#include <iomanip>

#include "xworld3d.h"
#include "xworld3d_flags.h"
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

    void step(const int frame_skip);

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
        legal_actions_({MOVE_FORWARD, MOVE_BACKWARD, TURN_LEFT, TURN_RIGHT, JUMP, COLLECT}),
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

inline void X3SimulatorImpl::step(const int frame_skip) {
    xworld3d_.step(frame_skip);
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

X3Simulator::X3Simulator(bool print, bool big_screen) :
        height_(0), width_(0),
        img_height_out_(FLAGS_x3_training_img_height),
        img_width_out_(FLAGS_x3_training_img_width) {
    impl_ = util::make_unique<X3SimulatorImpl>(
            FLAGS_x3_conf, print, big_screen);
}

X3Simulator::~X3Simulator() {}

void X3Simulator::reset_game() {
    TeachingEnvironment::reset_game();

    impl_->reset_game();
    height_ = impl_->height();
    width_ = impl_->width();
    // default
    if (max_steps_ == 0) {
        if (FLAGS_x3_task_mode == "arxiv_lang_acquisition") {
            max_steps_ = (height_ + width_ ) * 2;
        } else {
            max_steps_ = height_ * width_ * 2;
        }
    }

    history_messages_.push_back("--------------- New Game --------------");
    if (history_messages_.size() > n_history_) {
        history_messages_.pop_front();
    }
}

int X3Simulator::game_over() {
    if (FLAGS_x3_task_mode == "arxiv_lang_acquisition") {
        // Each session has a navigation task, during which some questions are
        // asked
        // The answer is appended after each question
        auto event = get_event_from_buffer();
        if (event == "correct_goal") {
            return SUCCESS;
        }
    } else if (FLAGS_x3_task_mode == "arxiv_interactive") {
        // Each session has a language task; there is no navigation
        auto event = get_event_from_buffer();
        if (event == "correct_reply") {
            return SUCCESS;
        } else if (event == "wrong_reply") {
            return DEAD;
        }
    } else if (FLAGS_x3_task_mode == "one_channel") {
        // Each session has all tasks until the max steps
    } else {
        LOG(FATAL) << "unsupported task mode: " << FLAGS_x3_task_mode;
    }
    return ALIVE;
}

int X3Simulator::get_num_actions() {
    return impl_->get_num_actions();
}

std::string X3Simulator::conf_file() {
    return FLAGS_x3_conf;
}

void X3Simulator::show_screen(float reward) {
    cv::Mat img;
    impl_->get_screen_rgb(active_agent_id_, img, false);

    cv::Mat img_wo_msg = concat_images(img, get_reward_image(reward), true);
    prev_screen_ = img_wo_msg;
    screen_ = concat_images(img_wo_msg,
                            get_message_image(history_messages_),
                            false);

    cv::namedWindow("XWorld3D");
    // screen_ must still be valid after the end of this function
    // so that it can be saved in save_screen
    cv::setMouseCallback("XWorld3D", util::save_screen, &screen_);
    cv::imshow("XWorld3D", screen_);
    cv::waitKey(200);
}

void X3Simulator::define_state_specs(StatePacket& state) {
    state = StatePacket();
    state.add_key("reward");
    state.add_key("screen");
    state.add_key("sentence");
    // set the teacher's sentence
    state.get_buffer("sentence")->set_str(
            agent_received_sentences_[active_agent_id_]);
}

void X3Simulator::get_extra_info(
        std::unordered_map<std::string, std::string>& info) {
    info.clear();
    auto type =
        get_teacher_sent_type_from_buffer();  // task type related to a sentence
    auto event = get_event_from_buffer();     // current event happending in env
    info["task"] = type;
    info["event"] = event;
}

void X3Simulator::get_world_dimensions(double& X, double& Y, double& Z) {
    X = width_;
    Y = height_;
    Z = 0.0;
}

void X3Simulator::get_screen_out_dimensions(size_t& height,
                                            size_t& width,
                                            size_t& channels) {
    height = img_height_out_;
    width = img_width_out_;
    channels = (FLAGS_color ? 3 : 1);
}

int X3Simulator::add_agent() {
    agent_received_sentences_.push_back("");
    return GameSimulatorMulti::add_agent();
}

void X3Simulator::apply_teacher_actions() {
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

void X3Simulator::get_all_entities(std::vector<Entity>& entities) {
    impl_->get_all_entities(entities);
}

boost::python::object X3Simulator::get_py_env() {
    return impl_->get_py_env();
}

void X3Simulator::update_environment() {
    impl_->update_environment();
}

float X3Simulator::take_action(const StatePacket& actions) {
    TeachingEnvironment::take_action(actions);
    last_action_ = "";

    int key = -1;
    // show screen
    if (FLAGS_pause_screen) {
        // The screen will pause at every step waiting for keyboard
        key = cv::waitKey(-1) % 256;
        // for some reason, ctrl+C in the terminal won't kill the program
        if (key == 27) {
            // ESC
            exit(EXIT_SUCCESS);
        }
    }

    //// speak
    if (FLAGS_x3_task_mode == "arxiv_interactive" ||
        FLAGS_x3_task_mode == "one_channel") {
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
    if (FLAGS_x3_task_mode == "arxiv_lang_acquisition" ||
        FLAGS_x3_task_mode == "one_channel") {
        CHECK(actions.contain_key("action"))
            << "The agent has to take the move action.";
        int action_idx = *(actions.get_buffer("action")->get_id());
        if (FLAGS_pause_screen) {
            switch (key) {
                case 'w':
                    action_idx = 0;
                    break;
                case 's':
                    action_idx = 1;
                    break;
                case 'a':
                    action_idx = 2;
                    break;
                case 'd':
                    action_idx = 3;
                    break;
                case 'j':
                    action_idx = 4;
                    break;
                case 'c':
                    action_idx = 5;
                default:
                    break;
            }
        }
        CHECK_LT(action_idx, get_num_actions());
        // take one step in the game
        last_action_success_ = impl_->act(active_agent_id_, action_idx);
        last_action_ += std::to_string(action_idx);
        record_agent_action_successful_in_buffer(last_action_success_);
    }
    impl_->step(FLAGS_x3_frame_skip);
    return 0;  // xworld rewards are given by the teacher
}

inline int X3Simulator::get_lives() {
    return game_over() | DEAD ? 0 : 1;
}

void X3Simulator::get_screen(StatePacket& screen) {
    GameFrame screen_vec;
    cv::Mat img;
    impl_->get_screen_rgb(active_agent_id_, img, false);
    resize_image_to_frame(img, screen_vec,
                          img_height_out_, img_width_out_,
                          FLAGS_color);
    screen = StatePacket();
    screen.add_buffer_value("screen", screen_vec);
}

cv::Mat X3Simulator::get_reward_image(float reward) {
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

cv::Mat X3Simulator::get_message_image(std::deque<std::string>& messages) {
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

    // TODO: task type
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

void X3Simulator::update_message_box_on_screen() {
    if (!prev_screen_.empty()) {
        cv::Mat img_all = concat_images(
            prev_screen_, get_message_image(history_messages_), false);
        cv::waitKey(1);
        cv::imshow("XWorld3D", img_all);
    }
}

void X3Simulator::resize_image_to_frame(const cv::Mat& img,
                                        GameFrame& frame,
                                        size_t img_height_out,
                                        size_t img_width_out,
                                        bool color) {
    cv::Mat img_out(img_height_out, img_width_out, CV_8UC3);
    cv::resize(img, img_out, img_out.size(), cv::INTER_LINEAR);

    size_t n_channels = (color ? 3 : 1);
    if (frame.size() != img_height_out * img_width_out * n_channels) {
        frame.resize(img_height_out * img_width_out * n_channels);
    }

    if (!color) {
        cv::cvtColor(img_out, img_out, cv::COLOR_BGR2GRAY);
    }

    for (size_t h = 0; h < img_height_out; ++h) {
        for (size_t w = 0; w < img_width_out; ++w) {
            for (size_t c = 0; c < n_channels; c++) {
                // normalize the pixel vals to [0,1]
                size_t offset = c * img_width_out * img_height_out +
                             h * img_width_out + w;
                frame[offset] =
                    (color ? img_out.at<cv::Vec3b>(cv::Point(w, h)).val[c]
                           : img_out.at<uchar>(cv::Point(w, h)));
            }
        }
    }
}

cv::Mat X3Simulator::concat_images(cv::Mat img1,
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

}} // namespace simulator::xworld3d
