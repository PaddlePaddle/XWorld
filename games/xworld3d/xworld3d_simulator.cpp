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

    void reset_game(bool map_reset);

    // called by AgentSpecificSimulator
    void define_state_specs(StatePacket& state);

    void get_world_dimensions(double& X, double& Y, double& Z);

    void get_all_entities(std::vector<Entity>& entities);

    std::vector<X3ItemPtr> get_agents() { return xworld3d_.get_agents(); }

    boost::python::object get_py_env();

    size_t height() const { return height_; }

    size_t width() const { return width_; }

    void update_environment();

    bool act(const size_t agent_id, const size_t action);

    void step(const int frame_skip);

    void get_screen_rgb(const size_t agent_id,
                        cv::Mat& img,
                        bool bird_view = false);  // get the current screenshot (color)

    std::set<std::string> contact_list(const size_t agent_id);

private:
    X3SimulatorImpl(const X3SimulatorImpl&) = delete;

    X3World xworld3d_;       // the environment for all the agents
    size_t height_;          // unit: grid
    size_t width_;           // unit: grid
};

X3SimulatorImpl::X3SimulatorImpl(const std::string& conf,
                                 bool print,
                                 bool big_screen) :
        xworld3d_(conf, print, big_screen),
        height_(0), width_(0) {}

void X3SimulatorImpl::reset_game(bool map_reset) {
    xworld3d_.reset_world(map_reset);

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

inline bool X3SimulatorImpl::act(const size_t agent_id, const size_t action) {
    return xworld3d_.act(agent_id, action);
}

inline void X3SimulatorImpl::step(const int frame_skip) {
    xworld3d_.step(frame_skip);
}

void X3SimulatorImpl::get_screen_rgb(
        const size_t agent_id, cv::Mat& screen, bool bird_view) {
    roboschool::RenderResult render_result =
            xworld3d_.render(agent_id, bird_view);
    std::string render_str = std::get<0>(render_result);
    int img_height = std::get<4>(render_result);
    int img_width = std::get<5>(render_result);

    screen.create(img_height, img_width, CV_8UC3);
    // TODO: get rid of for loop
    for (int h = 0, p = 0; h < img_height; ++h) {
        for (int w = 0; w < img_width; ++w, p += 3) {
            cv::Vec3b& color = screen.at<cv::Vec3b>(cv::Point(w, h));
            color[0] = render_str[p + 2];
            color[1] = render_str[p + 1];
            color[2] = render_str[p];
        }
    }
    // render agent's direction of view under bird_view
    if (bird_view) {
        auto agent_ptr = xworld3d_.get_agent(agent_id);
        double x;
        double y;
        agent_ptr->get_direction(x, y);
        cv::Point pt1(25, 25);
        cv::Point pt2(25 - 20 * y, 25 - 20 * x);
        cv::Scalar colorScalar = cv::Scalar( 0, 200, 0);
        cv::arrowedLine(screen, pt1, pt2, colorScalar, 2, 8, 0, 0.5);
    }
}

std::set<std::string> X3SimulatorImpl::contact_list(const size_t agent_id) {
    return xworld3d_.contact_list(xworld3d_.get_agent(agent_id));
}

X3Simulator::X3Simulator(bool print, bool big_screen) :
        legal_actions_({MOVE_FORWARD, MOVE_BACKWARD, MOVE_LEFT, MOVE_RIGHT,
                        TURN_LEFT, TURN_RIGHT}),
        height_(0), width_(0),
        img_height_out_(FLAGS_x3_training_img_height),
        img_width_out_(FLAGS_x3_training_img_width),
        bird_view_(false),
        agent_received_sentences_(0),
        agent_prev_actions_(0),
        keyboard_action_(-1) {
    impl_ = util::make_unique<X3SimulatorImpl>(
            FLAGS_x3_conf, print, big_screen);
}

X3Simulator::~X3Simulator() {}

void X3Simulator::reset_game() {
    TeachingEnvironment::reset_game();

    // only change the map when the agent succeeds
    // impl_->reset_game(
    //     history_messages_.size() == 0  // the very beginning of simulation; we must reset
    //     || game_over() == SUCCESS);
    impl_->reset_game(true);
    height_ = impl_->height();
    width_ = impl_->width();
    game_events_ = "";

    history_messages_.clear();
    history_messages_.push_back("--------------- New Game --------------");
    if (history_messages_.size() > n_history_) {
        history_messages_.pop_front();
    }
}

int X3Simulator::game_over() {
    auto event = get_event_from_buffer();
    if (event.find("correct") != std::string::npos) {
        return SUCCESS;
    } else if (event.find("wrong") != std::string::npos) {
        return DEAD;
    } else if (event == "time_up") {
        return MAX_STEP;
    }
    CHECK(event == "") << "Unrecognized event: " << event;
    return ALIVE;
}

// interface to get_state_data
std::string X3Simulator::get_teacher_sentence_for_agent() {
    std::string sent = agent_received_sentences_[active_agent_id_];
    if (sent.empty()) {
        sent = "-";
    }
    return sent;
}

int X3Simulator::get_num_actions() {
    return legal_actions_.size();
}

std::string X3Simulator::conf_file() {
    return FLAGS_x3_conf;
}

void X3Simulator::show_screen(float reward) {
    cv::Mat img;
    impl_->get_screen_rgb(active_agent_id_, img, bird_view_);

    cv::Mat command_img = get_command_image(history_messages_.back());
    cv::Mat reward_img = get_reward_image(reward);
    cv::Mat img_wo_msg = concat_images(command_img,
                                       concat_images(img, reward_img, true),
                                       true);

    prev_screen_ = img_wo_msg;
    screen_ = concat_images(img_wo_msg,
                            get_message_image(history_messages_),
                            false);

    cv::namedWindow("XWorld3D");
    // screen_ must still be valid after the end of this function
    // so that it can be saved in save_screen
    cv::setMouseCallback("XWorld3D", util::save_screen, &screen_);
    cv::imshow("XWorld3D", screen_);
    if (FLAGS_pause_screen) {
        // The screen will pause at every step waiting for keyboard
        keyboard_action_ = cv::waitKey(-1) % 256;
    } else {
        cv::waitKey(100);
    }
}

void X3Simulator::define_state_specs(StatePacket& state) {
    state = StatePacket();
    state.add_key("reward");
    state.add_key("screen");
    state.add_key("sentence");
    // set the teacher's sentence
    state.get_buffer("sentence")->set_str(get_teacher_sentence_for_agent());
}

void X3Simulator::get_extra_info(std::string& info) {
    info = std::to_string(::getpid()) + "|";
    auto type =
        get_teacher_sent_type_from_buffer();  // task type related to a sentence
    auto event = get_event_from_buffer();     // current event happending in env
    info += "task:" + type + ",";
    info += "event:" + event + ",";
    info += "height:" + std::to_string(height_) + ",";
    info += "width:" + std::to_string(width_);
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
    agent_prev_actions_.push_back(X3NavAction::NOOP);
    return GameSimulatorMulti::add_agent();
}

void X3Simulator::apply_teacher_actions() {
    auto sentence = get_teacher_sent_from_buffer();
    auto type = get_teacher_sent_type_from_buffer();
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

std::string X3Simulator::get_events_of_game() {
    auto ret = game_events_;
    game_events_ = "";
    return ret;
}

void X3Simulator::record_collision_events(
        const std::set<std::string>& collision_list) {
    if (!collision_list.empty()) {
        game_events_ += "collision:";
        for (auto& s : collision_list) {
            if (game_events_.back() == ':') {
                game_events_ += s;
            } else {
                game_events_ += "|" + s;
            }
        }
        game_events_ += "\n";
    }
}

boost::python::object X3Simulator::get_py_env() {
    return impl_->get_py_env();
}

void X3Simulator::update_environment() {
    impl_->update_environment();
}

float X3Simulator::take_action(const StatePacket& actions) {
    TeachingEnvironment::take_action();
    last_action_ = "";

    if (keyboard_action_ == 27) {
        exit(EXIT_SUCCESS);
    }

    //// speak
    if (FLAGS_x3_task_mode == "arxiv_interactive") {
        CHECK(actions.contain_key("pred_sentence"))
                << "The agent has to take the speak action.";
        std::string agent_sent =
                *(actions.get_buffer("pred_sentence")->get_str());
        record_agent_sent_in_buffer(agent_sent);
        last_action_ += "speak(" + agent_sent + ")";
        // update message box
        history_messages_.push_back("[Reply] Learner: " + agent_sent);  // add token
        update_message_box_on_screen();
        switch (keyboard_action_) {
            case 'z':
                bird_view_ = !bird_view_;
                break;
            default:
                break;
        }
    }

    //// move
    if (FLAGS_x3_task_mode == "arxiv_lang_acquisition" ||
        FLAGS_x3_task_mode == "one_channel") {
        CHECK(actions.contain_key("action"))
            << "The agent has to take the move action.";
        size_t action_idx = *(actions.get_buffer("action")->get_id());
        CHECK_LT(action_idx, get_num_actions());
        auto action = legal_actions_[action_idx];
        switch (keyboard_action_) {
            case 'w':
                action = X3NavAction::MOVE_FORWARD;
                break;
            case 's':
                action = X3NavAction::MOVE_BACKWARD;
                //                    action = X3NavAction::STOP;
                break;
            case 'a':
                action = X3NavAction::MOVE_LEFT;
                break;
            case 'd':
                action = X3NavAction::MOVE_RIGHT;
                break;
            case 'q':
                action = X3NavAction::TURN_LEFT;
                break;
            case 'e':
                action = X3NavAction::TURN_RIGHT;
                break;
            case 'j':
                action = X3NavAction::JUMP;
                break;
            case 'c':
                action = X3NavAction::COLLECT;
                break;
            case 'z':
                bird_view_ = !bird_view_;
                break;
            default:
                //                    action = X3NavAction::NOOP;
                break;
        }
        CHECK(std::find(legal_actions_.begin(), legal_actions_.end(), action)
              != legal_actions_.end()) << "action invalid!";

        ////// This block of code simulates uninterupted actions ///////
        // auto& prev_action = agent_prev_actions_[active_agent_id_];
        // if (action == X3NavAction::STOP) {
        //     action = X3NavAction::NOOP;
        // } else if (prev_action != X3NavAction::NOOP) {
        //     action = prev_action;
        // }
        // prev_action = action;
        ////////////////////////////////////////////////////////////////

        // take one step in the game
        last_action_success_ = impl_->act(active_agent_id_, action);
        last_action_ += std::to_string(action);
        record_agent_action_successful_in_buffer(last_action_success_);
    }
    impl_->step(1);
    auto s = impl_->contact_list(active_agent_id_);
    record_collision_events(s);

    return 0;  // xworld rewards are given by the teacher
}

inline int X3Simulator::get_lives() {
    return (game_over() & DEAD) ? 0 : 1;
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

cv::Mat X3Simulator::get_command_image(const std::string& cmd) {
    cv::Mat canvas(50, 500, CV_8UC3, cv::Scalar(0, 0, 0));
    auto content = cmd.substr(cmd.find("]") + 1); // remove task type
    cv::putText(canvas,
                content.substr(0, content.find(":") + 1),
                cv::Point(10, 20),
                cv::FONT_HERSHEY_SIMPLEX,
                0.5,
                cv::Scalar(150, 150, 150),
                1,
                CV_AA);
    cv::putText(canvas,
                content.substr(content.find(":") + 1),
                cv::Point(10, 40),
                cv::FONT_HERSHEY_SIMPLEX,
                0.4,
                cv::Scalar(255, 255, 255),
                1,
                CV_AA);
    return canvas;
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
        } else if (type.find("XWorld3DNav") == 0) {
            return green;
        } else if (type == "XWorld3DRecColorToObject" ||
                   type == "XWorld3DRecObjectToColor") {
            return red;
        } else if (type == "XWorld3DRecDirectionToObject" ||
                   type == "XWorld3DRecObjectToDirection") {
            return yellow;
        } else if (type == "XWorld3DRecDirectionToColor" ||
                   type == "XWorld3DRecColorToDirection") {
            return blue;
        } else if (type == "XWorld3DRecColorAndObject") {
            return magenta;
        } else if (type.find("XWorld3DRecDirectionAndObject") == 0) {
            return cyan;
        } else if (type.find("XWorld3DRecBetween") == 0) {
            return pink;
        } else if (type.find("XWorld3DLan") == 0) {
            return white;
        } else if (type.find("XWorld3DDia") == 0) {
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
