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

#include "simple_race_simulator.h"

DEFINE_string(track_type, "straight", "track type");
DEFINE_double(track_width, 20.0f, "width of the track");
DEFINE_double(track_length, 100.0f, "length of the track [straight]");
DEFINE_double(track_radius, 30.0f, "radius of the track [circle]");
DEFINE_bool(race_full_manouver,
            false,
            "whether enable full manouver or only turns");
DEFINE_bool(random, false, "randomly set up race");
DEFINE_string(difficulty, "easy", "difficulty level [easy | hard]");
DEFINE_double(reward_scale, 1.0, "actual_reward = reward * reward_scale");
DECLARE_bool(pause_screen);

namespace simulator {
namespace simple_race {

using cv::Mat;
using cv::Point2f;
using cv::Scalar;

const int WINDOW_WIDTH = 480;
const int WINDOW_HEIGHT = 720;
const Scalar lane_color = Scalar(105,105,105);
const Scalar road_base_color = Scalar(19,69,139);

template <typename T>
std::ostream &operator<<(std::ostream &os, const std::vector<T> vec) {
    os << "( ";
    for (size_t i = 0; i < vec.size(); ++i) os << vec[i] << " ";
    os << ")";
    return os;
}

// -------------------------------- Track --------------------------------
CircleTrack::CircleTrack() { CircleTrack(0.0f, 0.0f, 100.0f, 20.0f); }

CircleTrack::CircleTrack(float cx, float cy, float inner_radius, float width)
    : _center(cx, cy), _inner_radius(inner_radius) {
    _width = width;
    _outer_radius = inner_radius + _width;
}

CircleTrack::CircleTrack(const CircleTrack &ct) {
    this->_center = ct._center;
    this->_inner_radius = ct._inner_radius;
    this->_outer_radius = ct._outer_radius;
    this->_width = ct._width;
}

void CircleTrack::draw(Mat img) {
    cv::circle(img, _center, _outer_radius, lane_color, -1, 8);
    cv::circle(img, _center, _inner_radius, road_base_color, -1, 8);
    cv::circle(img, _center, _inner_radius, Scalar(250,250,250), 1, 8);
    cv::circle(img, _center, _outer_radius, Scalar(250,250,250), 1, 8);
}

bool CircleTrack::out_of_bound(const Point2f &pos) {
    float r = cv::norm(pos - _center);

    return r < _inner_radius || r > _outer_radius;
}

Point2f CircleTrack::get_start_pos(bool random) {
    if (!random) {
        Point2f p(_inner_radius + _width / 2, 0.0f);
        return p + _center;
    } else {
        float theta = util::get_rand_range_val(1.0) * 2 * PI;
        float r = _inner_radius + util::get_rand_range_val(1.0) * _width;
        return Point2f(r * cos(theta), r * sin(theta)) + _center;
    }
}

float CircleTrack::horizontal_displacement(const Point2f &p) {
    Point2f rel_pos = p - _center;
    return (2 * cv::norm(rel_pos) - _inner_radius - _outer_radius) / _width;
}

float CircleTrack::horizontal_displacement(float x, float y) {
    return horizontal_displacement(Point2f(x, y));
}

Point2f CircleTrack::get_tangent_vec(const Point2f &p) {
    Point2f t(_center.y - p.y, p.x - _center.x);
    return t * (1 / cv::norm(t));
}

StraightTrack::StraightTrack() { StraightTrack(0.0f, 0.0f, 100.0f, 20.0f); }

StraightTrack::StraightTrack(float x, float y, float length, float width)
    : _mid_pos(x, y), _length(length) {
    _width = width;
    _start_pos = _mid_pos - Point2f(0.0f, 0.4 * _length);
    _end_pos = _mid_pos + Point2f(0.0f, 0.6 * _length);
}

StraightTrack::StraightTrack(const StraightTrack &st) {
    _mid_pos = st._mid_pos;
    _end_pos = st._end_pos;
    _start_pos = st._start_pos;
    _length = st._length;
    _width = st._width;
}

void StraightTrack::draw(Mat img) {
    // road base
    cv::rectangle(img,
         _start_pos + Point2f(-0.75*_width, -10.0f),
         _end_pos + Point2f(0.75*_width, 30.0f),
         road_base_color,
         -1,
         8);
    // lane
    cv::rectangle(img,
         _start_pos + Point2f(-_width / 2, -10.0f),
         _end_pos + Point2f(_width / 2, 30.0f),
         lane_color,
         -1,
         8);
    
    // start line
    cv::line(img,
         _start_pos + Point2f(-0.75 * _width, 0.0f),
         _start_pos + Point2f(0.75 * _width, 0.0f),
         Scalar(255,255,255),
         2,
         8);
    cv::putText(img,
                std::string("START"),
                _start_pos + Point2f(-100-0.75*_width, 0.0f),
                cv::FONT_HERSHEY_SIMPLEX,
                1,
                Scalar(255, 255, 255),
                2,
                8);
    // finish line
    cv::line(img,
         _end_pos + Point2f(-0.75 * _width, 0.0f),
         _end_pos + Point2f(0.75 * _width, 0.0f),
         Scalar(255,255,255),
         2,
         8);
    cv::putText(img,
                std::string("FINISH"),
                _end_pos + Point2f(-100-0.75*_width, 00.0f),
                cv::FONT_HERSHEY_SIMPLEX,
                1,
                Scalar(255, 255, 255),
                2,
                8);

    // left and rigth boundaries
    cv::line(img,
         _start_pos + Point2f(-_width / 2, -10.0f),
         _end_pos + Point2f(-_width / 2, 30.0f),
         Scalar(255,255,255),
         1,
         8);
    cv::line(img,
         _start_pos + Point2f(_width / 2, -10.0f),
         _end_pos + Point2f(_width / 2, 30.0f),
         Scalar(255,255,255),
         1,
         8);
}

bool StraightTrack::out_of_bound(const cv::Point2f &pos) {
    return (pos.x < _mid_pos.x - _width / 2) ||
           (pos.x > _mid_pos.x + _width / 2) || (pos.y < _start_pos.y) ||
           (pos.y > _end_pos.y);
}

bool StraightTrack::race_finish(const cv::Point2f &pos) {
    return pos.y > _end_pos.y;
}

Point2f StraightTrack::get_start_pos(bool random) {
    if (!random) {
        return _start_pos;
    } else {
        float dy = util::get_rand_range_val(1.0) * _length / 2;
        float dx = (util::get_rand_range_val(1.0) - 0.5) * _width;
        return Point2f(dx, dy) + _start_pos;
    }
}

float StraightTrack::horizontal_displacement(const Point2f &p) {
    return 2 * (p.x - _mid_pos.x) / _width;
}

float StraightTrack::horizontal_displacement(float x, float y) {
    return 2 * (x - _mid_pos.x) / _width;
}

float StraightTrack::vertical_displacement(const Point2f &p) {
    return 2 * (p.y - _mid_pos.y) / _length;
}

float StraightTrack::vertical_displacement(float x, float y) {
    return 2 * (y - _mid_pos.y) / _length;
}

Point2f StraightTrack::get_tangent_vec(const Point2f &p) {
    return Point2f(0.0f, 1.0f);
}

// -------------------------------- Car --------------------------------
BaseCar::BaseCar() : _pos(0.0f, 0.0f), _angle(PI / 2) {}

BaseCar::BaseCar(float x, float y, float angle) : _pos(x, y), _angle(angle) {}

void BaseCar::move(float d, float da) {
    _angle += da;
    if (_angle > 2 * PI)
        _angle -= 2 * PI;
    else if (_angle < 0)
        _angle += 2 * PI;

    _pos += d * Point2f(cos(_angle), sin(_angle));
}

void BaseCar::set_angle(bool random) {
    if (random) {
        _angle = util::get_rand_range_val(1.0) * 2 * PI;
    } else {
        _angle = PI / 2;
    }
}

CircleCar::CircleCar() : BaseCar(), _radius(5.0f) {}

CircleCar::CircleCar(float x, float y, float angle, float radius)
    : BaseCar(x, y, angle), _radius(radius) {}

void CircleCar::draw(Mat img) {
    cv::circle(img, _pos, _radius, Scalar(255,0,0), -1, 8);
    Point2f p = _pos + 2 * _radius * Point2f(cos(_angle), sin(_angle));
    cv::arrowedLine(img, _pos, p, Scalar(255,255,255), 1, 8, 0, 0.4);
}

// -------------------------------- Race Engine --------------------------------
RaceEngine::RaceEngine(float width, float height)
    : _width(width), _height(height), _random(FLAGS_random) {
    _delta_ang = PI / 10;
    _delta_fwd = 1;
}

void RaceEngine::add_track(std::shared_ptr<Track> track) {
    _track_pools.push_back(track);
}

void RaceEngine::reset_game() {
    if (_random) {
        int track_id = round(std::max(
            0.0,
            std::min(floor(util::get_rand_range_val(1.0) * _track_pools.size()),
                     double(_track_pools.size() - 1))));
        _track = _track_pools[track_id];
    } else {
        _track = _track_pools[0];
    }

    _car.set_pos(_track->get_start_pos(_random));
    _car.set_angle(_random);
    VLOG(1) << "start pos: " << _random << " " << _car.get_pos() << " "
            << _car.get_angle();
    _steps = 0;
    _last_reward = 0.0f;
}

bool RaceEngine::game_over() {
    return out_of_bound() /*|| _steps >= _track->max_steps()*/;
}

float RaceEngine::act(Action a) {
    _steps++;
    int action_id = (int)a;
    if (FLAGS_lock_step) {
        int key = cv::waitKey(0) % 256;
        switch (key) {
            case 'l':
                action_id = 6;
                break;
            case 'j':
                action_id = 3;
                break;
            case 'i':
                action_id = 2;
                break;
            case 'k':
                action_id = 1;
                break;
            default:
                break;
        }
    }
    float d_forward = 0.0f, d_turn = 0.0f;
    switch (action_id % 3) {
        case 0:
            break;
        case 1:
            d_forward = _delta_fwd;
            break;
        case 2:
            d_forward = -_delta_fwd;
    }
    action_id /= 3;
    switch (action_id % 3) {
        case 0:
            break;
        case 1:
            d_turn = _delta_ang;
            break;
        case 2:
            d_turn = -_delta_ang;
    }

    _car.move(d_forward, d_turn);
    _last_reward = get_reward(d_forward, _car.get_angle());

    VLOG(1) << "[" << _steps << "] action " << a << "(" << d_forward << ","
            << d_turn << ")"
            << ", loc: " << _car.get_pos() << ", angle: " << _car.get_angle();

    return _last_reward;
}

Mat RaceEngine::draw() {
    Mat canvas = Mat::zeros(_height, _width, CV_8UC3);
    _track->draw(canvas);
    _car.draw(canvas);

    std::vector<float> state;
    get_screen(state);
    float theta = state[1] >= 0 ? acos(state[0]) : -acos(state[0]);
    char msg[1000];
    sprintf(msg, "orientation: %.2f(%.2f, %.2f),", theta / PI * 180, state[0], state[1]);
    cv::putText(canvas,
                std::string(msg),
                cv::Point(10, 40),
                cv::FONT_HERSHEY_SIMPLEX,
                0.5,
                Scalar(255, 255, 255),
                1.5,
                8);
    sprintf(msg, "dist to middle of lane: %.2f", state[2]);
    cv::putText(canvas,
                std::string(msg),
                cv::Point(10, 60),
                cv::FONT_HERSHEY_SIMPLEX,
                0.5,
                Scalar(255, 255, 255),
                1.5,
                8);
    sprintf(msg, "dist to finish line: %.2f", state[3]);
    cv::putText(canvas,
                std::string(msg),
                cv::Point(10, 80),
                cv::FONT_HERSHEY_SIMPLEX,
                0.5,
                Scalar(255, 255, 255),
                1.5,
                8);

    VLOG(1) << "[" << _steps << "] state: " << state << ", "
            << "angle to tagent: " << theta;

    return canvas;
}

float RaceEngine::get_reward(float forward, float angle) {
    Point2f p = _car.get_pos();
    Point2f t = _track->get_tangent_vec(p);
    // projected displacement
    float vx = cos(angle), vy = sin(angle);
    float reward_speed = (vx * t.x + vy * t.y) * forward;
    float reward_finish = _track->race_finish(p) ? 2.0f : 0.0f;
    float reward_boundary = 0;
    if (FLAGS_difficulty == "easy") {
        reward_boundary = -fabs(_track->horizontal_displacement(p));
        VLOG(1) << "[" << _steps << "] reward_finish: " << reward_finish;
        VLOG(1) << "[" << _steps << "] reward_boundary: " << reward_boundary;
        VLOG(1) << "[" << _steps << "] reward_speed: " << reward_speed;
    } else {
        bool hit_boundary = _track->out_of_bound(p) && !_track->race_finish(p);
        reward_boundary = hit_boundary ? -2.0f : 0.0f;
        VLOG(1) << "[" << _steps << "] reward_finish: " << reward_finish;
        VLOG(1) << "[" << _steps << "] reward_boundary: " << reward_boundary;
        VLOG(1) << "[" << _steps << "] reward_speed: " << reward_speed;
    }

    float reward = reward_finish + reward_boundary + reward_speed;

    return reward * FLAGS_reward_scale;
}

void RaceEngine::get_screen(std::vector<float> &state) {
    Point2f t = _track->get_tangent_vec(_car.get_pos());
    float a = _car.get_angle();
    float cos_theta =
        std::max(-1.0d, std::min(1.0d, t.x * cos(a) + t.y * sin(a)));
    float sin_theta = sqrt(1 - cos_theta * cos_theta);
    float theta = acos(cos_theta);
    if (cos(a) * t.y + sin(a) * t.x < 0) {
        sin_theta = -sin_theta;
        theta = -theta;
    }

    state.clear();
    state.push_back(cos_theta);
    state.push_back(sin_theta);
    // state.push_back(theta/PI);
    state.push_back(_track->horizontal_displacement(_car.get_pos()));
    state.push_back(_track->vertical_displacement(_car.get_pos()));
}

RaceEngine::ActionVect RaceEngine::get_action_set() {
    if (FLAGS_race_full_manouver) {
        LOG(INFO) << "using all manouver";
        return RaceEngine::ActionVect({0, 1, 2, 3, 4, 5, 6, 7, 8});
    } else {
        LOG(INFO) << "left/right turns only";
        return RaceEngine::ActionVect({4, 7});
    }
}

bool RaceEngine::out_of_bound() { return _track->out_of_bound(_car.get_pos()); }

SimpleRaceGame::SimpleRaceGame()
    : _race(WINDOW_WIDTH, WINDOW_HEIGHT) {
    _legal_actions = _race.get_action_set();
    float cx = WINDOW_WIDTH / 2, cy = WINDOW_HEIGHT / 2;
    std::shared_ptr<Track> track;
    if (FLAGS_track_type == "circle") {
        float r_in = FLAGS_track_radius, width = FLAGS_track_width;
        track.reset(new CircleTrack(cx, cy, r_in, width));
    } else if (FLAGS_track_type == "straight") {
        float length = FLAGS_track_length, width = FLAGS_track_width;
        track.reset(new StraightTrack(cx, cy, length, width));
    }
    _race.add_track(track);
    reset_game();
}

void SimpleRaceGame::reset_game() {
    _race.reset_game();
    GameSimulator::reset_game();
}

int SimpleRaceGame::game_over() {
    return GameSimulator::game_over() | (_race.game_over() ? DEAD : ALIVE);
}

float SimpleRaceGame::take_action(const StatePacket &actions) {
    CHECK_EQ(actions.size(), 1);
    int action_id = *(actions.get_buffer("action")->get_id());
    CHECK_LT(action_id, _legal_actions.size());
    last_action_ = std::to_string(action_id);
    simple_race::RaceEngine::Action a = _legal_actions[action_id];
    return _race.act(a);
}

int SimpleRaceGame::get_num_actions() { return _legal_actions.size(); }

void SimpleRaceGame::get_screen(StatePacket &screen) {
    _race.get_screen(_screen);
    screen = StatePacket();
    screen.add_buffer_value("screen", _screen);
}

void SimpleRaceGame::define_state_specs(StatePacket &state) {
    state = StatePacket();
    state.add_key("reward");
    state.add_key("screen");
}

void SimpleRaceGame::get_screen_out_dimensions(size_t &height,
                                               size_t &width,
                                               size_t &channels) {
    height = 1;
    if (_screen.empty()) {
        _race.get_screen(_screen);
    }
    width = _screen.size();
    channels = 1;
}

int SimpleRaceGame::get_lives() { return 1; }

void SimpleRaceGame::show_screen(float reward) {
    cv::Mat img = _race.draw();

    // imshow is not thread safe, so we use lock here.
    std::lock_guard<std::mutex> guard(GameSimulator::s_display_mutex_);
    cv::imshow("Simple Race", img);
    if (FLAGS_pause_screen) {
        cv::waitKey(-1);
    } else {
        cv::waitKey(20);
    }
}
}
}  // namespace simulator::simple_race
