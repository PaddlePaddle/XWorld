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

#include <glog/logging.h>
#include <math.h>
#include <iostream>
#include <memory>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <random>
#include <vector>
#include "simulator.h"

DECLARE_string(track_type);
DECLARE_double(track_width);
DECLARE_double(track_length);
DECLARE_double(track_radius);
DECLARE_bool(race_full_manouver);
DECLARE_bool(random);
DECLARE_string(difficulty);

namespace simulator {
namespace simple_race {

#define PI 3.1415926

template <typename T>
std::ostream &operator<<(std::ostream &os, const std::vector<T> vec);

// -------------------------------- Track --------------------------------
class Track {
  public:
    virtual ~Track(){};

    virtual void draw(cv::Mat img) = 0;

    virtual bool out_of_bound(const cv::Point2f &pos) = 0;

    virtual bool race_finish(const cv::Point2f &pos) { return false; }

    virtual cv::Point2f get_start_pos(bool random) = 0;

    virtual float horizontal_displacement(const cv::Point2f &p) = 0;
    virtual float horizontal_displacement(float x, float y) = 0;
    virtual float vertical_displacement(const cv::Point2f &p) { return 0; }
    virtual float vertical_displacement(float x, float y) { return 0; }

    virtual cv::Point2f get_tangent_vec(const cv::Point2f &p) = 0;

    virtual int max_steps() { return 2000; };

    // getters
    virtual cv::Point2f get_pos() = 0;
    virtual float get_pos_x() = 0;
    virtual float get_pos_y() = 0;
    // setters
    virtual void set_pos(cv::Point2f c) = 0;
    virtual void set_pos(float x, float y) = 0;
    virtual void set_pos_x(float x) = 0;
    virtual void set_pos_y(float y) = 0;

    float get_width() { return _width; }

  protected:
    float _width;
};

class CircleTrack : public Track {
  public:
    CircleTrack();
    CircleTrack(float cx, float cy, float inner_radius, float width);
    CircleTrack(const CircleTrack &ct);
    virtual ~CircleTrack(){};

    void draw(cv::Mat img);

    bool out_of_bound(const cv::Point2f &pos);

    cv::Point2f get_start_pos(bool random);

    float horizontal_displacement(const cv::Point2f &p);
    float horizontal_displacement(float x, float y);

    cv::Point2f get_tangent_vec(const cv::Point2f &p);

    // getters
    cv::Point2f get_pos() { return _center; };
    float get_pos_x() { return _center.x; }
    float get_pos_y() { return _center.y; }
    float get_inner_radius() { return _inner_radius; }
    float get_outer_radius() { return _outer_radius; }

    // setters
    void set_pos(cv::Point2f c) { _center = c; }
    void set_pos(float x, float y) {
        _center.x = x;
        _center.y = y;
    }
    void set_pos_x(float x) { _center.x = x; }
    void set_pos_y(float y) { _center.y = y; }

    void set_inner_radius(float inner_radius) {
        _inner_radius = inner_radius;
        _outer_radius = _inner_radius + _width;
    }
    void set_width(const float &width) {
        _width = width;
        _outer_radius = _inner_radius + _width;
    }

  private:
    cv::Point2f _center;
    float _inner_radius, _outer_radius;
};

class StraightTrack : public Track {
  public:
    StraightTrack();
    StraightTrack(float x, float y, float length, float width);
    StraightTrack(const StraightTrack &st);
    virtual ~StraightTrack(){};

    void draw(cv::Mat img);

    bool out_of_bound(const cv::Point2f &pos);

    bool race_finish(const cv::Point2f &pos);

    cv::Point2f get_start_pos(bool random);

    float horizontal_displacement(const cv::Point2f &p);
    float horizontal_displacement(float x, float y);
    float vertical_displacement(const cv::Point2f &p);
    float vertical_displacement(float x, float y);

    cv::Point2f get_tangent_vec(const cv::Point2f &p);

    // getters
    cv::Point2f get_pos() { return _mid_pos; }
    float get_pos_x() { return _mid_pos.x; }
    float get_pos_y() { return _mid_pos.y; }
    // setters
    void set_pos(cv::Point2f p) { _mid_pos = p; };
    void set_pos(float x, float y) {
        _mid_pos.x = x;
        _mid_pos.y = y;
    }
    void set_pos_x(float x) { _mid_pos.x = x; }
    void set_pos_y(float y) { _mid_pos.y = y; }

  private:
    cv::Point2f _mid_pos, _start_pos, _end_pos;
    float _length;
};

// -------------------------------- Car --------------------------------
class BaseCar {
  public:
    BaseCar();
    BaseCar(float x, float y, float angle = PI / 2);
    virtual ~BaseCar(){};

    virtual void move(float d, float da = 0.0f);

    virtual void draw(cv::Mat img) = 0;

    cv::Point2f get_pos() { return _pos; }
    float get_pos_x() { return _pos.x; }
    float get_pos_y() { return _pos.y; }

    float get_angle() { return _angle; }

    void set_pos(float x, float y) {
        _pos.x = x;
        _pos.y = y;
    }

    void set_pos(cv::Point2f pos) { _pos = pos; }

    void set_angle(float angle) { _angle = angle; }

    void set_angle(bool random);

  protected:
    cv::Point2f _pos;
    float _angle;
};

class CircleCar : public BaseCar {
  public:
    CircleCar();
    CircleCar(float x, float y, float angle, float _radius);

    virtual ~CircleCar(){};

    void draw(cv::Mat img);

  private:
    float _radius;
};

// -------------------------------- Race Engine --------------------------------
class RaceEngine {
  public:
    typedef int Action;
    typedef std::vector<Action> ActionVect;

    RaceEngine();
    RaceEngine(float width = 800.0f, float height = 800.0f);

    void add_track(std::shared_ptr<Track> track);

    void reset_game();

    bool game_over();

    float act(Action a);

    cv::Mat draw();

    void get_screen(std::vector<float> &state);

    ActionVect get_action_set();

  private:
    bool out_of_bound();

    float get_reward(float forward, float angle);

    std::vector<std::shared_ptr<Track>> _track_pools;
    CircleCar _car;
    std::shared_ptr<Track> _track;
    float _width, _height;
    float _last_reward;
    int _steps;

    bool _random;

    float _delta_fwd, _delta_ang;
};

// simple race interface.
class SimpleRaceGame : public GameSimulator {
  public:
    SimpleRaceGame();

    virtual void reset_game() override;

    virtual int game_over() override;

    virtual int get_num_actions() override;

    virtual void show_screen(float reward) override;

    virtual int get_lives() override;

    float take_action(const StatePacket &actions) override;

    void get_screen(StatePacket &screen) override;

    void define_state_specs(StatePacket &state);

    void get_screen_out_dimensions(size_t &height,
                                   size_t &width,
                                   size_t &channels) override;

  private:
    RaceEngine _race;
    RaceEngine::ActionVect _legal_actions;
    std::vector<float> _screen;
};
}
}
