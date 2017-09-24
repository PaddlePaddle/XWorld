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

#include <tuple>

#include "roboschool_API.h"
#include "simulator_util.h"

namespace simulator {
namespace xworld3d {

#define EPSILON 1e-6
#define UNIT 0.1

using roboschool::Pose;
using roboschool::Object;
using roboschool::World;
using roboschool::Camera;

struct Loc3 {
    int x;
    int y;
    int z;

    Loc3() { init(); }

    Loc3(int _x, int _y, int _z) : x(_x), y(_y), z(_z) {}

    Loc3(const Loc3& l) : x(l.x), y(l.y), z(l.z) {}

    Loc3(const std::vector<int>& loc) {
        CHECK(loc.empty() || loc.size() == 3);
        if (!loc.empty()) {
            x = loc[0];
            y = loc[1];
            z = loc[2];
        } else {
            init();
        }
    }

    void init() {
        x = std::numeric_limits<int>::min();
        y = std::numeric_limits<int>::min();
        z = std::numeric_limits<int>::min();
    }

    std::string to_string() const {
        return "(" + std::to_string(x) +
               "," + std::to_string(y) +
               "," + std::to_string(z) + ")";
    }

    bool defined() const {
        return x != std::numeric_limits<int>::min() &&
               y != std::numeric_limits<int>::min() &&
               z != std::numeric_limits<int>::min();
    }

    void random_loc(int w, int h) {
        x = int(w * simulator::util::get_rand_range_val(1.0));
        y = int(h * simulator::util::get_rand_range_val(1.0));
        z = 0;
    }

    bool in_boundary(int w, int h) const {
        return x >= 0 && x < w && y >= 0 && y < h;
    }

    bool operator>=(const Loc3& l) const {
        return x >= l.x && y >= l.y && z >= l.z;
    }

    bool operator==(const Loc3& l) const {
        return x == l.x && y == l.y && z == l.z;
    }

    bool operator!=(const Loc3& l) const {
        return x != l.x || y != l.y || z != l.z;
    }

    Loc3 operator-(const Loc3& l) const {
        return {x - l.x, y - l.y, z - l.z};
    }

    Loc3 operator+(const Loc3& l) const {
        return {x + l.x, y + l.y, z + l.z};
    }

    double square_distance(const Loc3& l) const {
        return (x-l.x) * (x-l.x) + (y-l.y) * (y-l.y) + (z-l.z) * (z-l.z);
    }
};

enum X3EntityType { GOAL = 0, AGENT = 1, BLOCK = 2, DUMMY = 3 };

class X3Entity {
public:
    X3Entity(std::string name, X3EntityType type) : name_(name), type_(type) {}

    virtual ~X3Entity() {}

    const std::string& name() const { return name_; }

    X3EntityType type() { return type_; }

    virtual void destroy() = 0;

protected:
    std::string name_;
    X3EntityType type_;
};

struct X3ItemInfo {
    X3ItemInfo(std::string n, X3EntityType t, std::string f) :
               name(n), type(t), model_file(f), loc() {}

    X3ItemInfo(std::string n, X3EntityType t, std::string f,
               int x, int y, int z) :
            name(n), type(t), model_file(f), loc(x, y, z) {}

    X3ItemInfo(std::string n, X3EntityType t, std::string f,
               const Loc3& l) :
            name(n), type(t), model_file(f), loc(l) {}

    X3ItemInfo(std::string n, X3EntityType t, std::string f,
               const std::vector<int>& v) :
            name(n), type(t), model_file(f), loc(v) {}

    std::string name;
    X3EntityType type;
    std::string model_file;
    Loc3 loc;
};

class X3Item : public X3Entity {
public:
    X3Item(const X3ItemInfo& info, World& world);

    virtual ~X3Item() {
        this->destroy();
    }

    const Object& object() {
        return object_;
    }

    Pose pose() { return object_.pose(); }

    void set_pose(const Pose& pose);

    void set_speed(double vx, double vy, double vz);

    void set_pose_and_speed(const Pose& pose, double vx, double vy, double vz);

    bool equal(const X3Item* i) {
        return (name_ == i->name());
    }

    void destroy() override;

protected:
    std::string model_file_;
    Object object_;
};

/*********************************** X3Block **********************************/
class X3Block : public X3Item {
public:
    X3Block(const X3ItemInfo& info, World& world) : X3Item(info, world) {
        CHECK(info.type == X3EntityType::BLOCK);
    }
};

typedef std::shared_ptr<X3Block> X3BlockPtr;

/*********************************** X3Goal ***********************************/
class X3Goal : public X3Item {
public:
    X3Goal(const X3ItemInfo& info, World& world) : X3Item(info, world) {
        CHECK(info.type == X3EntityType::GOAL);
    }
};

typedef std::shared_ptr<X3Goal> X3GoalPtr;

/*********************************** X3Agent **********************************/
class X3Agent : public X3Item {
public:
    X3Agent(const X3ItemInfo& info, World& world,
            float speed_norm, int orientation_bins,
            float reaching_dist);

    void move_forward();

    void turn_left();

    void turn_right();

    void jump();

    void get_direction(double &dir_x, double &dir_y) {
        dir_x = dir_x_;
        dir_y = dir_y_;
    }

    float reach_test(const Pose& gpose);

private:
    void set_direction();

    const float speed_norm_;
    int yaw_id_;
    double dir_x_;
    double dir_y_;
    const int orientation_bins_;
    const float reaching_dist_; // An agent can collect this goal if it is
                                // within the reaching distance of this goal
};

typedef std::shared_ptr<X3Agent> X3AgentPtr;

/********************************** X3Camera **********************************/
class X3Camera {
public:
    X3Camera(World& world, int img_height, int img_width);

    Pose pose() { return camera_.pose(); }

    roboschool::RenderResult render(X3Agent* agent, bool debug = false);

    void detach() { agent_ = NULL; }

private:
    void attach_agent(X3Agent* agent);

    void update(bool is_debug);

    Camera camera_;
    X3Agent* agent_;
};

}} // simulator::xworld3d

namespace std {
template <>
struct hash<simulator::xworld3d::Loc3> {
    size_t operator()(const simulator::xworld3d::Loc3& l) const {
        return hash<int>()(l.x) ^ hash<int>()(l.y);
    }
};
}