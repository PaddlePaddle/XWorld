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

#include <map>
#include <tuple>

#include "roboschool_API.h"
#include "simulator_entity.h"
#include "simulator_util.h"
#include "xworld3d_flags.h"

namespace simulator {
namespace xworld3d {

#define EPSILON 1e-6
#define UNIT FLAGS_x3_unit

class X3Item;
class X3Agent;

typedef double x3real;
typedef std::shared_ptr<X3Item> X3ItemPtr;
typedef std::shared_ptr<X3Agent> X3AgentPtr;

class X3Item {
protected:
    using Pose = roboschool::Pose;
    using Object = roboschool::Object;
    using World = roboschool::World;
    using Thingy = roboschool::Thingy;
    using RenderResult = roboschool::RenderResult;
    using Camera = roboschool::Camera;
public:
    static X3ItemPtr create_item(const Entity& e, World& world);

    X3Item(const Entity& e, World& world);

    X3Item(const X3Item&) = delete;

    X3Item& operator=(const X3Item&) = delete;

    void destroy() {
        object_.destroy();
    }

    // destructor
    virtual ~X3Item() { this->destroy(); }

    const Object& object() const { return object_; }

    // get the type of the item
    std::string type() const { return e_.type; }

    // get the id of the item
    std::string id() const { return e_.id; }

    // get the name of the item
    std::string name() const { return e_.name; }

    // get the color of the item
    std::string color() const { return e_.color; }

    // get the location of the item
    Vec3 location() const;

    // set the location of the item
    void set_item_location(const x3real x, const x3real y, const x3real z);

    Pose pose() const { return object_.pose(); }

    void set_pose(const Pose& pose);

    void set_speed(x3real vx, x3real vy, x3real vz);

    void set_pose_and_speed(const Pose& pose,
                                    x3real vx, x3real vy, x3real vz);
    void get_direction(x3real &dir_x, x3real &dir_y) const {
        dir_x = dir_x_;
        dir_y = dir_y_;
    }

    // set the type of the item
    void set_item_type(const std::string& item_type) { e_.type = item_type; }

    Entity entity() const { return e_; }

    void sync_entity_info();

    virtual void move_forward() { LOG(FATAL) << "actions not defined!"; }

    virtual void move_backward() { LOG(FATAL) << "actions not defined!"; }

    virtual void turn_left() { LOG(FATAL) << "actions not defined!"; }

    virtual void turn_right() { LOG(FATAL) << "actions not defined!"; }

    virtual void jump() { LOG(FATAL) << "actions not defined!"; }

    void move_underground();

    virtual X3ItemPtr collect_item(const std::map<std::string, X3ItemPtr>& items,
                                   const std::string& type) {
        LOG(FATAL) << "actions not defined!";
    }

    virtual int get_num_actions() const { return 0; }

    bool equal(const X3Item& i) const {
        return (this->e_.id == i.e_.id);
    }

protected:
    Entity e_;
    Object object_;
    x3real dir_x_;              // the x component of the agent's facing direction
    x3real dir_y_;              // the y component of the agent's facing direction
};

class X3Agent : public X3Item {
public:
    X3Agent(const Entity& e, World& world);

    X3Agent(const X3Agent&) = delete;

    X3Agent& operator=(const X3Agent&) = delete;

    void move_forward() override;

    void move_backward() override;

    void turn_left() override;

    void turn_right() override;

    void jump() override;

    X3ItemPtr collect_item(const std::map<std::string, X3ItemPtr>& items,
                           const std::string& type) override;

private:
    x3real reach_test(const Pose& pose);

    void set_direction();

    const x3real move_speed_norm_;
    const x3real jump_speed_norm_;
    const int orientation_bins_;
    const x3real reaching_dist_; // An agent can collect this goal if it is
                                // within the reaching distance of this goal
    int yaw_id_;
};

class X3Camera {
private:
    using Pose = roboschool::Pose;
    using World = roboschool::World;
    using Camera = roboschool::Camera;
public:
    X3Camera(World& world, int img_height, int img_width);

    X3Camera(const X3Camera&) = delete;

    X3Camera& operator=(const X3Camera&) = delete;

    // TODO: make this function const
    Pose pose() { return camera_.pose(); }

    // Return the image seen by agent.
    // If bird_view is true, a bird view image is also returned.
    roboschool::RenderResult render(X3Item* item, bool bird_view = false);

    // Camera can be attached to an agent so that the rendered image is centered
    // at the agent
    void attach_item(X3Item* item);

    void detach() { item_ = NULL; }

private:
    // Update the pose of the camera.
    void update(bool bird_view);

    Camera camera_;
    X3Item* item_;
};

}} // simulator::xworld3d

namespace std {
template <>
struct hash<simulator::Vec3> {
    size_t operator()(const simulator::Vec3& l) const {
        return hash<simulator::xworld3d::x3real>()(l.x) ^
               hash<simulator::xworld3d::x3real>()(l.y) ^
               hash<simulator::xworld3d::x3real>()(l.z);
    }
};
}
