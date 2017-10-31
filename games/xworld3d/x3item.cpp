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

#include "x3item.h"

namespace simulator {
namespace xworld3d {

using simulator::util::path_join;

X3Item::X3Item(const X3ItemInfo& info, World& world) :
        X3Entity(info.name, info.type),
        model_file_(info.model_file) {
    Pose pose(info.loc.x * UNIT, info.loc.y * UNIT, info.loc.z * UNIT);
    object_ = world.load_urdf(info.model_file, pose, false, false);
    object_.query_position();
}

void X3Item::set_pose(const Pose& pose) {
    object_.set_pose(pose);
}

void X3Item::set_speed(double vx, double vy, double vz) {
    object_.set_speed(vx, vy, vz);
}

void X3Item::set_pose_and_speed(const Pose& pose,
                                double vx, double vy, double vz) {
    object_.set_pose_and_speed(pose, vx, vy, vz);
}

inline void X3Item::destroy() {
    object_.destroy();
}

/*********************************** X3Block***********************************/

/*********************************** X3Goal ***********************************/

/*********************************** X3Agent **********************************/
X3Agent::X3Agent(const X3ItemInfo& info, World& world,
                 float speed_norm, int orientation_bins,
                 float reaching_dist) :
        X3Item(info, world),
        speed_norm_(speed_norm),
        orientation_bins_(orientation_bins),
        reaching_dist_(reaching_dist) {
    CHECK(info.type == X3EntityType::AGENT);
    yaw_id_ = simulator::util::get_rand_ind(orientation_bins_);
    set_direction();
}

void X3Agent::move_forward() {
    float vx = speed_norm_ * dir_x_;
    float vy = speed_norm_ * dir_y_;
    float vz = object_.speed_z();
    set_pose_and_speed(pose(), vx, vy, vz);
}

void X3Agent::turn_left() {
    yaw_id_ = (yaw_id_ + 1) % orientation_bins_;
    float vz = object_.speed_z();
    set_speed(0.0f, 0.0f, vz);
    set_direction();
}

void X3Agent::turn_right() {
    yaw_id_ = (yaw_id_ - 1) % orientation_bins_ + orientation_bins_;
    float vz = object_.speed_z();
    set_speed(0.0f, 0.0f, vz);
    set_direction();
}

void X3Agent::jump() {
    if (fabs(pose().z()) < EPSILON) {
        set_speed(0.0f, 0.0f, speed_norm_);
    }
}

float X3Agent::reach_test(const Pose& gpose) {
    float dx = gpose.x() - pose().x();
    float dy = gpose.y() - pose().y();
    float dz = gpose.z() - pose().z();
    float d = sqrt(dx * dx + dy * dy);
    float reaching_score = -2; // a value smaller than any possible cosin 
                               // similarity value
    if (d < reaching_dist_ && dz < 0.05) {
        dx /= d;
        dy /= d;
        reaching_score = dx * dir_x_ + dy * dir_y_;
    }

    return reaching_score;
}

void X3Agent::set_direction() {
    float rad = 2 * yaw_id_ * M_PI / orientation_bins_;
    dir_x_ = cos(rad);
    dir_y_ = sin(rad);
}

/********************************** X3Camera **********************************/
X3Camera::X3Camera(World& world, int img_height, int img_width) :
        camera_(world.new_camera_free_float(img_width, img_height, "camera")),
        agent_(NULL) {}

void X3Camera::attach_agent(X3Agent* agent) {
    if (agent && agent_ != agent) {
        agent_ = agent;
    }
}

void X3Camera::update(bool bird_view) {
    // TODO: to support rendering using detached camera
    CHECK(agent_) << "camera is detached";
    Pose p = agent_->pose();
    if (!bird_view) {
        double dir_x, dir_y;
        agent_->get_direction(dir_x, dir_y);
        camera_.move_and_look_at(p.x(), p.y(), p.z(),
                                 p.x() + dir_x, p.y() + dir_y, p.z());
    } else {
        // bird view
        camera_.move_and_look_at(p.x(), p.y(), 1.0f, p.x(), p.y(), p.z());
    }
}

roboschool::RenderResult X3Camera::render(X3Agent* agent, bool bird_view) {
    attach_agent(agent);
    update(bird_view);
    return camera_.render(false, false, false);
}

}} // simulator::xworld3d
