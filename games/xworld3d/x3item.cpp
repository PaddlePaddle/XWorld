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

const x3real X3Item::UNIT = FLAGS_x3_unit;
const x3real X3Item::UNIT_INV = 1.0 / FLAGS_x3_unit;

const x3real REACH_HEIGHT_THRESHOLD = X3Item::UNIT;
const x3real CAMERA_BIRD_VIEW_HEIGHT = 10.0 * X3Item::UNIT;

X3ItemPtr X3Item::create_item(const Entity& e, World& world) {
    if (e.type == "agent") {
        return std::make_shared<X3Agent>(e, world);
    } else {
        return std::make_shared<X3Item>(e, world);
    }
}

X3Item::X3Item(const Entity& e, World& world) :
        e_(e), dir_x_(0.0f), dir_y_(1.0f) {
    Pose pose(e_.loc.x * UNIT, e_.loc.y * UNIT, e_.loc.z * UNIT);
    object_ = world.load_urdf(e.asset_path, pose, false, false);
    object_.query_position();
}

Vec3 X3Item::location() const {
    const Pose& pose = object_.pose();
    return Vec3(pose.x(), pose.y(), pose.z());
}

void X3Item::set_item_location(const x3real x, const x3real y, const x3real z) {
    Pose pose(x * UNIT, y * UNIT, z * UNIT);
    set_pose(pose);
}

void X3Item::set_speed(x3real vx, x3real vy, x3real vz) {
    object_.set_speed(vx, vy, vz);
}

void X3Item::set_pose(const Pose& pose) {
    object_.set_pose(pose);
}

void X3Item::set_pose_and_speed(const Pose& pose,
                                x3real vx, x3real vy, x3real vz) {
    object_.set_pose_and_speed(pose, vx, vy, vz);
}

void X3Item::sync_entity_info() {
    std::tie(e_.loc.x, e_.loc.y, e_.loc.z) = object_.pose().xyz();
    e_.loc.scale(UNIT_INV);
}

void X3Item::move_underground() {
   Pose pose(object_.pose());
   pose.set_xyz(pose.x(), pose.y(), -2);
   set_pose_and_speed(pose, 0.0f, 0.0f, 0.0f);
}

X3Agent::X3Agent(const Entity& e, World& world) :
        X3Item(e, world),
        move_speed_norm_(FLAGS_x3_move_speed * UNIT),
        jump_speed_norm_(FLAGS_x3_jump_speed * UNIT),
        orientation_bins_(FLAGS_x3_orientation_bins),
        reaching_dist_(FLAGS_x3_reaching_distance * UNIT) {
    yaw_id_ = simulator::util::get_rand_ind(orientation_bins_);
    set_direction();
}

void X3Agent::move_forward() {
    Pose pose(object_.pose());
    pose.set_xyz(pose.x(), pose.y(), 0.0f);
    x3real vx = move_speed_norm_ * dir_x_;
    x3real vy = move_speed_norm_ * dir_y_;
    //    x3real vz = object_.speed_z();
    set_pose_and_speed(pose, vx, vy, 0.0f);
}

void X3Agent::move_backward() {
    Pose pose(object_.pose());
    pose.set_xyz(pose.x(), pose.y(), 0.0f);
    x3real vx = -move_speed_norm_ * dir_x_;
    x3real vy = -move_speed_norm_ * dir_y_;
    //    x3real vz = object_.speed_z();
    set_pose_and_speed(pose, vx, vy, 0.0f);
}

void X3Agent::turn_left() {
    Pose pose(object_.pose());
    pose.set_xyz(pose.x(), pose.y(), 0.0f);
    // TODO: call set_rpy
    yaw_id_ = (yaw_id_ + 1) % orientation_bins_;
    //    x3real vz = object_.speed_z();
    set_pose_and_speed(pose, 0.0f, 0.0f, 0.0f);
    set_direction();
}

void X3Agent::turn_right() {
    Pose pose(object_.pose());
    pose.set_xyz(pose.x(), pose.y(), 0.0f);
    yaw_id_ = (yaw_id_ - 1) % orientation_bins_ + orientation_bins_;
    //    x3real vz = object_.speed_z();
    set_pose_and_speed(pose, 0.0f, 0.0f, 0.0f);
    set_direction();
}

void X3Agent::jump() {
    if (fabs(pose().z()) < EPSILON) {
        set_speed(0.0f, 0.0f, jump_speed_norm_);
    }
}

X3ItemPtr X3Agent::collect_item(const std::map<std::string, X3ItemPtr>& items,
                                const std::string& type) {
    X3ItemPtr item = nullptr;
    set_speed(0.0f, 0.0f, 0.0f);
    // the angle between the agent's facing direction and
    // the direction from the agent to the item should be less than 45 degrees
    x3real best_score = 0.707; // 45 degrees is the minimum
    x3real score;
    for (auto& kv : items) {
        if (kv.second->type() == type) {
            score = reach_test(kv.second->pose());
            if (score > best_score) {
                item = kv.second;
                best_score = score;
            }
        }
    }
    return item;
}

x3real X3Agent::reach_test(const Pose& pose) {
    const Pose self = object_.pose();
    x3real dx = pose.x() - self.x();
    x3real dy = pose.y() - self.y();
    x3real dz = pose.z() - self.z();
    x3real d = sqrt(dx * dx + dy * dy);
    x3real reaching_score = -1; // lower end of cos range
    if (d < reaching_dist_ && dz < REACH_HEIGHT_THRESHOLD) {
        dx /= d;
        dy /= d;
        reaching_score = dx * dir_x_ + dy * dir_y_;
    }
    return reaching_score;
}

void X3Agent::set_direction() {
    x3real rad = 2 * yaw_id_ * M_PI / orientation_bins_;
    dir_x_ = cos(rad);
    dir_y_ = sin(rad);
}

/********************************** X3Camera **********************************/
X3Camera::X3Camera(World& world, int img_height, int img_width) :
        camera_(world.new_camera_free_float(img_width, img_height, "camera")),
        item_(NULL) {}

void X3Camera::attach_item(X3Item* item) {
    if (item && item_ != item) {
        item_ = item;
    }
}

void X3Camera::update(bool bird_view) {
    // TODO: to support rendering using detached camera
    CHECK(item_) << "camera is detached";
    Pose p = item_->pose();
    if (!bird_view) {
        x3real dir_x, dir_y;
        item_->get_direction(dir_x, dir_y);
        camera_.move_and_look_at(p.x(), p.y(), p.z() + 0.5 * X3Item::UNIT,
                                 p.x() + dir_x, p.y() + dir_y, p.z() + 0.5 * X3Item::UNIT);
    } else {
        // bird view
        camera_.move_and_look_at(p.x(), p.y(), CAMERA_BIRD_VIEW_HEIGHT, p.x(), p.y(), p.z());
    }
}

roboschool::RenderResult X3Camera::render(X3Item* item, bool bird_view) {
    attach_item(item);
    update(bird_view);
    return camera_.render(false, false, false);
}

}} // simulator::xworld3d
