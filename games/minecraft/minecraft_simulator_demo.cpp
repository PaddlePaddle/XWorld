// MIT License

// Copyright (c) 2017 Baidu Inc. All rights reserved.

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
/////////////////////////////////////////////////////////////////////////////////

#include "minecraft_simulator_base.h"

namespace simulator { namespace mcw {

void MinecraftSimulatorDemo::new_mission() {
   // TODO: make this more flexible
    int x = 0;
    int z = 0;
    do {
        x = util::get_rand_ind(5) - 2;
        z = util::get_rand_ind(5) - 2;
    } while ((x == -2 && z == -2) || (x == 2 && z == 2));
    int yaw = util::get_rand_ind(4);
    current_x_ = x + 0.5;
    current_z_ = z + 0.5;
    current_yaw_ = 90 * yaw;
    mission_->startAtWithPitchAndYaw(
        current_x_, 226, current_z_,
        /* pitch= */30,
        current_yaw_);

    // do not need reward for the first observe()
    got_reward_ = true;
}

void MinecraftSimulatorDemo::update_state(int action_id) {
    if (action_id < 2) {
        current_yaw_ += (1 - 2 * action_id) * 90;
        if (current_yaw_ < 0) {
            current_yaw_ += 360;
        } else if (current_yaw_ >= 360) {
            current_yaw_ -= 360;
        }
    } else if (action_id == 2) {
        if (approximately_equal(current_yaw_, 0)) {
            current_z_ = std::min(2.5f, current_z_ + 1);
        } else if (approximately_equal(current_yaw_, 90)) {
            current_x_ = std::max(-1.5f, current_x_ - 1);
        } else if (approximately_equal(current_yaw_, 180)) {
            current_z_ = std::max(-1.5f, current_z_ - 1);
        } else {
            current_x_ = std::min(2.5f, current_x_ + 1);
        }
    }
    got_reward_ = false;
    reached_location_ = false;
    VLOG(2) << "action=" << action_names_[action_id] << " to"
            << " x=" << current_x_ - 0.5
            << " z=" << current_z_ - 0.5
            << " yaw=" << current_yaw_ / 90;

}

bool MinecraftSimulatorDemo::reached_desired_state() {
    auto video_frame = world_state_.video_frames.empty() ? nullptr
            : world_state_.video_frames.back();
    VLOG(3) << " video: " << !world_state_.video_frames.empty()
            << " obs: "
            << (!world_state_.observations.empty()
                && world_state_.observations.back()->text != "{}")
            << " reward: " << !world_state_.rewards.empty()
            << " x=" << (video_frame ? video_frame->xPos - 0.5 : 100)
            << " z=" << (video_frame ? video_frame->zPos - 0.5 : 100)
            << " yaw=" << (video_frame ? video_frame->yaw / 90 : 100);
    if (!world_state_.rewards.empty()) {
        got_reward_ = true;
    }
    if (!world_state_.video_frames.empty()) {
        auto video_frame = world_state_.video_frames.back();
        reached_location_ = approximately_equal(video_frame->xPos, current_x_)
                && approximately_equal(video_frame->zPos, current_z_)
                && approximately_equal(video_frame->yaw, current_yaw_);
    }
    return got_reward_ && reached_location_;
}

}}  // namespace simulator::mcw
