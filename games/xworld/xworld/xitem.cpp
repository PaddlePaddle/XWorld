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

#include "xitem.h"

DECLARE_int32(visible_radius);

namespace simulator {
namespace xwd {

std::unordered_map<std::string, cv::Mat> XItem::item_imgs_;

XItemPtr XItem::create_item(const Entity& e) {
    if (e.type == "agent") {
        return std::make_shared<XAgent>(e);
    } else {
        return std::make_shared<XItem>(e);
    }
}

cv::Mat XItem::get_item_image() {
    auto img_name = e_.asset_path;
    if (!img_name.empty()) {
        if (item_imgs_.count(img_name) == 0) {
            cv::Mat img;
            img = cv::imread(img_name, 1);
            CHECK(!img.empty()) << "could not open or find the image: " + img_name;
            cv::resize(img,
                       img,
                       cv::Size(XItem::item_size_, XItem::item_size_),
                       cv::INTER_LINEAR);
            item_imgs_[img_name] = img;
        }

        // transform the icon
        auto icon = item_imgs_[img_name].clone();
        cv::Point2f center(icon.cols / 2.0, icon.rows / 2.0);
        auto rot_mat = cv::getRotationMatrix2D(center, e_.yaw * 180 / M_PI, e_.scale);
        // e_.offset + e_.scale / 2 - 0.5 computes the translation of the icon image
        // inside the grid
        rot_mat.at<double>(0, 2) += (e_.offset + e_.scale / 2 - 0.5) * icon.cols;
        rot_mat.at<double>(1, 2) += (e_.offset + e_.scale / 2 - 0.5) * icon.rows;
        cv::warpAffine(icon, icon, rot_mat, icon.size(),
                       cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(255, 255, 255));
        return icon;
    }
    return cv::Mat();
}

std::string XItem::get_item_facing_dir(double yaw) {
    std::string dir = "";
    double eps = 1e-4;
    if (fabs(yaw) < eps) {
        dir = "down";
    } else if (fabs(yaw - M_PI / 2) < eps) {
        dir = "right";
    } else if (fabs(yaw - M_PI) < eps) {
        dir = "up";
    } else {
        dir = "left";
    }
    return dir;
}

XAgent::XAgent(const Entity& e) : XItem(e) {
    CHECK(e.type == "agent");
    if (FLAGS_visible_radius == 0) {
        legal_actions_ = {MOVE_UP, MOVE_DOWN, MOVE_LEFT, MOVE_RIGHT};
    } else {
        legal_actions_ = {MOVE_FORWARD, MOVE_BACKWARD, MOVE_LEFT_FPV, MOVE_RIGHT_FPV, TURN_LEFT, TURN_RIGHT};
    }
}

Loc XAgent::act(int action_id) {
    Loc cur_loc = get_item_location();
    CHECK(action_id >= 0 && action_id < get_num_actions());

    auto dir = XItem::get_item_facing_dir(e_.yaw);
    switch(legal_actions_[action_id]) {
        case MOVE_UP: return Loc(cur_loc.x, cur_loc.y - 1);
        case MOVE_DOWN: return Loc(cur_loc.x, cur_loc.y + 1);
        case MOVE_LEFT: return Loc(cur_loc.x - 1, cur_loc.y);
        case MOVE_RIGHT: return Loc(cur_loc.x + 1, cur_loc.y);
        case MOVE_FORWARD:
            if (dir == "right") {
                return Loc(cur_loc.x + 1, cur_loc.y);
            } else if (dir == "left") {
                return Loc(cur_loc.x - 1, cur_loc.y);
            } else if (dir == "up") {
                return Loc(cur_loc.x, cur_loc.y - 1);
            } else {
                return Loc(cur_loc.x, cur_loc.y + 1);
            }
        case MOVE_BACKWARD:
            if (dir == "right") {
                return Loc(cur_loc.x - 1, cur_loc.y);
            } else if (dir == "left") {
                return Loc(cur_loc.x + 1, cur_loc.y);
            } else if (dir == "up") {
                return Loc(cur_loc.x, cur_loc.y + 1);
            } else {
                return Loc(cur_loc.x, cur_loc.y - 1);
            }
        case MOVE_LEFT_FPV:
            if (dir == "right") {
                return Loc(cur_loc.x, cur_loc.y - 1);
            } else if (dir == "left") {
                return Loc(cur_loc.x, cur_loc.y + 1);
            } else if (dir == "up") {
                return Loc(cur_loc.x - 1, cur_loc.y);
            } else {
                return Loc(cur_loc.x + 1, cur_loc.y);
            }
        case MOVE_RIGHT_FPV:
            if (dir == "right") {
                return Loc(cur_loc.x, cur_loc.y + 1);
            } else if (dir == "left") {
                return Loc(cur_loc.x, cur_loc.y - 1);
            } else if (dir == "up") {
                return Loc(cur_loc.x + 1, cur_loc.y);
            } else {
                return Loc(cur_loc.x - 1, cur_loc.y);
            }
        case TURN_LEFT:
            e_.yaw += M_PI / 2;
            if (e_.yaw > M_PI + 1e-4) {
                e_.yaw -= 2 * M_PI;
            }
            return cur_loc;
        case TURN_RIGHT:
            e_.yaw -= M_PI / 2;
            if (e_.yaw < -M_PI / 2 - 1e-4) {
                e_.yaw += 2 * M_PI;
            }
            return cur_loc;
        default:
            LOG(FATAL) << "unrecognized action for the agent";
    }
    return Loc();
}

}
}  // namespace simulator::xwd
