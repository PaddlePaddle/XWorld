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

#include "xmap.h"
#include "xitem.h"
#include <cmath>

namespace simulator {
namespace xwd {

XMap::XMap() { init(0, 0); }

XMap::XMap(int height, int width) { init(height, width); }

void XMap::init(int height, int width) {
    height_ = height;
    width_ = width;

    grid_size_ = XItem::item_size_;
    item_ptr_cube_.resize(height_);
    for (int i = 0; i < height_; i++) {
        item_ptr_cube_[i].resize(width_);
    }
}

void XMap::add_items(const std::vector<XItemPtr>& item_list) {
    for (unsigned int i = 0; i < item_list.size(); i++) {
        add_item(item_list[i]);
    }
}

void XMap::remove_items(const std::vector<XItemPtr>& item_list) {
    for (unsigned int i = 0; i < item_list.size(); i++) {
        remove_item(item_list[i]);
    }
}

void XMap::add_item(XItemPtr item_ptr) {
    Loc loc = item_ptr->get_item_location();
    bool exist_flag = false;
    auto& items = item_ptr_cube_[loc.y][loc.x];
    for (size_t i = 0; i < items.size(); i++) {
        if (item_ptr->get_item_id() == items[i]->get_item_id()) {
            exist_flag = true;
        }
    }
    if (!exist_flag) {
        items.push_back(item_ptr);
    }
}

void XMap::remove_item(XItemPtr item_ptr) {
    Loc loc = item_ptr->get_item_location();
    auto& items = item_ptr_cube_[loc.y][loc.x];
    for (unsigned int i = 0; i < items.size(); i++) {
        if (item_ptr->get_item_id() == items[i]->get_item_id()) {
            items.erase(items.begin() + i);
            break;
        }
    }
}

bool XMap::move_item(XItemPtr item, const Loc& target) {
    if (is_reachable(target.x, target.y)) {
        remove_item(item);
        item->set_item_location(target.x, target.y);
        add_item(item);
        return true;
    }
    return false;
}

bool XMap::is_reachable(int x, int y) const {
    if (x < 0 || y < 0 || x >= width_ || y >= height_) return false;

    bool is_reachable_flag = true;
    for (unsigned int i = 0; i < item_ptr_cube_[y][x].size(); i++) {
        is_reachable_flag =
            is_reachable_flag && item_ptr_cube_[y][x][i]->is_reachable();
    }
    return is_reachable_flag;
}

bool XMap::is_empty(int x, int y) {
    if (x < 0 || y < 0 || x >= width_ || y >= height_) return false;
    return item_ptr_cube_[y][x].size() > 0 ? false : true;
}

Loc XMap::get_item_location(std::string item_id) {
    unsigned int nr = item_ptr_cube_.size();
    unsigned int nc = item_ptr_cube_[0].size();
    Loc loc = {2, 0};
    for (unsigned int i = 0; i < nr; i++) {
        for (unsigned int j = 0; j < nc; j++) {
            for (unsigned int k = 0; k < item_ptr_cube_[i][j].size(); k++) {
                if (item_ptr_cube_[i][j][k]->get_item_id() == item_id) {
                    loc = item_ptr_cube_[i][j][k]->get_item_location();
                    return loc;
                }
            }
        }
    }
    return loc;
}

cv::Mat XMap::to_image(const Loc& item_loc,
                       double yaw,
                       bool flag_illustration,
                       int visible_radius_unit) {
    cv::Mat world(height_ * grid_size_,
                  width_ * grid_size_,
                  CV_8UC3,
                  cv::Scalar(255, 255, 255));

    for (int i = 0; i < height_; i++) {
        for (int j = 0; j < width_; j++) {
            for (unsigned int k = 0; k < item_ptr_cube_[i][j].size(); k++) {
                cv::Mat item_image = item_ptr_cube_[i][j][k]->get_item_image();

                cv::Mat imageROI = world(cv::Rect(j * grid_size_,
                                                  i * grid_size_,
                                                  grid_size_,
                                                  grid_size_));
                item_image.copyTo(imageROI);
            }
        }
    }

    cv::Mat view;
    if (visible_radius_unit > 0) {
        // pad the original image
        cv::copyMakeBorder(world,
                           world,
                           visible_radius_unit * grid_size_,
                           visible_radius_unit * grid_size_,
                           visible_radius_unit * grid_size_,
                           visible_radius_unit * grid_size_,
                           cv::BORDER_CONSTANT,
                           cv::Scalar(0, 0, 0));
        auto roi = image_masking(world,
                                 item_loc,
                                 yaw,
                                 visible_radius_unit);
        world(roi).copyTo(view);
        if (flag_illustration) {
            cv::Mat black(world.rows, world.cols, CV_8UC3, cv::Scalar(0, 0, 0));
            cv::addWeighted(black, 0.5, world, 0.5, 0.0, world);
            view.copyTo(world(roi));
            view = world;
        } else { // rotate the image according to the agent's yaw
            cv::Point2f center(view.cols / 2.0, view.rows / 2.0);
            cv::Mat rot_mat = cv::getRotationMatrix2D(center, yaw * 180 / M_PI + 180, 1.0);
            cv::warpAffine(view, view, rot_mat, view.size());
            // TODO: shadow behind wall blocks
        }
    } else {
        view = image_centering(world, item_loc);
    }

    return view;
}

void XMap::draw_success(cv::Mat& img, int success) {
    int height = img.rows;
    int width = img.cols;
    int radius = std::min(height, width) / 3;
    if (success > 0) {
        cv::circle(img,
                   cv::Point(width / 2, height / 2),
                   radius,
                   cv::Scalar(0, 255, 0),
                   3,
                   CV_AA);
    } else {
        int size = int(0.7 * radius);  // sqrt(2)
        cv::line(img,
                 cv::Point(width / 2 - size, height / 2 - size),
                 cv::Point(width / 2 + size, height / 2 + size),
                 cv::Scalar(0, 0, 255),
                 3,
                 CV_AA);
        cv::line(img,
                 cv::Point(width / 2 + size, height / 2 - size),
                 cv::Point(width / 2 - size, height / 2 + size),
                 cv::Scalar(0, 0, 255),
                 3,
                 CV_AA);
    };
}

void XMap::dashed_line(cv::Mat& img, int every, cv::Point p1, cv::Point p2) {
    const int segment_len = 10;
    CHECK_LT(every, 10);
    CHECK_GT(every, 0);
    cv::LineIterator it(
        img, p1, p2, 8 /* connectivity */);  // get a line iterator
    for (int i = 0; i < it.count; i++, it++) {
        if (i % segment_len <= every) {
            // gray
            (*it)[0] = 220;
            (*it)[1] = 220;
            (*it)[2] = 220;
        }
    }
}

cv::Mat XMap::image_centering(cv::Mat img_in, const Loc& item_loc) {
    int xa = item_loc.x;
    int ya = item_loc.y;
    int X = width_;
    int Y = height_;
    int xl = X - xa - 1;
    int xr = xa;
    int yt = Y - ya - 1;
    int yb = ya;
    cv::Mat img_out;
    cv::copyMakeBorder(img_in,
                       img_out,
                       yt * grid_size_,
                       yb * grid_size_,
                       xl * grid_size_,
                       xr * grid_size_,
                       cv::BORDER_CONSTANT,
                       cv::Scalar(0, 0, 0));
    return img_out;
}

cv::Rect XMap::image_masking(cv::Mat img_in,
                            const Loc& item_loc,
                            double yaw,
                            int visible_radius_unit) {
    CHECK_EQ(visible_radius_unit % 2, 1) << "Must be an odd int";

    int xa = item_loc.x + visible_radius_unit;
    int ya = item_loc.y + visible_radius_unit;

    auto dir = XItem::get_item_facing_dir(yaw);
    if (dir == "right") {
        xa += (visible_radius_unit + 1) / 2;
    } else if (dir == "up") {
        ya -= (visible_radius_unit + 1) / 2;
    } else if (dir == "left") {
        xa -= (visible_radius_unit + 1) / 2;
    } else {
        ya += (visible_radius_unit + 1) / 2;
    }

    int x_st = xa - visible_radius_unit / 2;
    int y_st = ya - visible_radius_unit / 2;

    return cv::Rect(x_st * grid_size_,
                    y_st * grid_size_,
                    visible_radius_unit * grid_size_,
                    visible_radius_unit * grid_size_);
}

}}  // namespace simulator::xwd
