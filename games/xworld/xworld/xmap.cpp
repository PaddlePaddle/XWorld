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
        std::vector<bool> shadow;
        auto roi_ori = image_masking(item_loc, yaw, visible_radius_unit, shadow);

        // pad the original image
        cv::copyMakeBorder(world,
                           world,
                           visible_radius_unit * grid_size_,
                           visible_radius_unit * grid_size_,
                           visible_radius_unit * grid_size_,
                           visible_radius_unit * grid_size_,
                           cv::BORDER_CONSTANT,
                           cv::Scalar(0, 0, 0));
        auto roi = cv::Rect(roi_ori.x * grid_size_,
                            roi_ori.y * grid_size_,
                            roi_ori.width * grid_size_,
                            roi_ori.height * grid_size_);

        world(roi).copyTo(view);

        // render shadow in the agent's view
        for (int x = 0; x < roi_ori.width; x ++) {
            for (int y = 0; y < roi_ori.height; y ++) {
                if (shadow[y * roi_ori.width + x]) {
                    cv::Mat black(grid_size_, grid_size_, CV_8UC3, cv::Scalar(0, 0, 0));
                    auto grid = view(cv::Rect(x * grid_size_, y * grid_size_, grid_size_, grid_size_));
                    if (flag_illustration) {
                        cv::addWeighted(black, 0.7, grid, 0.3, 0.0, grid);
                    } else {
                        black.copyTo(grid);
                    }
                }
            }
        }

        if (flag_illustration) {
            cv::Mat black(world.rows, world.cols, CV_8UC3, cv::Scalar(0, 0, 0));
            cv::addWeighted(black, 0.7, world, 0.3, 0.0, world);
            view.copyTo(world(roi));
            view = world;
        } else { // rotate the image according to the agent's yaw
            cv::Point2f center(view.cols / 2.0, view.rows / 2.0);
            cv::Mat rot_mat = cv::getRotationMatrix2D(center, 180 - yaw * 180 / M_PI, 1.0);
            cv::warpAffine(view, view, rot_mat, view.size());
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

cv::Rect XMap::image_masking(const Loc& item_loc,
                             double yaw,
                             int visible_radius_unit,
                             std::vector<bool>& shadow) {
    CHECK_EQ(visible_radius_unit % 2, 1) << "Must be an odd int";

    int xa = item_loc.x + visible_radius_unit;
    int ya = item_loc.y + visible_radius_unit;

    int major_inc_x = 0;
    int major_inc_y = 0;
    int minor_inc_x = 0;
    int minor_inc_y = 0;
    int scan_x = 0;
    int scan_y = 0;

    auto dir = XItem::get_item_facing_dir(yaw);
    if (dir == "right") {
        xa += (visible_radius_unit + 1) / 2;
        major_inc_y = 1;
        minor_inc_x = 1;
    } else if (dir == "up") { //
        ya -= (visible_radius_unit + 1) / 2;
        major_inc_x = 1;
        minor_inc_y = -1;
        scan_y = visible_radius_unit - 1;
    } else if (dir == "left") { //
        xa -= (visible_radius_unit + 1) / 2;
        major_inc_y = 1;
        minor_inc_x = -1;
        scan_x = visible_radius_unit - 1;
    } else {
        ya += (visible_radius_unit + 1) / 2;
        major_inc_x = 1;
        minor_inc_y = 1;
    }

    int x_st = xa - visible_radius_unit / 2;
    int y_st = ya - visible_radius_unit / 2;

    // compute which grids the agent's ray can start going forward
    std::vector<bool> ray_starts(visible_radius_unit, true);
    for (int o = -1; o <= 1; o += 2) {
        bool block = false;
        int ray_x = item_loc.x;
        int ray_y = item_loc.y;
        for (int k = 1; k <= visible_radius_unit / 2; k ++) {
            ray_x += o * major_inc_x;
            ray_y += o * major_inc_y;
            if (ray_x >= 0 && ray_x < width_ && ray_y >= 0 && ray_y < height_
                && item_ptr_cube_[ray_y][ray_x].size()
                && item_ptr_cube_[ray_y][ray_x][0]->get_item_type() == "block") {
                block = true;
            }
            if (block) {
                ray_starts[ray_starts.size() / 2 + o * k] = false;
            }
        }
    }


    // compute shadow grids due to the occulusion of the wall blocks
    shadow = std::vector<bool>(visible_radius_unit * visible_radius_unit, false);
    for (int k = 0; k < visible_radius_unit; k ++) { // major order
        bool block = !ray_starts[k];
        int cur_x = scan_x;
        int cur_y = scan_y;
        for (int j = 0; j < visible_radius_unit; j ++) { // minor order
            if (block) {
                shadow[cur_y * visible_radius_unit + cur_x] = true;
            }
            int g_x = x_st - visible_radius_unit + cur_x;
            int g_y = y_st - visible_radius_unit + cur_y;
            if (g_x >= 0 && g_x < width_ && g_y >= 0 && g_y < height_
                && item_ptr_cube_[g_y][g_x].size()
                && item_ptr_cube_[g_y][g_x][0]->get_item_type() == "block") {
                block = true;
            }
            cur_x = (cur_x + minor_inc_x + visible_radius_unit) % visible_radius_unit;
            cur_y = (cur_y + minor_inc_y + visible_radius_unit) % visible_radius_unit;
        }
        scan_x += major_inc_x;
        scan_y += major_inc_y;
    }

    return cv::Rect(x_st,
                    y_st,
                    visible_radius_unit,
                    visible_radius_unit);
}

}}  // namespace simulator::xwd
