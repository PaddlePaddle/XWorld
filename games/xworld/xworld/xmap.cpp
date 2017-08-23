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

#include "xmap.h"
#include "xitem.h"
#include "xagent.h"

namespace simulator { namespace xwd {
XMap::XMap() {
    init(0, 0);
}

XMap::XMap(int height, int width) {
    init(height, width);
}

void XMap::init(int height, int width) {

    this->height_ = height;
    this->width_ = width;

    this->grid_size_ = XItem::get_item_size();
    this->item_ptr_cube_.resize(this->height_);
    for (int i = 0; i < this->height_; i++) {
        this->item_ptr_cube_[i].resize(this->width_);
    }
}

void XMap::add_items(const std::vector<XItem*> & item_list) {
    for (unsigned int i = 0; i < item_list.size(); i++) {
        this->add_item(item_list[i]);
    }
}

void XMap::remove_items(const std::vector<XItem*> & item_list) {
    for (unsigned int i = 0; i < item_list.size(); i++) {
        this->remove_item(item_list[i]);
    }
}

void XMap::add_item(XItem* item_ptr) {
    Loc loc = item_ptr->get_item_location();
    bool exist_flag = false;
    auto& items = item_ptr_cube_[loc.y][loc.x];
    for (unsigned int i = 0; i < items.size(); i++) {
        if (item_ptr->get_item_name() == items[i]->get_item_name()) {
            exist_flag = true;
        }
    }
    if (!exist_flag) {
        items.push_back(item_ptr);
    }
}

void XMap::remove_item(XItem* item_ptr) {
    Loc loc = item_ptr->get_item_location();
    auto& items = item_ptr_cube_[loc.y][loc.x];
    for (unsigned int i = 0; i < items.size(); i++) {
        if (item_ptr->get_item_name() == items[i]->get_item_name()) {
            items.erase(items.begin() + i);
            break;
        }
    }
}

bool XMap::is_reachable(int x, int y) const {
    if (x < 0 || y < 0 || x >= this->width_ || y >= this->height_)
        return false;

    bool is_reachable_flag = true;
    for (unsigned int i = 0; i < this->item_ptr_cube_[y][x].size(); i++) {
        is_reachable_flag = is_reachable_flag && this->item_ptr_cube_[y][x][i]->is_reachable();
    }
    return is_reachable_flag;
}

bool XMap::is_empty(int x, int y) {
    if (x < 0 || y < 0 || x >= this->width_ || y >= this->height_)
        return false;
    return this->item_ptr_cube_[y][x].size()>0? false : true;
}

Loc XMap::get_item_location(std::string item_name) {
    unsigned int nr = item_ptr_cube_.size();
    unsigned int nc = item_ptr_cube_[0].size();
    Loc loc = {2, 0};
    for (unsigned int i = 0; i < nr; i++) {
        for (unsigned int j = 0; j < nc; j++) {
            for (unsigned int k = 0; k < item_ptr_cube_[i][j].size(); k++) {
                if (item_ptr_cube_[i][j][k]->get_item_name() == item_name) {
                    loc = item_ptr_cube_[i][j][k]->get_item_location();
                    return loc;
                }
            }
        }
    }
    return loc;
}

cv::Mat XMap::to_image(const std::vector<std::vector<std::string>>& goal_names,
                       bool flag_item_centric, std::string item_name,
                       bool flag_illustration, int success,
                       int visible_radius_unit, bool flag_crop_receiptive_field) {
    cv::Mat world(this->height_*this->grid_size_,
                  this->width_*this->grid_size_,
                  CV_8UC3, cv::Scalar(255, 255, 255));

    for (int i = 0; i < this->height_; i ++) {
        for (int j = 0; j < this->width_; j ++) {
            for (unsigned int k = 0; k < item_ptr_cube_[i][j].size(); k ++) {
                // when training, don't render the agent in the image
                if (!flag_illustration
                    && item_ptr_cube_[i][j][k]->get_item_type() == AGENT)
                    continue;

                cv::Mat item_image = item_ptr_cube_[i][j][k]->get_item_image();

                cv::Mat item_image_active;
                item_image.copyTo(item_image_active);

                // render goals
                for (size_t a = 0; a < goal_names.size(); a++) {  // which agent
                    for (size_t g = 0; g < goal_names[a].size(); g++) {  // which valid goal for this agent
                        if (item_ptr_cube_[i][j][k]->get_item_type() == GOAL
                            && item_ptr_cube_[i][j][k]->get_item_name() == goal_names[a][g]) {
                            int thickness = 2;
                            cv::Rect border(cv::Point(0, 0), item_image_active.size());
                            CHECK_LT(a, agent_colors.size()) << "You need define more agent colors";
                            cv::Scalar color(agent_colors[a].b,
                                             agent_colors[a].g,
                                             agent_colors[a].r);
                            cv::rectangle(item_image_active, border, color, thickness);
                        }
                    }
                }
                cv::Mat imageROI = world(cv::Rect(j*this->grid_size_, i*this->grid_size_,
                                                 this->grid_size_, this->grid_size_));
                item_image_active.copyTo(imageROI);
            }
        }
    }

    if (flag_illustration) {
        for (int i = 1; i < this->height_; i++) {
            dashed_line(world, 4,
                        cv::Point(0,i*this->grid_size_),
                        cv::Point(world.cols-1, i*this->grid_size_));
        }
        for (int i = 1; i < this->width_; i++) {
            dashed_line(world, 4,
                        cv::Point(i*this->grid_size_,0),
                        cv::Point(i*this->grid_size_,world.rows-1));
        }
    }

    if (visible_radius_unit > 0) {
        int mask_value = 0;
        world = image_masking(world, item_name, visible_radius_unit,
                              mask_value, flag_crop_receiptive_field);
    }
    else if (flag_item_centric) {
        world = image_centering(world, item_name);
    }

    if (success != 0) {
        draw_success(world, success);
    }

    return world;
}

void XMap::draw_success(cv::Mat& img, int success) {
    int height = img.rows;
    int width = img.cols;
    int radius = std::min(height,width) / 3;
    if (success > 0) {
        cv::circle(img, cv::Point(width/2,height/2), radius,
                   cv::Scalar(0,255,0), 3, CV_AA);
    } else {
        int size = int(0.7 * radius);  // sqrt(2)
        cv::line(img,
                 cv::Point(width/2-size, height/2-size),
                 cv::Point(width/2+size, height/2+size),
                 cv::Scalar(0,0,255),
                 3, CV_AA);
        cv::line(img,
                 cv::Point(width/2+size, height/2-size),
                 cv::Point(width/2-size, height/2+size),
                 cv::Scalar(0,0,255),
                 3, CV_AA);
    };
}

void XMap::dashed_line(cv::Mat& img, int every, cv::Point p1, cv::Point p2) {
    const int segment_len = 10;
    CHECK_LT(every, 10);
    CHECK_GT(every, 0);
    cv::LineIterator it(img, p1, p2,
                        8 /* connectivity */);            // get a line iterator
    for (int i = 0; i < it.count; i++, it++) {
        if (i%segment_len <= every) {
            // gray
            (*it)[0] = 220;
            (*it)[1] = 220;
            (*it)[2] = 220;
        }
    }
}

cv::Mat XMap::image_centering(cv::Mat img_in, std::string item_name) {
    Loc loc = get_item_location(item_name);
    int xa = loc.x;
    int ya = loc.y;
    int X = this->width_;
    int Y = this->height_;
    int xl = X - xa-1;
    int xr = xa;
    int yt = Y-ya-1;
    int yb = ya;
    cv::Mat img_out;
    copyMakeBorder(img_in, img_out,
                   yt * grid_size_,
                   yb * grid_size_,
                   xl * grid_size_,
                   xr * grid_size_,
                   cv::BORDER_CONSTANT,
                   cv::Scalar( 0, 0, 0));
    return img_out;
}

cv::Mat XMap::image_masking(cv::Mat img_in, std::string item_name, int visible_radius_unit,
                            int mask_value, bool flag_crop_receiptive_field) {
    Loc loc = get_item_location(item_name);
    int xa = loc.x;
    int ya = loc.y;
    cv::Mat img_partial;
    if (!flag_crop_receiptive_field) {
      cv::Mat img_p(height_*grid_size_, width_*grid_size_, CV_8UC3,
                    cv::Scalar(mask_value, mask_value, mask_value));
      img_partial = img_p;
    } else {
      cv::Mat img_p((2*visible_radius_unit+1)*grid_size_,
                    (2*visible_radius_unit+1)*grid_size_, CV_8UC3,
                    cv::Scalar(mask_value, mask_value, mask_value));
      img_partial = img_p;
    }
    int x_st = xa - visible_radius_unit;
    int y_st = ya - visible_radius_unit;
    int x_ed = xa + visible_radius_unit;
    int y_ed = ya + visible_radius_unit;

    int x_st_src = (x_st > 0) ? x_st : 0;
    int y_st_src = (y_st > 0) ? y_st : 0;
    int x_ed_src = (x_ed < this->width_-1)? x_ed : this->width_-1;
    int y_ed_src = (y_ed < this->height_-1)? y_ed : this->height_-1;

    int x_delta = x_st_src - x_st;
    int y_delta = y_st_src - y_st;
    cv::Rect rect_src = cv::Rect(x_st_src*grid_size_,
                                 y_st_src*grid_size_,
                                 (x_ed_src-x_st_src+1)*grid_size_,
                                 (y_ed_src-y_st_src+1)*grid_size_);
    cv::Rect rect_dst;
    if (!flag_crop_receiptive_field) {
      rect_dst = rect_src;
    } else {
      int x_st_dst = 0 + x_delta;
      int y_st_dst = 0 + y_delta;
      x_st_src = (x_st_src > 0) ? x_st_src : 0;
      y_st_src = (y_st_src > 0) ? y_st_src : 0;
      rect_dst = cv::Rect(x_st_dst*grid_size_,
                          y_st_dst*grid_size_,
                          (x_ed_src-x_st_src+1)*grid_size_,
                          (y_ed_src-y_st_src+1)*grid_size_);
    }
    cv::Mat imageROI_dst= img_partial(rect_dst);
    cv::Mat imageROI_src= img_in(rect_src);
    imageROI_src.copyTo(imageROI_dst);
    return img_partial;
}

XMap::~XMap() {
}

}} //namespace simulator::xwd
