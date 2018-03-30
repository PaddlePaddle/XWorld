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
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>

#include "xitem.h"

namespace simulator {
namespace xwd {

/**
*  class for organizing all the items in XWorld
*
*/

class XMap {
  public:
    // constructor
    XMap();
    XMap(int height, int width);

    /**
    *  Function to get an image representation of all the items in map.
    *  @param item_loc:  the location of the item for centering the view
    *  @param flag_illustration: whether for visualisation or training
    *  @param visible_radius_unit: radius of the visible range for the agent in
    * terms of xworld unit;
    *                      zero for full observation
    *  @param flag_crop_receiptive_field: flag for whether to crop only the
    * visible region (for learning)
    *                             or keep the full image uncropped but masked
    * (for
    * visualization)
    */
    cv::Mat to_image(const Loc& item_loc,
                     double yaw,
                     bool flag_illustration,
                     int visible_radius_unit);

    /**
    *  Function to generate a partially observed view for an input image
    *  @param img_in: input image representing the full view
    *  @param item_loc: the location of the item that specifies the center of the
    * mask
    *  @param visible_radius_unit: radius of the visible range for the agent
    */
    cv::Rect image_masking(const Loc& item_loc,
                           double yaw,
                           int visible_radius_unit,
                           std::vector<bool>& shadow);

    // add an item to the map
    void add_item(XItemPtr item_ptr);

    // remove an item from the map
    void remove_item(XItemPtr item_ptr);

    // add a list of items for the map
    void add_items(const std::vector<XItemPtr>& item_list);

    // remove a list of items from the map
    void remove_items(const std::vector<XItemPtr>& item_list);

    // move an item to a target location
    bool move_item(XItemPtr item, const Loc& target);

    // get the location for the item
    Loc get_item_location(std::string item_id);

    // check if the specified location is reachable
    bool is_reachable(int x, int y) const;

    // check if the specified location is empty (no item)
    bool is_empty(int x, int y);

  private:
    // size of the world
    int height_;
    int width_;

    // 3d matrix containing all the items in the 2D world
    // Haonan: could be improved with a sparse representation
    std::vector<std::vector<std::vector<XItemPtr>>> item_ptr_cube_;

    // length of the grid in terms of pixels (same for both horizontal and
    // vertical directions)
    int grid_size_;

    void init(int height, int width);

    /**
       @param success >0 for success, otherwise for failure
    */
    void draw_success(cv::Mat& img, int success);

    void dashed_line(cv::Mat& img, int every, cv::Point p1, cv::Point p2);

    // center image with respect to the item specified by item_loc
    cv::Mat image_centering(cv::Mat img_in, const Loc& item_loc);
};
}
}  // namespace xworld
