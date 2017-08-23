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

#pragma once
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "xitem.h"

namespace simulator { namespace xwd {

struct AgentColors {
    uint8_t b;
    uint8_t g;
    uint8_t r;
};

static const std::vector<AgentColors>
agent_colors({{0,0,255}, {255,0,0}, {0,255,0},
              {255,255,0}, {0,255,255}, {255,0,255}, {0,0,0}});

/**
*  class for organizing all the items in XWorld
*
*/

class XMap {

public:

    // constructor
    XMap();
    XMap(int height, int width);

    // destructor
    virtual ~XMap();

    /**
    *  Function to get an image representation of all the items in map.
    *  @param goal_names: a set of goal names to be highlighted for visualization
    *  @param flag_item_centric: flag for item centric view or not (item specified by item_name)
    *  @param item_name:  the name of the item for centering the view if flag_item_centric is true
    *  @param flag_grid_lines: flag for drawing the grid lines in xworld
    *  @param success: draw a success sign if positive and failure sign otherwise
    *  @param visible_radius_unit: radius of the visible range for the agent in terms of xworld unit;
    *                      zero for full observation
    *  @param flag_crop_receiptive_field: flag for whether to crop only the visible region (for learning)
    *                             or keep the full image uncropped but masked (for visualization)
    */
    cv::Mat to_image(const std::vector<std::vector<std::string>>& goal_names,
                     bool flag_item_centric, std::string item_name,
                     bool flag_illustration, int success,
                     int visible_radius_unit, bool flag_crop_receiptive_field);

    /**
    *  Function to generate a partially observed view for an input image
    *  @param img_in: input image representing the full view
    *  @param item_name: the name of the item that specifies the center of the mask
    *  @param visible_radius_unit: radius of the visible range for the agent
    *  @param mask_value: value to be filled for the invisible region
    *  @param flag_crop_receipt_field: flag for whether to crop only the visible region (for learning)
    *                          or keep the full image uncropped but masked (for visualization)
    */
    cv::Mat image_masking(cv::Mat img_in, std::string item_name, int visible_radius_unit,
                          int mask_value, bool flag_crop_receiptive_filed = false);

    // add an item to the map
    void add_item(XItem* item_ptr);

    // remove an item from the map
    void remove_item(XItem* item_ptr);

    // add a list of items for the map
    void add_items(const std::vector<XItem*> & item_list);

    // remove a list of items from the map
    void remove_items(const std::vector<XItem*> & item_list);

    // get the location for the item
    Loc get_item_location(std::string item_name);

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
    std::vector<std::vector<std::vector<XItem*> > > item_ptr_cube_;

    // length of the grid in terms of pixels (same for both horizontal and vertical directions)
    int grid_size_;

    void init(int height, int width);

    /**
       @param success >0 for success, otherwise for failure
    */
    void draw_success(cv::Mat& img, int success);

    void dashed_line(cv::Mat& img, int every, cv::Point p1, cv::Point p2);

    // center image with respect to the item specified by item_name
    cv::Mat image_centering(cv::Mat img_in, std::string item_name);
};

}} //namespace xworld
