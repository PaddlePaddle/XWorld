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
#include <iostream>
#include <memory>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sstream>
#include <unordered_set>

#include "xitem.h"
#include "xmap.h"

namespace simulator {
namespace xwd {

class XWorld {
  public:
    XWorld(const std::string& conf, bool print_conf);

    void reset(bool map_reset = true);

    // get an image representation of the world
    // the world for the specified item
    // flag_active_goal=true: generate an image with the specified goal
    // highlighted
    cv::Mat to_image(int agent_id = 0,
                     bool flag_illustration = true,
                     int visible_radius_unit = 0);

    // agent specified by agent_ptr takes action action_id
    // return whether the action is successful or not
    bool act(int agent_id, int action_id);

    // get number of actions
    int get_num_actions();

    boost::python::object get_py_env();

    void get_entities(std::vector<Entity>& entities);

    int height() { return height_; }

    int width() { return width_; }

    std::string conf_file() { return conf_; }

  private:
    std::string conf_;
    XMap map_;
    int height_;
    int width_;
    boost::python::object xwd_env_;
    std::vector<std::shared_ptr<XItem>> item_list_;    // list of all items, including agents
    std::vector<std::shared_ptr<XItem>> agent_list_;   // list of all agents,
};
}
}  // namespace simulator::xwd
