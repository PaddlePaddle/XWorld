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

#include <map>
#include <memory>
#include <vector>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "x3item.h"
#include "xworld3d_flags.h"

namespace simulator {
namespace xworld3d {

enum X3NavAction {
    MOVE_FORWARD = 0,
    MOVE_BACKWARD = 1,
    TURN_LEFT = 2,
    TURN_RIGHT = 3,
    JUMP = 4,
    COLLECT = 5
};

typedef std::map<std::string, X3ItemPtr> IDItemMap;
typedef std::unique_ptr<roboschool::World> WorldPtr;

class X3Stadium {
public:
    X3Stadium() {};

    void load_stadium(const std::string& item_path, const WorldPtr& world);

private:
    static const int STADIUM_SCALE = 0.01f;

    roboschool::Thingy floor_; // prevent the pointer to thingy being
                               // released
    std::vector<roboschool::Object> olist_;
};

class X3World {
public:
    X3World(const std::string& conf, bool print_conf, bool big_screen);

    X3World(const X3World&)  = delete;

    void reset_world(bool map_reset = true);

    void clear_world();

    int height() const { return height_; }

    int width() const { return width_; }

    int img_height() const { return img_height_; }

    int img_width() const { return img_width_; }

    void add_item(const Entity& e);

    std::string conf_file() { return conf_; }

    boost::python::object get_py_env() { return xwd_env_; }

    void get_entities(std::vector<Entity>& entities);

    bool act(const size_t agent_id, const int a);

    roboschool::RenderResult render(const size_t agent_id, bool debug);

    void step(const int frame_skip);

private:
    void build_world(const std::vector<Entity>& entities);

    bool apply_action(const X3ItemPtr& item, const int a);

    void remove_item(X3ItemPtr item);

    std::string conf_;
    int height_;         // world size
    int width_;          // world size
    int img_height_;     // size for opengl rendering (used for show and debug)
    int img_width_;      // size for opengl rendering (used for show and debug)
    std::string item_path_;
    std::string map_;

    // large sizes will crash in camera render
    static const int IMG_HEIGHT_SHOW = 512;
    static const int IMG_WIDTH_SHOW = 512;

    IDItemMap items_;
    std::vector<X3ItemPtr> agents_;
    std::unique_ptr<X3Camera> camera_;

    WorldPtr world_;
    X3Stadium stadium_; // later we can extend it to handle more complex terrain
    boost::python::object xwd_env_;
};

}} // simulator::xworld_3