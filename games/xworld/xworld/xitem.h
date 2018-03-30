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
#include <glog/logging.h>
#include <memory>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>
#include <unordered_map>
#include <vector>
#include "simulator_util.h"
#include "simulator_entity.h"

namespace simulator {
namespace xwd {

enum XWorldAction {
    MOVE_UP = 0,
    MOVE_DOWN,
    MOVE_LEFT,
    MOVE_RIGHT,
    MOVE_FORWARD,
    MOVE_BACKWARD,
    MOVE_LEFT_FPV,
    MOVE_RIGHT_FPV,
    TURN_LEFT,
    TURN_RIGHT
};

struct Loc {
    int x;
    int y;
    Loc() { init(); }
    Loc(int _x, int _y) : x(_x), y(_y) {}
    Loc(const std::vector<int>& loc) {
        CHECK(loc.empty() || loc.size() == 2);
        if (!loc.empty()) {
            x = loc[0];
            y = loc[1];
        } else {
            init();
        }
    }
    void init() {
        x = std::numeric_limits<int>::min();
        y = std::numeric_limits<int>::min();
    }
    std::string to_string() const {
        return "(" + std::to_string(x) + "," + std::to_string(y) + ")";
    }
    bool defined() const {
        return x != std::numeric_limits<int>::min() &&
               y != std::numeric_limits<int>::min();
    }
    void random_loc(int w, int h) {
        x = int(w * util::get_rand_range_val(1.0));
        y = int(h * util::get_rand_range_val(1.0));
    }
    bool in_boundary(int w, int h) const {
        return x >= 0 && x < w && y >= 0 && y < h;
    }
    bool operator>=(const Loc& l) const { return x >= l.x && y >= l.y; }
    bool operator==(const Loc& l) const { return x == l.x && y == l.y; }
    bool operator!=(const Loc& l) const { return x != l.x || y != l.y; }
    Loc operator-(const Loc& l) const { return {x - l.x, y - l.y}; }
    Loc operator+(const Loc& l) const { return {x + l.x, y + l.y}; }
    double square_distance(const Loc& l) const {
        return (x - l.x) * (x - l.x) + (y - l.y) * (y - l.y);
    }
};
}
}  // namespace simulator::xwd

namespace std {
template <>
struct hash<simulator::xwd::Loc> {
    size_t operator()(const simulator::xwd::Loc& l) const {
        return hash<int>()(l.x) ^ hash<int>()(l.y);
    }
};
}

namespace simulator {
namespace xwd {

class XAgent;

/**
*  class as the basic building block in XWorld
*
*/
class XItem {
  public:
    XItem(const Entity& e) : e_(e) {}

    // destructor
    virtual ~XItem() {}

    // get the type of the item
    std::string get_item_type() { return e_.type; }

    // get the name of the item
    std::string get_item_id() { return e_.id; }

    // get the color of the item
    std::string get_item_color() { return e_.color; }

    // get the icon image of the item
    cv::Mat get_item_image();

    // get the location of the item
    Loc get_item_location() { return Loc(int(e_.loc.x), int(e_.loc.y)); }

    double get_item_yaw() { return e_.yaw; }

    // set the location of the item
    void set_item_location(int x, int y) { e_.loc = Vec3(x, y, 0); }

    // set the type of the item
    void set_item_type(const std::string& item_type) { e_.type = item_type; }

    // whether the item is reachable
    bool is_reachable() { return get_item_type() != "block"; }

    Entity entity() { return e_; }

    virtual Loc act(int action_id) {
        LOG(FATAL) << "actions not defined!";
        return Loc();
    }

    virtual int get_num_actions() { return 0; }

    // The original size of the item image stored on disk
    // This is also the shown grid size of xworld in OpenCV if no
    // resize of the GUI window is applied
    static const int item_size_ = 64;

    static std::unordered_map<std::string, cv::Mat> item_imgs_;

    static std::shared_ptr<XItem> create_item(const Entity& e);

    static std::string get_item_facing_dir(double yaw);

  protected:
    Entity e_;
};

typedef std::shared_ptr<XItem> XItemPtr;

class XAgent : public XItem {
  public:
    // constructor
    XAgent(const Entity& e);

    // perform action specified by action_id
    Loc act(int action_id) override;

    int get_num_actions() override {
        return legal_actions_.size();
    }

  private:
    std::vector<size_t> legal_actions_;
};

}
}  // namespace simulator::xwd
