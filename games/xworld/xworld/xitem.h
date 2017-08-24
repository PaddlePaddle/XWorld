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
#include <iostream>
#include <memory>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>
#include "simulator_util.h"

namespace simulator {
namespace xwd {

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

enum ItemType { GOAL, AGENT, BLOCK, DUMMY };

typedef struct {
    ItemType type;         // item type
    std::string name;      // item name (unique id)
    std::string img_path;  // item image path on the disk
    Loc location;          // item location
    std::string color;     // item color:
                           // "green"|"red"|"yellow"|"blue"|""
    bool active;  // for a BOX, active == true means whether it can be pushed
                  // ...
} ItemInfo;

typedef std::vector<ItemInfo> WorldUnitList;

class ItemRepository;

/**
*  class as the basic building block in XWorld
*
*/
class XItem {
  public:
    // active: set the active information for a particular item
    // this int is obtained from xworld_parser
    XItem(ItemType type,
          std::string name,
          Loc loc,
          std::string img_path = "",
          bool active = true,
          bool perturb = false);

    // destructor
    virtual ~XItem();

    // get the type of the item
    ItemType get_item_type();

    // get the name of the item
    std::string get_item_name();

    // get the color of the item
    std::string get_item_color();

    static bool item_color_defined(std::string color);

    // get the size of item
    static int get_item_size();

    // get the icon image of the item
    cv::Mat get_item_image();

    // get the location of the item
    Loc get_item_location();

    // set the location of the item
    void set_item_location(int x, int y);

    // set the type of the item
    void set_item_type(ItemType item_type);

    // whether the item is reachable
    bool is_reachable();

    bool get_active() { return active_; }

    // perform action specified by action_id
    // return true if action is successfully performed
    // return false if action is not successfully performed, for example,
    // because of obstacle.
    virtual bool act(int action_id) { return true; };

    // The original size of the item image stored on disk
    // This is also the shown grid size of xworld in OpenCV if no
    // resize of the GUI window is applied
    static const int item_size_ = 64;

    static ItemRepository item_rep_;

  private:
    ItemType type_;      // item type: e.g., agent, block, goal,...
    std::string name_;   // item name: e.g., agent1, block2, goal5...
    std::string color_;  // item color: "green"|"red"|"yellow"|"blue"|""
    Loc loc_;            // item location: [x , y]
    bool perturb_;       // whether apply random perturbation on the item
    struct {
        double angle;  // item rotation angle: value between 0 and 360
        double scale;  // item scale: value between 0 and 1
        double x;      // item horizontal shift: loc.x + transform.x * width
        double y;      // item vertical shift: loc.y + transform.y * height
    } perturb_transform_;
    std::string img_name_;  // item image file: path and name for the image file
                            // corresponding to the specific item (e.g., goal5)
    cv::Mat img_;           // item image: image for the item loaded
                            // from the above specified image path
    static int size_;       // item size: in terms of pixels
    // (same for both horizontal and vertical directions)

    void init(ItemType type,
              std::string name,
              Loc loc,
              std::string img_name,
              bool active,
              bool perturb);

    bool active_;  // the same meaning with the "active" field of ItemInfo
};

/**
   A class that reads and stores the hierarchy of the directory that contains
   all
   the item files of the game.
   It enables the configuration of specific classes of goals in the json file,
   e.g., "fruit", "animal" ...
 */
class ItemNameTree {
  public:
    std::unordered_map<std::string, std::shared_ptr<ItemNameTree>>
        nodes_;  // subdir names
    std::unordered_map<std::string, std::vector<std::string>>
        leaves_;  // icon names
                  // class -> instance abs path
    void insert_item(std::string item_name);
    std::shared_ptr<ItemNameTree>& insert_node(std::string node_name);
    void retrieve_item_classes(std::string node,
                               std::vector<std::string>& classes);
    std::string retrieve_item_path(const std::string& class_name);
    std::vector<std::string> retrieve_item_paths(const std::string& class_name);
};

/** class used to load all the items ONCE across all the games
    It should be a static member of the XItem class
 */
class ItemRepository {
  private:
    struct ItemProperty {
        std::vector<std::string> keys;
        std::vector<std::string> values;
    };

  public:
    ItemRepository();
    std::shared_ptr<ItemNameTree> item_tree_;
    std::unordered_map<std::string, cv::Mat> item_imgs_;
    std::unordered_map<std::string, ItemProperty> item_properties_;
    static const std::string img_path_;

    std::string retrieve_property(std::string name, std::string key);

  private:
    void get_all_item_names(std::shared_ptr<ItemNameTree>& root,
                            std::string dir);
    void preload_all_item_images(std::shared_ptr<ItemNameTree> root);
    void preload_all_item_properties();
};
}
}  // namespace simulator::xwd
