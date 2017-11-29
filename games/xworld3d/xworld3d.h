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

#include <deque>
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
    MOVE_FORWARD,
    MOVE_BACKWARD,
    MOVE_LEFT,
    MOVE_RIGHT,
    TURN_LEFT,
    TURN_RIGHT,
    JUMP,
    COLLECT
};

typedef std::map<std::string, X3ItemPtr> IDItemMap;
typedef std::map<std::string, std::deque<X3ItemPtr>> ItemPoolOfOneKind;
typedef std::unique_ptr<roboschool::World> WorldPtr;

/**
 * An container that recycles items removed from X3World.
 * 
 * Everytime we remove an item from X3World, the X3Item directly goes to this
 * container instead of being deleted from memory. Next time when X3World need
 * an item of some kind, it will first try to reuse one from container.
 * 
 * The items in the container are organized through a 2-level mapping. The first
 * level key is item type (e.g., goal, blocks), and the second level key is item
 * name (e.g., apple, cat).
 */
class X3ItemPool {
public:
    X3ItemPool();
    
    /**
     * Reuse an item from the container.
     *
     * The type and name of the item are specified in e. If not such item is
     * available, a new one will be created.
     */
    X3ItemPtr get_item(const Entity& e, const WorldPtr& world);
    
    /**
     * Put removed item into the container for later usage.
     */
    void recycle_item(const X3ItemPtr& item);

    size_t size() { return size_; }

private:
    std::map<std::string, ItemPoolOfOneKind> item_pool_;
    size_t size_; // the number of items in the container
};

class X3Stadium {
public:
    X3Stadium() { olist_.clear(); };

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

    ~X3World();

    /**
     * Reset the 3D environment.
     *
     * This function is called from two places. One place is where an
     * environment update is request (when map_reset is false). The function 
     * will get the updated entity information from thy python object 
     * XWorld3DEnv (defined in xworld3d_env.py).
     * 
     * Another place is where we reset the game (when map_reset is true). Before 
     * getting the update from XWorld3dEnv, the items in current environment 
     * except X3Stadium will all be removed.
     */
    void reset_world(bool map_reset = true);

    int height() const { return height_; }

    int width() const { return width_; }

    int img_height() const { return img_height_; }

    int img_width() const { return img_width_; }

    std::string conf_file() { return conf_; }

    boost::python::object get_py_env() { return xwd_env_; }

    void get_entities(std::vector<Entity>& entities);

    bool act(const size_t agent_id, const size_t action);

    roboschool::RenderResult render(const size_t agent_id, bool debug);

    void step(const int frame_skip);

private:
    void clear_world();

    /**
     * Update items whose entity informations are changed by the python object 
     * XWorld3DTask.
     * 
     * @param   entities[in]    updated entity information
     */
    void update_world(const std::vector<Entity>& entities);

    bool apply_action(const X3ItemPtr& item, const size_t action);

    void add_item(const Entity& e);

    void remove_item(X3ItemPtr& item);

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
    X3ItemPool item_pool_;
};

}} // simulator::xworld3d
