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

#include <algorithm>
#include <fstream>
#include <limits>
#include <math.h>
#include <random>
#include <utility>

#include <boost/algorithm/string.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/exception/diagnostic_information.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "xworld3d.h"

namespace simulator {
namespace xworld3d {

namespace pt = boost::property_tree;
namespace py = boost::python;

void X3Stadium::load_stadium(const std::string& item_path,
                             const WorldPtr& world) {
    std::string floor_file = item_path + "/floor/floor.xml";
    std::string stadium_file = item_path + "/floor/stadium/stadium1.obj";

    olist_ = world->load_mjcf(floor_file);
    for (auto& o : olist_) {
        o.query_position();
    }

    roboschool::Pose stadium_pose(0.0f, 0.0f, 0.0f);
    floor_ = world->load_thingy(stadium_file,
                                stadium_pose,
                                X3Stadium::STADIUM_SCALE,
                                0.0f, // mass: 0 means that the stadium is
                                      // non-movable
                                0xffffff,
                                true);
}

X3World::X3World(const std::string& conf, bool print_conf, bool big_screen) :
        // if big_screen=true, we don't care about rendering speed;
        // otherwise the dimensions should be the same with the training input image
        conf_(conf),
        img_height_(big_screen? IMG_HEIGHT_SHOW: FLAGS_x3_training_img_height),
        img_width_(big_screen? IMG_WIDTH_SHOW: FLAGS_x3_training_img_width) {
    if (print_conf) {
        std::ifstream infile(conf_);
        std::string line;
        while (std::getline(infile, line)) {
            LOG(INFO) << line;
        }
    }

    std::unique_ptr<pt::ptree> tree;
    tree.reset(new pt::ptree);
    try {
        pt::read_json(conf_, *(tree.get()));
    } catch (const boost::exception& ex) {
        LOG(FATAL) << "world config file error: "
                   << boost::diagnostic_information(ex);
    }

    CHECK_GT(tree->count("item_path"), 0);
    CHECK_GT(tree->count("map"), 0);
    item_path_ = tree->get<std::string>("item_path");
    map_ = tree->get<std::string>("map");

    CHECK(Py_IsInitialized());

    try {
        std::string f = __FILE__;
        std::string path = f.substr(0, f.find_last_of("/") + 1);
        auto main_mod = py::import("__main__");
        auto main_namespace = main_mod.attr("__dict__");
        py::exec("import sys", main_namespace);
        std::string cmd = "sys.path.append(\"" + path + "maps\")";
        py::exec(cmd.c_str(), main_namespace);

        auto mod = py::import(map_.c_str());
        item_path_ = path + item_path_;
        xwd_env_ = mod.attr(map_.c_str())(item_path_.c_str());
    } catch (...) {
        PyErr_Print();
        LOG(FATAL) << "Error loading map: " << map_;
    }

    world_ = util::make_unique<roboschool::World>(
            FLAGS_x3_gravity, FLAGS_x3_time_step);
    world_->set_glsl_path(FLAGS_x3_glsl_path);

    camera_ = util::make_unique<X3Camera>(
            *(world_.get()), img_height_, img_width_);
}

void X3World::reset_world(bool map_reset) {
    std::vector<Entity> entities;
    try {
        if (map_reset) {
            // regenerate a xwd map
            xwd_env_.attr("reset")();
        }

        py::tuple dims = py::extract<py::tuple>(xwd_env_.attr("get_dims")());
        height_ = py::extract<int>(dims[0]);
        width_ = py::extract<int>(dims[1]);

        py::list py_entities = py::extract<py::list>(
                xwd_env_.attr("cpp_get_entities")());

        for (int i = 0; i < py::len(py_entities); i ++) {
            py::dict py_e = py::extract<py::dict>(py_entities[i]);
            entities.emplace_back(py_e);
        }
    } catch (...) {
        PyErr_Print();
        LOG(FATAL) << "Error resetting world";
    }

    clear_world();
    build_world(entities);
}

void X3World::clear_world() {
    items_.clear();
    agents_.clear();
    camera_->detach();
    world_->clean_everything();
}

void X3World::build_world(const std::vector<Entity>& entities) {
    stadium_.load_stadium(item_path_, world_);

    for (auto const& e : entities) {
        add_item(e);
    }

    CHECK_GT(agents_.size(), 0) << "There should be at least one agent.";
}

void X3World::add_item(const Entity& e) {
    CHECK(items_.find(e.id) == items_.end())
            << e.type << " " << e.id << " exists.";

    // TODO: check overlapping with existing items

    auto item_ptr = X3Item::create_item(e, *world_);
    items_[e.id] = item_ptr;
    if (e.type == "agent") {
        agents_.push_back(item_ptr);
    }
}

void X3World::get_entities(std::vector<Entity>& entities) {
    entities.clear();
    for (auto& i : items_) {
        entities.push_back(i.second->entity());
    }
}

bool X3World::act(const size_t agent_id, const int a) {
    CHECK_LT(agent_id, agents_.size());
    auto agent_ptr = agents_[agent_id];
    return apply_action(agent_ptr, a);
}

bool X3World::apply_action(const X3ItemPtr& item, const int a) {
    bool action_success = true;
    switch (a) {
        case X3NavAction::MOVE_FORWARD:
            item->move_forward();
            break;
        case X3NavAction::MOVE_BACKWARD:
            item->move_backward();
            break;
        case X3NavAction::TURN_LEFT:
            item->turn_left();
            break;
        case X3NavAction::TURN_RIGHT:
            item->turn_right();
            break;
        case X3NavAction::JUMP:
            item->jump();
            break;
        case X3NavAction::COLLECT: {
            X3ItemPtr goal = item->collect_item(items_, "goal");
            if (goal) {
                remove_item(goal);
            } else {
                action_success = false;
            }
            break;
        }
        default:
            action_success = false;
            LOG(ERROR) << "unknown action id: " << a;
    }
    return action_success;
}

void X3World::remove_item(X3ItemPtr item) {
    CHECK_NE(item->type(), "agent");
    auto it = items_.find(item->id());
    if (it != items_.end()) {
        auto& i = it->second;
        // TODO: remove object from bullet physics
        //world_->remove_object(i->object());
        //i->destroy();
        i->move_underground();
        items_.erase(it);
    }
}

roboschool::RenderResult X3World::render(const size_t agent_id, bool debug) {
    CHECK_LT(agent_id, agents_.size());
    return camera_->render(agents_[agent_id].get(), debug);
}

void X3World::step(const int frame_skip) {
    world_->step(frame_skip);
}

}} // simulator::xworld3d
