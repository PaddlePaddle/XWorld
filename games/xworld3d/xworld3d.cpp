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

DECLARE_string(curriculum_stamp);

namespace simulator {
namespace xworld3d {

namespace pt = boost::property_tree;
namespace py = boost::python;

X3ItemPool::X3ItemPool() : size_(0) {}

X3ItemPtr X3ItemPool::get_item(const Entity& e, const WorldPtr& world) {
    auto& pool = item_pool_[e.type];
    X3ItemPtr item = nullptr;
    if (pool.find(e.asset_path) != pool.end() && !pool[e.asset_path].empty()) {
        item = pool[e.asset_path].front();
        item->set_entity(e);
        pool[e.asset_path].pop_front();
        size_--;
    } else {
        item = X3Item::create_item(e, *world);
    }
    return item;
}

void X3ItemPool::recycle_item(const X3ItemPtr& item) {
    auto& pool = item_pool_[item->type()];
    pool[item->asset_path()].push_back(item);
    size_++;
}

void X3Stadium::load_stadium(const std::string& item_path,
                             const WorldPtr& world) {
    if (olist_.size()) {
        // stadium has already been loaded
        return;
    }
    std::string floor_file = item_path + "/floor/floor.xml";
    std::string stadium_file = item_path + "/floor/stadium/stadium0.obj";

    olist_ = world->load_mjcf(floor_file);
    for (auto& o : olist_) {
        o.query_position();
    }

    roboschool::Pose stadium_pose(0.0f, 0.0f, 0.0f);
    floor_ = world->load_thingy(stadium_file,
                                stadium_pose,
                                1.0,//X3Stadium::STADIUM_SCALE,
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
    std::string map = tree->get<std::string>("map");

    CHECK(Py_IsInitialized());

    try {
        std::string f = __FILE__;
        std::string path = f.substr(0, f.find_last_of("/") + 1);
        auto main_mod = py::import("__main__");
        auto main_namespace = main_mod.attr("__dict__");
        py::exec("import sys", main_namespace);
        std::string cmd = "sys.path.append(\"" + path + "maps\")";
        py::exec(cmd.c_str(), main_namespace);

        auto mod = py::import(map.c_str());
        item_path_ = path + item_path_;
        // read the curriculum record if possible
        if (FLAGS_curriculum_stamp != "") {
            int start_level = 0;
            std::ifstream infile(FLAGS_curriculum_stamp);
            infile >> start_level;
            xwd_env_ = mod.attr(map.c_str())(item_path_.c_str(), start_level);
        } else { // default start_level
            xwd_env_ = mod.attr(map.c_str())(item_path_.c_str());
        }
    } catch (...) {
        PyErr_Print();
        LOG(FATAL) << "Error loading map: " << map;
    }

    world_ = util::make_unique<roboschool::World>(
            img_height_, img_width_, FLAGS_x3_gravity, FLAGS_x3_time_step);

    std::string glsl_path = __FILE__;
    glsl_path = glsl_path.substr(0, glsl_path.find_last_of("/") + 1) + "glsl";

    world_->set_glsl_path(glsl_path);

    camera_ = util::make_unique<X3Camera>(
            *(world_.get()), img_height_, img_width_);
}

X3World::~X3World() {
    items_.clear();
    agents_.clear();
    world_->clean_everything();
    b3handle_to_id_.clear();
}

void X3World::reset_world(bool map_reset) {
    std::vector<Entity> entities;
    try {
        if (map_reset) {
            // regenerate an xwd map
            xwd_env_.attr("reset")();
            // reset the change flag
            CHECK(xwd_env_.attr("env_changed")());
            clear_world();
        }

        py::tuple dims = py::extract<py::tuple>(xwd_env_.attr("get_dims")());
        height_ = py::extract<int>(dims[0]);
        width_ = py::extract<int>(dims[1]);

        const int record_curriculum_period = 200;
        static int n_games = 0;
        if (FLAGS_curriculum_stamp != "" && (++n_games) % record_curriculum_period == 0) {
            int level = py::extract<int>(xwd_env_.attr("dump_curriculum_progress")());
            std::ofstream outfile(FLAGS_curriculum_stamp);
            outfile << level;
        }

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

    update_world(entities);
}

void X3World::clear_world() {
    // move all items underground to pretend they are gone
    for (auto& i : items_) {
        i.second->move_underground();
        item_pool_.recycle_item(i.second);
    }
    items_.clear();
    agents_.clear();
    // floor is not cleared
}

void X3World::update_world(const std::vector<Entity>& entities) {
    stadium_.load_stadium(item_path_, world_);

    auto found_item_in_entities = [&](const std::string& id)->bool {
        bool ret = false;
        for (const auto& e : entities) {
            if (e.id == id) {
                ret = true;
                break;
            }
        }
        return ret;
    };
    std::map<std::string, X3ItemPtr> tmp(items_);
    for (auto& i : tmp) {
        if (!found_item_in_entities(i.second->id())) {
            // If an item is not found among the udpated entities, that means
            // the item is removed by XWorld3dTask. Then we remove it from
            // X3World too.
            remove_item(i.second);
        }
    }

    for (auto const& e : entities) {
        if (items_.find(e.id) != items_.end()) {
            // Update an existing item
            items_[e.id]->set_entity(e);
        } else {
            // Create a new item
            add_item(e);
        }
    }

    CHECK_GT(agents_.size(), 0) << "There should be at least one agent.";
    // Call step to update the 3D environment.
    world_->step(1);
}

void X3World::add_item(const Entity& e) {
    CHECK(items_.find(e.id) == items_.end())
            << e.type << " " << e.id << " exists.";

    // TODO: check overlapping with existing items
    auto item_ptr = item_pool_.get_item(e, world_);
    b3handle_to_id_[item_ptr->b3handle()] = e.id;
    items_[e.id] = item_ptr;
    if (e.type == "agent") {
        agents_.push_back(item_ptr);
    }
}

X3ItemPtr& X3World::get_agent(const size_t agent_id) {
    CHECK_LT(agent_id, agents_.size());
    return agents_[agent_id];
}

void X3World::get_entities(std::vector<Entity>& entities) {
    entities.clear();
    for (auto& i : items_) {
        entities.push_back(i.second->entity());
    }
}

bool X3World::act(const size_t agent_id, const size_t action) {
    auto agent_ptr = get_agent(agent_id);
    return apply_action(agent_ptr, action);
}

bool X3World::apply_action(const X3ItemPtr& item, const size_t action) {
    bool action_success = true;
    switch (action) {
        case X3NavAction::MOVE_FORWARD:
            item->move_forward();
            break;
        case X3NavAction::MOVE_BACKWARD:
            item->move_backward();
            break;
        case X3NavAction::MOVE_LEFT:
            item->move_left();
            break;
        case X3NavAction::MOVE_RIGHT:
            item->move_right();
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
            if (goal && goal->type() != "agent") {
                remove_item(goal);
            } else {
                //                action_success = false;
            }
            break;
        }
        case X3NavAction::NOOP:
            // stay static
            item->clear_move();
            break;
        default:
            LOG(ERROR) << "unknown action id: " << action;
    }
    return action_success;
}

void X3World::remove_item(X3ItemPtr& item) {
    auto it = items_.find(item->id());
    if (it != items_.end()) {
        item->move_underground();
        item_pool_.recycle_item(item);
        items_.erase(it);
    }
}

roboschool::RenderResult X3World::render(const size_t agent_id, bool debug) {
    auto agent_ptr = get_agent(agent_id);
    return camera_->render(agent_ptr.get(), debug);
}

void X3World::step(const int frame_skip) {
    world_->step(frame_skip);
    for (auto& i : items_) {
        i.second->sync_entity_info();
    }
}

std::set<std::string> X3World::contact_list(X3ItemPtr& item) {
    auto& o = item->object_mutable();
    auto l = o.contact_list();

    std::set<std::string> id_set;
    for (auto& t : l) {
        id_set.insert(b3handle_to_id_[t.bullet_handle()]);
    }

    return id_set;
}

}} // simulator::xworld3d
