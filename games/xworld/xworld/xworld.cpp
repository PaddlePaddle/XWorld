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

#include "xworld.h"
#include <gflags/gflags.h>
#include <algorithm>
#include <boost/algorithm/string.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/exception/diagnostic_information.hpp>
#include <fstream>
#include <limits>
#include <utility>

DECLARE_string(curriculum_stamp);

namespace simulator {
namespace xwd {

namespace pt = boost::property_tree;
namespace py = boost::python;

int XWorld::get_num_actions() {
    if (agent_list_.size() > 0) {
        return agent_list_[0]->get_num_actions();
    } else {
        Entity e;
        e.type = "agent";
        auto agent = XItem::create_item(e);
        return agent->get_num_actions();
    }
}

py::object XWorld::get_py_env() {
    return xwd_env_;
}

void XWorld::get_entities(std::vector<Entity>& entities) {
    entities.clear();
    for (auto i : item_list_) {
        entities.push_back(i->entity());
    }
}

XWorld::XWorld(const std::string& conf, bool print_conf) : conf_(conf) {
    if (print_conf) {
        std::ifstream infile(conf);
        std::string line;
        while (std::getline(infile, line)) {
            LOG(INFO) << line;
        }
    }

    std::unique_ptr<pt::ptree> tree;
    tree.reset(new pt::ptree);
    try {
        pt::read_json(conf, *(tree.get()));
    } catch (const boost::exception& ex) {
        LOG(FATAL) << "world config file error: "
                   << boost::diagnostic_information(ex);
    }
    CHECK_GT(tree->count("item_path"), 0);
    CHECK_GT(tree->count("map"), 0);
    std::string item_path = tree->get<std::string>("item_path");
    std::string map = tree->get<std::string>("map");

    CHECK(Py_IsInitialized());


    try {
        std::string f = __FILE__;
        std::string path = f.substr(0, f.find_last_of("/") + 1);
        auto main_mod = py::import("__main__");
        auto main_namespace = main_mod.attr("__dict__");
        py::exec("import sys", main_namespace);
        std::string cmd = "sys.path.append(\"" + path + "../maps\")";
        py::exec(cmd.c_str(), main_namespace);

        auto mod = py::import(map.c_str());
        item_path = path + "../" + item_path;
        // read the curriculum record if possible
        if (FLAGS_curriculum_stamp != "") {
            int start_level = 0;
            std::ifstream infile(FLAGS_curriculum_stamp);
            infile >> start_level;
            xwd_env_ = mod.attr(map.c_str())(item_path.c_str(), start_level);
        } else { // default start_level
            xwd_env_ = mod.attr(map.c_str())(item_path.c_str());
        }
    } catch (...) {
        PyErr_Print();
        LOG(FATAL) << "Error loading map: " << map;
    }

    reset();
}

void XWorld::reset(bool map_reset) {
    try {
        if (map_reset) {
            // regenerate a xwd map
            xwd_env_.attr("reset")();
            // reset the change flag
            CHECK(xwd_env_.attr("env_changed")());
        }
        item_list_.clear();
        agent_list_.clear();

        py::tuple dims = py::extract<py::tuple>(xwd_env_.attr("get_max_dims")());
        height_ = py::extract<int>(dims[0]);
        width_ = py::extract<int>(dims[1]);
        map_ = XMap(height_, width_);

        const int record_curriculum_period = 200;
        static int n_games = 0;
        if (FLAGS_curriculum_stamp != "" && (++n_games) % record_curriculum_period == 0) {
            int level = py::extract<int>(xwd_env_.attr("dump_curriculum_progress")());
            std::ofstream outfile(FLAGS_curriculum_stamp);
            outfile << level;
        }

        py::list entities = py::extract<py::list>(xwd_env_.attr("cpp_get_entities")());
        for (int i = 0; i < py::len(entities); i ++) {
            py::dict e = py::extract<py::dict>(entities[i]);
            auto item = XItem::create_item(Entity(e));
            item_list_.push_back(item);
            if (item->get_item_type() == "agent") {
                agent_list_.push_back(item);
            }
        }
        map_.add_items(item_list_);
    } catch (...) {
        PyErr_Print();
        LOG(FATAL) << "Error resetting map";
    }
}

cv::Mat XWorld::to_image(int agent_id,
                         bool flag_illustration, /* true */
                         int visible_radius_unit /*0*/) {
    return map_.to_image(agent_list_[agent_id]->get_item_location(),
                         agent_list_[agent_id]->get_item_yaw(),
                         flag_illustration,
                         visible_radius_unit);
}

bool XWorld::act(int agent_id, int action_id) {
    auto agent_ptr = agent_list_[agent_id];
    Loc target = agent_ptr->act(action_id);
    return map_.move_item(agent_ptr, target);
}

}
}  // namespace simulator::xwd
