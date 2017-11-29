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

#include <sstream>

#include <boost/algorithm/string.hpp>
#include <boost/exception/all.hpp>

#include "teacher.h"

DEFINE_bool(task_groups_exclusive,
            true,
            "whether task groups can happen simultaneously");
DEFINE_double(curriculum, 0, "performance threshold for entering the next level");

namespace simulator {

namespace py = boost::python;

using std::vector;

Teacher::Teacher(const std::string& teacher_conf,
                 TeachingEnvPtr game,
                 bool print_teacher_config)
    : task_groups_exclusive_(FLAGS_task_groups_exclusive),
      teacher_conf_(teacher_conf),
      conf_root_(nullptr),
      num_games_so_far_(0),
      game_(game) {
    set_py_task_paths();
    reset_config(print_teacher_config);
}

void Teacher::set_py_task_paths() {
    //// list all the relative paths of the python tasks here
    std::vector<std::string> relative_python_paths = {
        "./games/xworld3d/tasks", "./games/xworld/tasks"
    };

    CHECK(Py_IsInitialized());

    try {
        std::string f = __FILE__;
        std::string path = f.substr(0, f.find_last_of("/") + 1);
        auto main_mod = py::import("__main__");
        auto main_namespace = main_mod.attr("__dict__");
        py::exec("import sys", main_namespace);

        for (const auto& rp : relative_python_paths) {
            std::string cmd = "sys.path.append(\"" + path + rp + "\")";
            py::exec(cmd.c_str(), main_namespace);
        }
    } catch (...) {
        PyErr_Print();
        LOG(FATAL) << "Error setting py_task paths";
    }
}

void Teacher::add_task_group(const pt::ptree::value_type& node) {
    // Specify task sampling method
    std::string schedule = "random";
    if (node.second.count("schedule") > 0) {
        schedule = node.second.get<std::string>("schedule");
    }
    // Different task groups should have different held-out patterns
    std::vector<std::string> held_out;
    if (node.second.count("held_out") > 0) {
        for (const auto& h : node.second.get_child("held_out")) {
            held_out.push_back(h.second.get_value<std::string>());
        }
    }
    // Specify task group weight
    double weight = 0;
    if (node.second.count("weight") > 0) {
        weight = node.second.get<double>("weight");
    }

    auto group = std::make_shared<TaskGroup>(node.first, schedule, game_);
    task_groups_.push_back(group);
    task_group_weights_.push_back(weight);

    // add tasks for this task group
    CHECK_GT(node.second.count("tasks"), 0);
    for (const auto& n : node.second.get_child("tasks")) {
        double weight = n.second.get_value<double>();
        group->add_task(n.first, weight);
    }
}

bool Teacher::is_idle() {
    for (auto g : task_groups_) {
        if (!g->is_idle()) {
            return false;
        }
    }
    return true;
}

void Teacher::reset_config(const std::string& teacher_conf, bool print) {
    num_games_so_far_++;
    teacher_conf_ = teacher_conf;
    task_groups_.clear();
    task_group_weights_.clear();

    // only read the teacher config once
    if (!conf_root_) {
        if (print) {
            std::ifstream infile(teacher_conf);
            std::string line;
            while (std::getline(infile, line)) {
                LOG(INFO) << line;
            }
        }
        conf_root_.reset(new pt::ptree);
        try {
            pt::read_json(teacher_conf, *(conf_root_.get()));
        } catch (const boost::exception& ex) {
            LOG(FATAL) << "Teacher config file error: "
                       << boost::diagnostic_information(ex);
        }
    }

    CHECK_GT(conf_root_->count("task_groups"), 0);
    // Get all the groups
    // The groups have priorties according to their order in json
    // From high to low
    for (const auto& node : conf_root_->get_child("task_groups")) {
        add_task_group(node);
    }
}

void Teacher::nondeterministic_sort_task_groups() {
    // suppose the task groups have weights w_0, w_1, ..., w_{n-1}
    // we want to sort them so that:
    // 1. each group i has probility prop to w_i to be the first group
    // 2. excluding the first selected group, for the remaining groups,
    //    group j has probability prop to w_j to be the second group
    // 3. this goes on until the groups are sorted
    // So for a specific group k, it has the prob of w_k/W to be the first group
    // the prob of (1-w_k/W)*(w_k/(W-w_i)) to be the second group ...
    for (size_t i = 0; i < task_groups_.size(); i++) {
        std::vector<double> remain_weights(task_group_weights_.begin() + i,
                                           task_group_weights_.end());
        // compute accumulated weights
        for (size_t j = 1; j < remain_weights.size(); j++) {
            remain_weights[j] += remain_weights[j - 1];
        }
        int idx = util::simple_importance_sampling(remain_weights);
        std::swap(task_groups_[i], task_groups_[idx + i]);
        std::swap(task_group_weights_[i], task_group_weights_[idx + i]);
    }
}

void Teacher::print_total_possible_sentences() {
    size_t total = 0;
    for (auto g : task_groups_) {
        size_t cnt = g->total_possible_sentences();
        total += cnt;
        LOG(INFO) << "Task group " << g->name() << " sentences: " << cnt;
    }
    LOG(INFO) << "Total: " << total;
}

void Teacher::report_task_performance() {
    // the key is task's name
    // the value is a pair of success and failure times
    std::unordered_map<std::string, std::pair<size_t, size_t>> benchmark;
    for (auto g : task_groups_) {
        g->report_task_performance(benchmark);
    }
    // print
    for (const auto& task : benchmark) {
        LOG(INFO) << "=== " << task.first << " ===";
        size_t success = task.second.first;
        size_t failure = task.second.second;
        if (success + failure == 0) {  // skip task that did not occur
            continue;
        }
        LOG(INFO) << "=== " << success << "(S)/" << failure << "(F) -> "
                  << double(success) / (success + failure);
    }
}

void Teacher::before_teach() {
    // Clear teacher's env buffer so that it will store new info
    game_->clear_teacher_env_buffer();
}

bool Teacher::teach() {
    before_teach();
    if (task_groups_exclusive_) {
        nondeterministic_sort_task_groups();
        TaskGroupPtr any_busy_group = nullptr;
        for (auto g : task_groups_) {
            if (!g->is_idle()) {
                any_busy_group = g;
            }
        }
        if (!any_busy_group) {
            any_busy_group = task_groups_[0];
        }
        // only run one group at a time
        any_busy_group->run_stage();
    } else {
        // run all groups in parallel
        for (auto g : task_groups_) {
            g->run_stage();
        }
    }
    after_teach();
    return true;
}

void Teacher::after_teach() {
    // This will CHANGE the environment with teacher's actions
    game_->apply_teacher_actions();
    // Clear agent's buffer so that later agent can fill in
    game_->clear_agent_env_buffer();
}

void Teacher::print_current_stages() {
    for (auto g : task_groups_) {
        LOG(INFO) << "Task_group-> " << g->current_stage();
    }
}

void Teacher::reset_after_game_reset() {
    for (auto g : task_groups_) {
        g->reset();
    }
    game_->clear_teacher_env_buffer();
    game_->clear_agent_env_buffer();
}

}  // namespace simulator
