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

#include "teaching_task.h"
#include "simulator_util.h"

namespace simulator {

namespace py = boost::python;

void Task::init_py_task() {
    try {
        // check if there is a python file with this name
        auto mod = py::import(name_.c_str());
        py_task_ = mod.attr(name_.c_str())(game_->get_py_env());
    } catch (...) {
        PyErr_Print();
    }
}

void Task::register_stages() {
    CHECK(PyObject_HasAttrString(py_task_.ptr(), "get_stage_names"))
            << "Python function get_stage_names() is undefined!";
    py::list py_stage_names = py::extract<py::list>(py_task_.attr("get_stage_names")());

    std::vector<std::string> stage_names;
    for (int i = 0; i < py::len(py_stage_names); i ++) {
        stage_names.push_back(py::extract<std::string>(py_stage_names[i]));
    }

    for (const auto& name : stage_names) {
        stages_[name] = [this, name]() {
            return py_stage(name);
        };
    }
}

size_t Task::total_possible_sentences() {
    return py::extract<int>(py_task_.attr("total_possible_sentences")());
}

std::string Task::py_stage(const std::string& stage_name) {
    CHECK(PyObject_HasAttrString(py_task_.ptr(), stage_name.c_str()))
            << "Python task stage is undefined: " << stage_name;

    // pre-stage: update the python env with simulator changes
    std::vector<Entity> entities;
    game_->get_all_entities(entities);
    py::list py_entities;  // a list of py::dict
    for (const auto& e : entities) {
        py_entities.append(e.to_py_dict());
    }
    py::object env = game_->get_py_env();
    env.attr("update_entities_from_cpp")(py_entities);

    auto agent_sent = game_->get_agent_sent_from_buffer();
    env.attr("update_agent_sentence_from_cpp")(agent_sent.c_str());

    auto action_success = game_->get_agent_action_successful_from_buffer();
    env.attr("update_agent_action_success_from_cpp")(action_success);

    // during the stage
    py::list ret = py::extract<py::list>(py_task_.attr(stage_name.c_str())());

    // post-stage: the teacher might have changed the environment
    if (env.attr("env_changed")()) {
        game_->update_environment();
    }

    CHECK_EQ(py::len(ret), 3) << "Incorrect length of stage returns";
    std::string next_stage = py::extract<std::string>(ret[0]);
    double reward = py::extract<double>(ret[1]);
    std::string sentence = py::extract<std::string>(ret[2]);
    give_reward(reward);
    teacher_speak(sentence);
    return next_stage;
}

void Task::teacher_speak(const std::string& sentence) {
    if (game_->can_record_teacher_sent_in_buffer()) {
        game_->record_teacher_sent_in_buffer(sentence);
        game_->record_teacher_sent_type_in_buffer(name_);
    } else {
        // (task_groups_exclusive = false)
        // TODO: if sentence is not spoken, then the task
        // perhaps should not proceed to the next stage.
    }
}

void Task::run_stage() {
    CHECK_GT(stages_.count(current_stage_), 0) << "Unrecognized stage name: "
                                               << current_stage_;
    current_stage_ = stages_[current_stage_]();
}

void Task::obtain_performance(size_t& num_successes_since_simulation,
                              size_t& num_failures_since_simulation) {
    py::tuple perf = py::extract<py::tuple>(py_task_.attr("obtain_performance")());
    num_successes_since_simulation = py::extract<int>(perf[0]);
    num_failures_since_simulation = py::extract<int>(perf[1]);
}

void TaskGroup::add_task(const std::string& task, double weight) {
    CHECK_GT(weight, 0) << "A task must have a positive weight";
    CHECK_EQ(task.find(name_), 0)
        << "Task group name must be a prefix of the task name:" << name_ << " "
        << task;
    TaskPtr task_ptr = std::make_shared<Task>(task, game_);
    task_list_.push_back(task_ptr);
    if (task_weights_.empty()) {
        task_weights_.push_back(weight);
    } else {
        task_weights_.push_back(task_weights_.back() + weight);
    }
}

void TaskGroup::report_task_performance(
    std::unordered_map<std::string, std::pair<size_t, size_t>>& benchmark) {
    for (auto task : task_list_) {
        auto task_name = task->name();
        size_t success = 0;
        size_t failure = 0;
        task->obtain_performance(success, failure);
        auto& p = benchmark[task_name];
        if (benchmark.count(task_name) == 0) {
            p = std::make_pair(success, failure);
        } else {
            p.first += success;
            p.second += failure;
        }
    }
}

void TaskGroup::reset() {
    // For now, we only need to untrack the busy task.
    // We will do a lazy reset later when a task
    // is picked as the busy task.
    busy_task_ = nullptr;
}

bool TaskGroup::is_idle() {
    if (!busy_task_) {
        return true;
    } else {  // a task that was busy might be idle now; we need to check
        if (busy_task_->is_idle()) {
            busy_task_ = nullptr;
            return true;
        } else {
            return false;
        }
    }
}

std::string TaskGroup::current_stage() {
    if (busy_task_) {
        return name_ + " | Task-> " + busy_task_->current_stage();
    } else {
        return name_ + " | All tasks: idle";
    }
}

void TaskGroup::run_stage() {
    auto sample_task = [&]() {
        int idx = -1;
        if (schedule_ == "weighted") {
            idx = util::simple_importance_sampling(task_weights_);
        } else {  // random
            idx = util::get_rand_ind(task_list_.size());
        }
        return idx;
    };
    if (is_idle()) {
        // when idle, randomly sample a task
        busy_task_ = task_list_[sample_task()];
        // Here we do a lazy reset of the busy task,
        // only before we are running that task.
        busy_task_->reset();
    }
    busy_task_->run_stage();
}

size_t TaskGroup::total_possible_sentences() {
    size_t total = 0;
    for (auto task : task_list_) {
        total += task->total_possible_sentences();
    }
    return total;
}

}  // namespace simulator
