// MIT License

// Copyright (c) 2017 Baidu Inc. All rights reserved.

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
/////////////////////////////////////////////////////////////////////////////////

#include "teaching_task.h"
#include "simulator_util.h"

namespace simulator {

std::unordered_map<std::string,
                   std::function<TaskPtr(TeachingEnvPtr, const std::vector<std::string>&)>> \
        TaskGroup::create_tasks_;

bool Task::teacher_speak(bool held_out,
                         const std::string& task_name,
                         SentenceTemplatePtr sen_temp,
                         TeachingEnvPtr game) {
    // Generate
    CHECK(sen_temp);
    std::string sentence = sen_temp->instantiate(
        held_out,
        game->num_games_since_simulation(),
        game->curriculum_learning());
    bool success_speak = (sentence != "");
    // The teacher might have held_out sentence patterns
    if (!held_out) {
        CHECK(success_speak);
    }
    success_speak &= game->can_record_teacher_sent_in_buffer();
    if (success_speak) {
        game->record_teacher_sent_in_buffer(sentence);
        game->record_teacher_sent_type_in_buffer(task_name);
    }
    return success_speak;
}

void Task::run_stage() {
    CHECK_GT(stages_.count(current_stage_), 0)
            << "Unrecognized stage name: " << current_stage_;
    current_stage_ = stages_[current_stage_]();
}

void TaskGroup::add_task(const std::string& task, double weight) {
    CHECK_GT(weight, 0) << "A task must have a positive weight";
    CHECK_EQ(task.find(name_), 0) << "Task group name must be a prefix of the task name:"
                                  << name_ << " " << task;
    CHECK_GT(create_tasks_.count(task), 0)
            << "Unrecognized task name: " << task;
    auto task_ptr = create_tasks_[task](game_, held_out_);
    task_ptr->init();
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
        auto success = task->num_successes();
        auto failure = task->num_failures();
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
    auto sample_task = [&] () {
        int idx = -1;
        if (schedule_ == "weighted") {
            idx = util::simple_importance_sampling(task_weights_);
        } else { // random
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

} // namespace simulator
