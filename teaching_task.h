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
#include <functional>
#include <unordered_map>
#include "simulator.h"

namespace simulator {

/**
   A task is a Finite State Machine (FSM) that has several stages (states).
   Each stage can be programmed to jump to another stage.
   At each stage, the teacher performs an action, whether it being
   a sentence sent to the agent, or some changes to the environment.
   Each stage corresponds to a time step.

   A wrapper class that use the embedded Python class for defining tasks
   The python users are responsible for implementing
   1. all the stage functions
   2. the grammar for generating the sentences
   3. the reward of each stage
 **/
class Task {
  public:
    Task(std::string name,
         TeachingEnvPtr game)
        : game_(game),
          name_(name),
          current_stage_("idle") {
        init_py_task();
        register_stages();
    }

    const std::string& name() const { return name_; }

    void reset() {
        py_task_.attr("reset")();
        current_stage_ = "idle";
    }

    bool is_idle() const { return current_stage_ == "idle"; }

    std::string current_stage() const { return name_ + ": " + current_stage_; }

    // run the current stage
    void run_stage();

    void obtain_performance(size_t& num_successes_since_simulation,
                            size_t& num_failures_since_simulation);

    size_t total_possible_sentences();

    void teacher_speak(const std::string& sentence);

  protected:
    // teacher calls this function to give the agent a reward
    void give_reward(double reward) { game_->add_teacher_reward(reward); }

    TeachingEnvPtr game_;  // The environment the teacher and the agent reside in
    std::string name_;  // A string that identifies the task

    // stores all the stage functions
    std::unordered_map<std::string, std::function<std::string()>> stages_;

  private:
    void init_py_task();

    // decide which stages to register
    void register_stages();

    // Call a python stage function and process its outputs
    std::string py_stage(const std::string& stage_name);

    // Convert a simulator_entity to a dictionary in Python
    boost::python::dict convert_entity_to_py_entity(const Entity& e);

    // This object holds the Python task class
    boost::python::object py_task_;

    std::string current_stage_;  // Maintains the current stage of the task
};

typedef std::shared_ptr<Task> TaskPtr;

/**
   A task group is a collection of tasks that have the same group id.
   There can only be one busy task in a group at any moment
 **/
class TaskGroup {
  public:
    TaskGroup(std::string name,
              std::string schedule,
              TeachingEnvPtr game)
        : name_(name),
          schedule_(schedule),
          game_(game),
          busy_task_(nullptr) {  // points to a task that is not in idle stage
    }

    // add a task with a sampling weight
    // when schedule="weighted", the tasks in a group are sampled in prop to
    // their weights
    void add_task(const std::string& task, double weight);

    bool is_idle();

    void reset();

    std::string name() { return name_; }

    // call run_stage of the busy task
    void run_stage();

    size_t total_possible_sentences();

    std::string current_stage();

    // fill in benchmark with task performance
    void report_task_performance(
        std::unordered_map<std::string, std::pair<size_t, size_t>>& benchmark);

    static std::unordered_map<
        std::string,
        std::function<TaskPtr(TeachingEnvPtr, const std::vector<std::string>&)>>
        create_tasks_;

  private:
    std::vector<TaskPtr> task_list_;
    std::vector<double> task_weights_;
    std::string name_;
    std::string schedule_;
    TeachingEnvPtr game_;
    TaskPtr busy_task_;
};

typedef std::shared_ptr<TaskGroup> TaskGroupPtr;

}  // namespace simulator
