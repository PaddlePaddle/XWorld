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
#include <boost/python.hpp>
#include "simulator.h"
#include "teacher_sentence_generator.h"
#include "world_scanner.h"

// Should call this in the derived classes of Task
#define REGISTER_STAGE(__stage)                                \
    stages_[#__stage] = [this]() {                             \
        before_stage_callback_func(scanner_);                  \
        auto next_stage = __stage(scanner_, sen_temp_, game_); \
        after_stage_callback_func(scanner_);                   \
        return next_stage;                                     \
    }

#define REGISTER_TASK(__task)                                                \
    static util::InitFunction __reg_class_##__task([]() {                    \
        TaskGroup::create_tasks_[#__task] = [](                              \
            TeachingEnvPtr game, const std::vector<std::string>& held_out) { \
            return std::make_shared<__task>(#__task, game, held_out);        \
        };                                                                   \
    })

namespace simulator {

/**
   A task is a Finite State Machine (FSM) that has several stages (states).
   Each stage can be programmed to jump to another stage.
   At each stage, the teacher performs an action, whether it being
   a sentence sent to the agent, or some changes to the environment.
   Each stage corresponds to a time step.
 **/
class Task {
  public:
    Task(std::string name,
         TeachingEnvPtr game,
         const std::vector<std::string>& held_out)
        : game_(game),
          name_(name),
          current_stage_("idle"),
          num_successes_(0),
          num_failures_(0) {
        sen_temp_ = std::make_shared<TeacherSentenceTemplate>(held_out);
        scanner_ = std::make_shared<WorldScanner>(game);
    }

    virtual ~Task() {}

    const std::string& name() const { return name_; }

    void init() {
        define_sen_temp_rules(sen_temp_, game_);
        sen_temp_->check_rules();
        register_stages();
    }

    virtual void reset() {
        current_stage_ = "idle";
        scanner_
            ->init_scan();  // scan the world all over again after game reset
    }

    bool is_idle() const { return current_stage_ == "idle"; }

    std::string current_stage() const { return name_ + ": " + current_stage_; }

    // run the current stage
    void run_stage();

    void record_success() { num_successes_++; }

    void record_failure() { num_failures_++; }

    size_t num_successes() { return num_successes_; }

    size_t num_failures() { return num_failures_; }

    size_t total_possible_sentences() {
        return sen_temp_->total_possible_sentences();
    }

    // Generate a teacher's sentence according to the template
    // Return a boolean value indicating whether the speak action succeeds or
    // not
    // It fails when the sentence generator returns an empty string.
    // The reason for this might be that the originally generated sentence has
    // some
    // "held_out" phrases in it.
    static bool teacher_speak(bool held_out,
                              const std::string& task_name,
                              SentenceTemplatePtr sen_temp,
                              TeachingEnvPtr game);

  protected:
    /////////////////// Task APIs ///////////////////
    // idle stage: deciding whether to be triggered
    virtual std::string idle(ScannerPtr scanner,
                             SentenceTemplatePtr sen_temp,
                             TeachingEnvPtr game) = 0;

    // decide which stages to register
    virtual void register_stages() = 0;

    // Add rules to the sentence template
    virtual void define_sen_temp_rules(SentenceTemplatePtr sen_temp,
                                       TeachingEnvPtr game) = 0;
    /////////////////////////////////////////////////

    // This function is called before every stage is executed
    // Can be used to prepare for the stage's execution
    // e.g., see xworld_task.h implementation
    virtual void before_stage_callback_func(ScannerPtr scanner) {}

    // This function is called after every stage is executed
    // Can be used to update the internal states of the Task class
    // e.g., see xworld_task.h implementation
    virtual void after_stage_callback_func(ScannerPtr scanner) {}

    // teacher calls this function to give the agent a reward
    void give_reward(double reward) { game_->add_teacher_reward(reward); }

    //// Three crucial components of a task
    //// 1. scanner_: scan the world and return info for the task
    //// 2. sen_temp_: a sentence template for generating sentences
    //// 3. game_: the game in which the task is defined
    ScannerPtr scanner_;  // A world scanner that provide internal world state
    SentenceTemplatePtr
        sen_temp_;  // The sentence template the teacher replies on for speaking
    TeachingEnvPtr
        game_;  // The environment the teacher and the agent reside in

    std::string name_;  // A string that identifies the task

    // stores all the stage functions
    std::unordered_map<std::string, std::function<std::string()>> stages_;

  private:
    std::string current_stage_;  // Maintains the current stage of the task
    size_t num_successes_;       // number of successes in total
    size_t num_failures_;        // number of failures in total
};

typedef std::shared_ptr<Task> TaskPtr;

/**
   A wrapper class that use the embedded Python class for defining tasks
   The python users are responsible for implementing
   1. all the stage functions
   2. the grammar for generating the sentences
   3. the reward of each stage
 **/
class PyTask : public Task {
  public:
    PyTask(const std::string& name,
           TeachingEnvPtr game,
           const std::vector<std::string>& held_out)
            : Task(name, game, held_out) {
        // initialize the python interpreter
        if (!Py_IsInitialized()) {
            Py_Initialize();
        }
        try {
            // check if there is a python file with this name
            auto mod = boost::python::import(name.c_str());
            py_task_ = mod.attr(name.c_str())();
        } catch (...) {
            LOG(FATAL) << "Unrecognized task name: " << name;
        }
    }

    //// Currently boost python does not support Py_Finalize
    //// http://www.boost.org/doc/libs/1_64_0/libs/python/doc/html/tutorial/tutorial/embedding.html
    // ~PyTask() {
    //     if (Py_IsInitialized()) {
    //         Py_Finalize();
    //     }
    // }

  protected:
    virtual std::string idle(ScannerPtr scanner,
                             SentenceTemplatePtr sen_temp,
                             TeachingEnvPtr game) override {
        return py_stage(sen_temp, game, "idle");
    }

    // Not used here; We will directly use the sentence returned by Python
    virtual void define_sen_temp_rules(SentenceTemplatePtr sen_temp,
                                       TeachingEnvPtr game) override {
        sen_temp->add_rule(sen_temp->start_symbol(), {""});
    }

    virtual void register_stages() override;

  private:
    // Call a python stage function and process its outputs
    std::string py_stage(SentenceTemplatePtr sen_temp, TeachingEnvPtr game,
                         const std::string& stage_name);

    // Convert a simulator_entity to a dictionary in Python
    boost::python::dict convert_entity_to_py_entity(const Entity& e);

    // This object holds the Python task class
    boost::python::object py_task_;
};

/**
   A task group is a collection of tasks that have the same group id.
   There can only be one busy task in a group at any moment
 **/
class TaskGroup {
  public:
    TaskGroup(std::string name,
              std::string schedule,
              TeachingEnvPtr game,
              const std::vector<std::string>& held_out)
        : name_(name),
          schedule_(schedule),
          game_(game),
          held_out_(held_out),
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
    std::vector<std::string> held_out_;
    TaskPtr busy_task_;
};

typedef std::shared_ptr<TaskGroup> TaskGroupPtr;

}  // namespace simulator
