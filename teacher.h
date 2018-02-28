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
#include <algorithm>
#include <boost/algorithm/string.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <functional>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include "simulator.h"
#include "teaching_task.h"

DECLARE_double(curriculum);

namespace simulator {

namespace pt = boost::property_tree;

/**
   A teacher is able to get the true, unambiguous state of the world.
   A teacher will not learn anything but only communicate with agents according
   to a predefined set of rules,
   It can raise questions, give instructions, teach concepts, make statements, etc.
   A teacher maintains a list of task groups.
   Each task group contains a collection of tasks that are exclusive to each
   other but of the same type
   The task groups are parallel to each other.
**/
class Teacher {
  public:
    Teacher(const std::string& teacher_conf,
            TeachingEnvPtr game,
            bool print_teacher_config = false);

    // reset the teacher's configure file
    // when print=true, print the content of the conf file
    void reset_config(const std::string& teacher_conf, bool print = false);

    // this simply resets the teacher without changing the conf file
    void reset_config(bool print = false) {
        reset_config(teacher_conf_, print);
    }

    // Called every time step
    // The teacher first sort task groups according to their weights
    // (see nondeterministic_sort_task_groups),
    // then run through every group and call run_stage of that group
    // If currently the teacher is teaching, this function always return true
    bool teach();

    // print all the task stages of all the task groups
    void print_current_stages();

    // Reset the teacher after game is reset
    void reset_after_game_reset();

    // print the number of sentences the teacher can possibly say
    void print_total_possible_sentences();

    // This function should be called by game_player at the end of benchmarking
    void report_task_performance();

    double give_reward() { return game_->get_teacher_reward(); }

    // whether all task groups are idle;
    // return false if some task group is busy
    bool is_idle();

  protected:
    void before_teach();  // pre-processing before teaching
    void after_teach();   // post-processing after teaching

  private:
    void set_py_task_paths();
    void add_task_group(const pt::ptree::value_type& node);
    void nondeterministic_sort_task_groups();

    std::vector<TaskGroupPtr> task_groups_;
    std::vector<double> task_group_weights_;
    bool task_groups_exclusive_;
    std::string teacher_conf_;
    std::unique_ptr<pt::ptree> conf_root_;
    size_t num_games_so_far_;
    TeachingEnvPtr game_;
};

typedef std::shared_ptr<Teacher> TeacherPtr;

}  // namespace simulator
