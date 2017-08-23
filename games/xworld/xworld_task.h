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

#include <vector>
#include <algorithm>
#include <functional>
#include <string>
#include "teacher.h"
#include "teaching_task.h"
#include "./xworld/xitem.h"

DECLARE_string(task_mode);

namespace simulator { namespace xwd {

// This class provides a middle layer between the base Task class and
// the specific derived task classes.
// It contains some common functions used by all specific task classes
class XWorldTask : public Task {
  public:
    XWorldTask(const std::string& name,
               TeachingEnvPtr game,
               const std::vector<std::string>& held_out)
            : Task(name, game, held_out) {
        movements_ = std::vector<Loc>({{1,0}, {-1,0}, {0,1}, {0,-1}});
        neighbors_ = std::vector<Loc>({{1,1}, {-1,-1}, {1,-1}, {-1,1}});
        neighbors_.insert(neighbors_.end(), movements_.begin(), movements_.end());
        // only used for grammar right hand side, not for computing direction strings
        // do not have to correspond to the neighbors order
        directions_ = {"north", "south", "west", "east",
                       "northeast", "northwest", "southeast", "southwest"};
    }

    virtual ~XWorldTask() {}

    static constexpr double time_penalty = -0.02;
    static constexpr double correct_reward = 1.0;
    static constexpr double wrong_reward = -1.0;
    static constexpr double failed_action_reward = -0.2;

  protected:
    std::vector<Loc> movements_;
    std::vector<Loc> neighbors_;
    Vec3 target_; // the target location of the current (sub)goal
    Vec3 agent_prev_location_; // the location of the previous time step
    bool is_reply_correct_;
    std::string answer_;
    int steps_in_cur_task_;  // how many steps the agent has taken in the current task

    // Tokens used for defining sentence template rules
    std::vector<std::string> uni_objects_;
    std::vector<std::string> uni_colors_;
    std::vector<std::string> uni_colored_objects_;
    std::vector<std::string> directions_;

    // If there is valid goal, the teacher says something and proceed to the next stage
    std::string find_goal_and_generate_sentence(ScannerPtr scanner,
                                                SentenceTemplatePtr sen_temp,
                                                TeachingEnvPtr game,
                                                const std::string& next_stage_if_success);

    // Find qualified a goal set at a particular time step.
    // It can be the beginning of a game, or in the middle of a game,
    // In the latter case, the override function is responsible for tracking
    // the history for finding goal candidates
    virtual std::vector<std::vector<Entity>> find_goal(ScannerPtr scanner) {
        return std::vector<std::vector<Entity>>();
    }

    // Define the goal and specify how to bind the symbols in the sentence template.
    // The override function is responsible for tracking the teacher sentence generation
    // history and the expected response to the generated sentence.
    // Return a debug string related to the generated sentence for DISPLAY purpose only.
    //
    // The content of the debug string is determined by the task itself
    // For example, in a QA task, the string might be the answer;
    // in the language task, the string might be a possible sentence the teacher expects to
    // hear from the agent; in the navigation task, the string might be a (x,y) coordinate.
    virtual void generate_sentence(
        const std::vector<std::vector<Entity>>& goal_sets,
        ScannerPtr scanner, SentenceTemplatePtr sen_temp, TeachingEnvPtr game) {
    }

    std::string simple_navigation_reward(ScannerPtr scanner,
                                         SentenceTemplatePtr sen_temp,
                                         TeachingEnvPtr game);

    std::string simple_recognition_reward(ScannerPtr scanner,
                                          SentenceTemplatePtr sen_temp,
                                          TeachingEnvPtr game);

    // a dummy stage at the end of each session before episode end to ensure that
    // the agent can learn the teacher's correct reply when continuous_task=false
    std::string conversation_wrapup(ScannerPtr scanner,
                                    SentenceTemplatePtr sen_temp,
                                    TeachingEnvPtr game);

    // Get the direction of "around" w.r.t. "refer"
    std::string get_direction(Vec3 refer, Vec3 around);

    // whether start can reach dest
    bool destination_reachable_from_start(const Vec3& start, const Vec3& dest, ScannerPtr scanner);

    // given a target location and an agent location (the target is not necessarily
    // the same with the agent), get all the locations around the target so
    // that the agent can reach those locations from its current location
    // not passing through any block or goal
    std::vector<Vec3> get_target_around_reachable(const Entity& target,
                                                  const Entity& agent,
                                                  ScannerPtr scanner);

    // Determine if any of entities has the same location with "e"
    // If yes, then overwrite "e" with it
    // type=unique_goal|goal|block|empty
    bool location_type(Entity& e, std::string type, ScannerPtr scanner);

    // given a location, return entities of type around
    // type = "empty", "unique_goal", "goal", "color_goal", or "block"
    // "goals" can be unique or not
    std::vector<Entity> surrounding_filter(const Entity& e, std::string type, ScannerPtr scanner);

    // find out if some grid is between two goals on the east and west
    // if is_goal is true, the center grid must also be a goal
    void between_two_goals(std::vector<Entity>& middle,
                           std::vector<Entity>& west_goals,
                           std::vector<Entity>& east_goals,
                           ScannerPtr scanner,
                           bool is_goal); // if the middle grid must be a goal

    virtual void before_stage_callback_func(ScannerPtr scanner) {
        scanner->scan_agent();  // For XWorld, most likely only agent can change
                                // Thus we only scan agent before every time step
    }

    virtual void after_stage_callback_func(ScannerPtr scanner) {
        agent_prev_location_ = scanner->agent_.location;
    }

  protected:
    ////////////////////////// XWorld Task APIs /////////////////////////
    // idle stage: deciding whether to be triggered
    virtual std::string idle(ScannerPtr scanner, SentenceTemplatePtr sen_temp,
                             TeachingEnvPtr game) = 0;

    // decide which stages to register
    virtual void register_stages() = 0;

    // Add rules to the sentence template object
    virtual void define_sen_temp_rules(SentenceTemplatePtr sen_temp,
                                       TeachingEnvPtr game) override;
    /////////////////////////////////////////////////////////////////////

  private:
    // From a set of objects, find
    // 1. the set of unique object names,
    // 2. the set of unique colors
    // 3. the set of unique "color-name" pairs
    void get_unique_colors_and_objects(const std::vector<Entity>& objects);
};

}} // namespace simulator::xwd
