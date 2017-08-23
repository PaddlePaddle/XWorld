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

#include "xworld_task.h"
#include "teacher_sentence_generator.h"
#include <gflags/gflags.h>
#include <boost/algorithm/string.hpp>
#include <queue>

DEFINE_string(task_mode, "one_channel",
              "arxiv_lang_acquisition | arxiv_interactive | one_channel");

namespace simulator { namespace xwd {

std::string XWorldTask::get_direction(Vec3 refer, Vec3 around) {
    std::string direction = "";
    if (refer.square_distance(around) <= 2) {
        direction = refer.get_direction(around);
    }
    return direction;
}

bool XWorldTask::destination_reachable_from_start(const Vec3& start,
                                                  const Vec3& dest,
                                                  ScannerPtr scanner) {
    // Define start and destination
    Loc start2d = Loc(int(start.x), int(start.y));
    Loc dest2d = Loc(int(dest.x), int(dest.y));
    // Both "block" and "goal" are obstacles
    std::unordered_set<Loc> obstacles;
    for (const auto& b : scanner->blocks_) {
        obstacles.insert(Loc(b.location.x, b.location.y));
    }
    for (const auto& g : scanner->goals_) {
        obstacles.insert(Loc(g.location.x, g.location.y));
    }
    // BFS
    std::queue<Loc> q;
    std::unordered_set<Loc> visited;
    visited.insert(start2d);
    q.push(start2d);
    while (!q.empty()) {
        Loc cur = q.front();
        q.pop();
        if (cur == dest2d) {
            return true;
        }
        for (const auto& m: movements_) {
            Loc next = cur + m;
            if (!scanner->outside_world(Vec3(next.x, next.y, 0))
                && obstacles.count(next) == 0
                && visited.count(next) == 0) {
                visited.insert(next);
                q.push(next);
            }
        }
    }
    return false;
}

std::vector<Vec3> XWorldTask::get_target_around_reachable(const Entity& target,
                                                          const Entity& agent,
                                                          ScannerPtr scanner) {
    std::vector<Vec3> locations;
    for (const auto& m : neighbors_) {
        Vec3 target_around = target.location + Vec3(m.x, m.y, 0);
        if (destination_reachable_from_start(agent.location, target_around, scanner)) {
            locations.push_back(target_around);
        }
    }
    return locations;
}

bool XWorldTask::location_type(Entity& e, std::string type, ScannerPtr scanner) {
    auto any_equal_location = [&] (const std::vector<Entity>& entities) {
        for (const auto& en : entities) {
            if (en.location == e.location) {
                e = en;
                return true;
            }
        }
        return false;
    };
    if (type == "unique_goal") {
        return any_equal_location(scanner->unique_goals_);
    } else if (type == "goal") {
        return any_equal_location(scanner->goals_);
    } else if (type == "color_goal") {
        return any_equal_location(scanner->color_goals_);
    } else if (type == "block") {
        return any_equal_location(scanner->blocks_);
    } else if (type == "empty") {
        return !any_equal_location(scanner->goals_) \
                && !any_equal_location(scanner->blocks_);
    } else {
        LOG(FATAL) << "Unsupported type";
    }
    return false;
}

std::vector<Entity> XWorldTask::surrounding_filter(
    const Entity& e, std::string type, ScannerPtr scanner) {
    std::vector<Entity> surround;
    for (const auto& m : neighbors_) {
        Entity around;
        around.location = e.location - Vec3(m.x, m.y, 0);
        around.set_property("name", "nothing");
        if (!scanner->outside_world(around.location) // within the boundary
            && location_type(around, type, scanner)) {
            surround.push_back(around);
        }
    }
    return surround;
}

void XWorldTask::between_two_goals(std::vector<Entity>& middle,
                                   std::vector<Entity>& west_goals,
                                   std::vector<Entity>& east_goals,
                                   ScannerPtr scanner,
                                   bool is_goal) {
    middle.clear();
    west_goals.clear();
    east_goals.clear();

    for (const auto& wg : scanner->unique_goals_) { // enumerate west goal
        Entity east_goal;
        Entity m;
        east_goal.location = wg.location + Vec3(2, 0, 0);
        bool flag = false;
        if (location_type(east_goal, "unique_goal", scanner)) {
            m.location = wg.location + Vec3(1, 0, 0);
            m.set_property("name", "nothing");
            if (!is_goal && location_type(m, "empty", scanner)) { // middle should be empty
                flag = true;
            } else if (is_goal && location_type(m, "goal", scanner)) { // middle should be goal, too
                flag = true;
            }
        }
        if (flag) {
            middle.push_back(m);
            west_goals.push_back(wg);
            east_goals.push_back(east_goal);
        }
    }
}

std::string XWorldTask::find_goal_and_generate_sentence(
    ScannerPtr scanner,
    SentenceTemplatePtr sen_temp,
    TeachingEnvPtr game,
    const std::string& next_stage_if_success) {
    auto goal_sets = find_goal(scanner);
    if (!goal_sets.empty()) {
        generate_sentence(goal_sets, scanner, sen_temp, game);
        game->record_teacher_sent_answer_in_buffer(answer_);
        if (Task::teacher_speak(true, name_, sen_temp, game)) {
            steps_in_cur_task_ = 0; // start counting steps
            return next_stage_if_success;
        }
    }
    return "idle";
}

std::string XWorldTask::simple_navigation_reward(ScannerPtr scanner,
                                                 SentenceTemplatePtr sen_temp,
                                                 TeachingEnvPtr game) {
    if (FLAGS_task_mode == "arxiv_lang_acquisition") {
        give_reward(-0.1);
    } else {
        give_reward(time_penalty);
    }
    // a failed action
    // NOTE: when the teacher's command is not present
    //       a failed action won't impose a negative reward
    if (scanner->agent_.location == agent_prev_location_) {
        give_reward(failed_action_reward);
        game->record_event_in_buffer("hit_wall");
    }
    if (scanner->agent_.location == target_) { // The agent succeeds; returns to idle
        record_success();
        give_reward(correct_reward);
        game->record_event_in_buffer("correct_goal");
        // say
        sen_temp->bind(sen_temp->start_symbol(), "$END");
        Task::teacher_speak(false, name_, sen_temp, game);
        // No wrapup stage; the agent cannot learn the teacher's last sentence
        // if continuous_task=false
        return "idle";
    } else if (location_type(scanner->agent_, "goal", scanner)) { // steps on the wrong goal
        record_failure();
        give_reward(wrong_reward);
        game->record_event_in_buffer("wrong_goal");
    }

    steps_in_cur_task_ ++;
    if (FLAGS_task_mode == "one_channel"
        && steps_in_cur_task_ == scanner->world_size_ / 2) {
        record_failure();
        game->record_event_in_buffer("time_up");
        // say
        sen_temp->bind(sen_temp->start_symbol(), "$TIMEUP");
        Task::teacher_speak(false, name_, sen_temp, game);
        return "idle";
    }
    return "simple_navigation_reward";
}

std::string XWorldTask::simple_recognition_reward(ScannerPtr scanner, SentenceTemplatePtr sen_temp,
                                                  TeachingEnvPtr game) {
    is_reply_correct_ = scanner->scan_agent_sent_from_env() == answer_;
    sen_temp->clear_rules();
    sen_temp->add_rule(sen_temp->start_symbol(), {answer_});
    Task::teacher_speak(false, name_, sen_temp, game);
    if (is_reply_correct_) {
        give_reward(correct_reward / 2.0);
    } else {
        give_reward(wrong_reward / 2.0);
    }
    // re-init the sentence template after providing the answer
    define_sen_temp_rules(sen_temp, game);
    return "conversation_wrapup";
}

std::string XWorldTask::conversation_wrapup(
    ScannerPtr scanner, SentenceTemplatePtr sen_temp, TeachingEnvPtr game) {
    if (is_reply_correct_) {
        record_success();
        game->record_event_in_buffer("correct_reply");
    } else {
        record_failure();
        game->record_event_in_buffer("wrong_reply");
    }
    return "idle";
}

void XWorldTask::define_sen_temp_rules(SentenceTemplatePtr sen_temp,
                                       TeachingEnvPtr game) {
    std::vector<Entity> objects;
    game->get_all_possible_objects(objects);
    get_unique_colors_and_objects(objects);
}

void XWorldTask::get_unique_colors_and_objects(const std::vector<Entity>& objects) {
    std::unordered_set<std::string> unique_colors_set;
    std::unordered_set<std::string> unique_objects_set;
    std::unordered_set<std::string> unique_color_objects_set;
    for (const auto& o : objects) {
        std::string color = o.property("color");
        std::string object = o.property("name");
        if (XItem::item_color_defined(color)) {
            unique_colors_set.insert(color);
            unique_color_objects_set.insert(color + " " + object);
        }
        unique_objects_set.insert(object);
    }
    uni_objects_ = std::vector<std::string>(
        unique_objects_set.begin(), unique_objects_set.end());
    uni_colors_ = std::vector<std::string>(
        unique_colors_set.begin(), unique_colors_set.end());
    uni_colored_objects_ = std::vector<std::string>(
        unique_color_objects_set.begin(), unique_color_objects_set.end());
    CHECK_GT(uni_objects_.size(), 0);
    CHECK_GT(uni_colors_.size(), 0);
    CHECK_GT(uni_colored_objects_.size(), 0);
}

}}  // namespace simulator::xwd
