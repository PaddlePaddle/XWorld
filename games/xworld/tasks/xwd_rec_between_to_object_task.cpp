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

#include "../xworld_task.h"
#include "teaching_task.h"

namespace simulator {
namespace xwd {

class XWorldRecBetweenToObjectTask : public XWorldTask {
  public:
    XWorldRecBetweenToObjectTask(const std::string& name,
                                 TeachingEnvPtr game,
                                 const std::vector<std::string>& held_out)
        : XWorldTask(name, game, held_out) {}

  private:
    void register_stages() override;

    std::string idle(ScannerPtr scanner,
                     SentenceTemplatePtr sen_temp,
                     TeachingEnvPtr game) override;

    void define_sen_temp_rules(SentenceTemplatePtr sen_temp,
                               TeachingEnvPtr game) override;

    std::vector<std::vector<Entity>> find_goal(ScannerPtr scanner) override;

    void generate_sentence(const std::vector<std::vector<Entity>>& goal_sets,
                           ScannerPtr scanner,
                           SentenceTemplatePtr sen_temp,
                           TeachingEnvPtr game) override;
};

REGISTER_TASK(XWorldRecBetweenToObjectTask);

void XWorldRecBetweenToObjectTask::register_stages() {
    REGISTER_STAGE(idle);
    REGISTER_STAGE(simple_recognition_reward);
    REGISTER_STAGE(conversation_wrapup);
}

std::string XWorldRecBetweenToObjectTask::idle(ScannerPtr scanner,
                                               SentenceTemplatePtr sen_temp,
                                               TeachingEnvPtr game) {
    return find_goal_and_generate_sentence(
        scanner,
        sen_temp,
        game,
        FLAGS_task_mode == "arxiv_lang_acquisition"
            ? "idle"
            : "simple_recognition_reward");
}

void XWorldRecBetweenToObjectTask::define_sen_temp_rules(
    SentenceTemplatePtr sen_temp, TeachingEnvPtr game) {
    XWorldTask::define_sen_temp_rules(sen_temp, game);
    sen_temp->add_rule(sen_temp->start_symbol(),
                       {"$G what ?",
                        "What $Z in $G ?",
                        "Name of the $Z in $G ?",
                        "The $Z in $G ?",
                        "What is in $G ?",
                        "What is the $Z in $G ?",
                        "What is $G ?",
                        "Say the $Z in $G .",
                        "Identify the $Z in $G .",
                        "Tell the name of the $Z which is $G .",
                        "The $Z in $G is ?"});
    sen_temp->add_rule("$G", {"between $O and $T"});
    sen_temp->add_rule("$O", uni_objects_, true /*must_bound*/);
    sen_temp->add_rule("$T", uni_objects_, true /*must_bound*/);
    sen_temp->add_rule("$Z", {"object", "thing", "block", "grid"});
}

std::vector<std::vector<Entity>> XWorldRecBetweenToObjectTask::find_goal(
    ScannerPtr scanner) {
    std::vector<std::vector<Entity>> goal_sets;
    std::vector<Entity> middle;
    std::vector<Entity> west_goals;
    std::vector<Entity> east_goals;
    between_two_goals(middle,
                      west_goals,
                      east_goals,
                      scanner,
                      util::get_rand_range_val(1.0) < 0.5 ? true : false);
    for (size_t i = 0; i < middle.size(); i++) {
        // To avoid the agent learns a trivial meaning of "between X and Y"
        // as "east of X" or "west of Y", we randomly switch the
        // positions of X and Y
        if (util::get_rand_range_val(1.0) < 0.5) {
            std::swap(west_goals[i], east_goals[i]);
        }
        goal_sets.push_back(
            std::vector<Entity>({middle[i], west_goals[i], east_goals[i]}));
    }
    return goal_sets;
}

void XWorldRecBetweenToObjectTask::generate_sentence(
    const std::vector<std::vector<Entity>>& goal_sets,
    ScannerPtr scanner,
    SentenceTemplatePtr sen_temp,
    TeachingEnvPtr game) {
    auto goal_set = util::sample_set<std::vector<Entity>>(goal_sets);
    sen_temp->bind("$O", goal_set[1].property("name"));
    sen_temp->bind("$T", goal_set[2].property("name"));
    answer_ = goal_set[0].property("name");
}
}
}  // namespace simulator::xwd
