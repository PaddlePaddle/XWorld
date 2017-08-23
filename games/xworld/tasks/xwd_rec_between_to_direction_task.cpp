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
#include "../xworld_task.h"

namespace simulator { namespace xwd {

class XWorldRecBetweenToDirectionTask : public XWorldTask {
  public:
    XWorldRecBetweenToDirectionTask(const std::string& name,
                                    TeachingEnvPtr game,
                                    const std::vector<std::string>& held_out)
            : XWorldTask(name, game, held_out) {}
  private:
    void register_stages() override;

    std::string idle(ScannerPtr scanner, SentenceTemplatePtr sen_temp,
                     TeachingEnvPtr game) override;

    void define_sen_temp_rules(SentenceTemplatePtr sen_temp,
                               TeachingEnvPtr game) override;

    std::vector<std::vector<Entity>> find_goal(ScannerPtr scanner) override;

    void generate_sentence(
        const std::vector<std::vector<Entity>>& goal_sets,
        ScannerPtr scanner, SentenceTemplatePtr sen_temp, TeachingEnvPtr game) override;
};

REGISTER_TASK(XWorldRecBetweenToDirectionTask);

void XWorldRecBetweenToDirectionTask::register_stages() {
    REGISTER_STAGE(idle);
    REGISTER_STAGE(simple_recognition_reward);
    REGISTER_STAGE(conversation_wrapup);
}

std::string XWorldRecBetweenToDirectionTask::idle(ScannerPtr scanner, SentenceTemplatePtr sen_temp,
                                             TeachingEnvPtr game) {
    return find_goal_and_generate_sentence(
        scanner, sen_temp, game,
        FLAGS_task_mode == "arxiv_lang_acquisition"? "idle": "simple_recognition_reward");
}

void XWorldRecBetweenToDirectionTask::define_sen_temp_rules(SentenceTemplatePtr sen_temp, TeachingEnvPtr game) {
    XWorldTask::define_sen_temp_rules(sen_temp, game);
    sen_temp->add_rule(sen_temp->start_symbol(), {"$G location ?",
                    "$G where ?",
                    "Where is $G ?",
                    "What is the location of $G ?",
                    "Where is $G located .",
                    "Which direction is $G ?",
                    "Which side is $G on you ?",
                    "Please locate $G .",
                    "Find $G .",
                    "The location of $G is .",
                    "Say the location of $G .",
                    "Identify the direction of $G .",
                    "Tell the location of $G ."});
    sen_temp->add_rule("$G", {"the object between $O and $T"});
    sen_temp->add_rule("$O", uni_objects_, true/*must_bound*/);
    sen_temp->add_rule("$T", uni_objects_, true/*must_bound*/);
}

std::vector<std::vector<Entity>> XWorldRecBetweenToDirectionTask::find_goal(ScannerPtr scanner) {

    std::vector<std::vector<Entity>> goal_sets;
    std::vector<Entity> middle;
    std::vector<Entity> west_goals;
    std::vector<Entity> east_goals;
    between_two_goals(middle, west_goals, east_goals, scanner, false);
    for (size_t i = 0; i < middle.size(); i ++) {
        if (get_direction(scanner->agent_.location, middle[i].location) == "") {
            continue;
        }
        // To avoid the agent learns a trivial meaning of "between X and Y"
        // as "east of X" or "west of Y", we randomly switch the
        // positions of X and Y
        if (util::get_rand_range_val(1.0) < 0.5) {
            std::swap(west_goals[i], east_goals[i]);
        }
        goal_sets.push_back(std::vector<Entity>({middle[i], west_goals[i], east_goals[i]}));
    }
    return goal_sets;
}

void XWorldRecBetweenToDirectionTask::generate_sentence(
    const std::vector<std::vector<Entity>>& goal_sets,
    ScannerPtr scanner, SentenceTemplatePtr sen_temp, TeachingEnvPtr game) {

    auto goal_set = util::sample_set<std::vector<Entity>>(goal_sets);
    sen_temp->bind("$O", goal_set[1].property("name"));
    sen_temp->bind("$T", goal_set[2].property("name"));
    answer_ = get_direction(scanner->agent_.location, goal_set[0].location);
}

}} // namespace simulator::xwd
