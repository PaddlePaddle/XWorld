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

class XWorldRecColorAndObjectTask : public XWorldTask {
  public:
    XWorldRecColorAndObjectTask(const std::string& name,
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

REGISTER_TASK(XWorldRecColorAndObjectTask);

void XWorldRecColorAndObjectTask::register_stages() {
    REGISTER_STAGE(idle);
    REGISTER_STAGE(simple_recognition_reward);
    REGISTER_STAGE(conversation_wrapup);
}

std::string XWorldRecColorAndObjectTask::idle(ScannerPtr scanner,
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

void XWorldRecColorAndObjectTask::define_sen_temp_rules(
    SentenceTemplatePtr sen_temp, TeachingEnvPtr game) {
    XWorldTask::define_sen_temp_rules(sen_temp, game);
    sen_temp->add_rule(sen_temp->start_symbol(),
                       {"$G location ?",
                        "$G where ?",
                        "Where is the $G ?",
                        "What is the location of $G ?",
                        "Where is $G located .",
                        "Which direction is the $G ?",
                        "Which side is the $G on you ?",
                        "Please locate $G .",
                        "Find $G .",
                        "The location of the $G is .",
                        "Say the location of the $G .",
                        "Identify the direction of the $G .",
                        "Tell the location of the $G ."});
    sen_temp->add_rule("$G", uni_colored_objects_, true /*must_bound*/);
}

std::vector<std::vector<Entity>> XWorldRecColorAndObjectTask::find_goal(
    ScannerPtr scanner) {
    std::vector<std::vector<Entity>> goal_sets;
    auto around = surrounding_filter(scanner->agent_, "color_goal", scanner);
    auto object_to_color = scanner->get_property_mapping("name", "color");
    auto color_to_object = scanner->get_property_mapping("color", "name");
    for (auto& a : around) {
        // this rule is satisfied if and only if:
        // 1. there are multiple goals with the same name but different colors,
        // OR
        // 2. there are multiple goals with the same color but different names
        auto name = a.property("name");
        auto color = a.property("color");
        if (util::check_unique_and_different(object_to_color[name], color) ||
            util::check_unique_and_different(color_to_object[color], name)) {
            goal_sets.push_back(std::vector<Entity>({a}));
        }
    }
    return goal_sets;
}

void XWorldRecColorAndObjectTask::generate_sentence(
    const std::vector<std::vector<Entity>>& goal_sets,
    ScannerPtr scanner,
    SentenceTemplatePtr sen_temp,
    TeachingEnvPtr game) {
    auto goal_set = util::sample_set<std::vector<Entity>>(goal_sets);
    sen_temp->bind(
        "$G",
        goal_set[0].property("color") + " " + goal_set[0].property("name"));
    answer_ = get_direction(scanner->agent_.location, goal_set[0].location);
}
}
}  // namespace simulator::xwd
