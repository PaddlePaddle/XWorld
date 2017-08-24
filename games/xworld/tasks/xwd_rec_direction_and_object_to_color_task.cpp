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

class XWorldRecDirectionAndObjectToColorTask : public XWorldTask {
  public:
    XWorldRecDirectionAndObjectToColorTask(
        const std::string& name,
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

REGISTER_TASK(XWorldRecDirectionAndObjectToColorTask);

void XWorldRecDirectionAndObjectToColorTask::register_stages() {
    REGISTER_STAGE(idle);
    REGISTER_STAGE(simple_recognition_reward);
    REGISTER_STAGE(conversation_wrapup);
}

std::string XWorldRecDirectionAndObjectToColorTask::idle(
    ScannerPtr scanner, SentenceTemplatePtr sen_temp, TeachingEnvPtr game) {
    return find_goal_and_generate_sentence(
        scanner,
        sen_temp,
        game,
        FLAGS_task_mode == "arxiv_lang_acquisition"
            ? "idle"
            : "simple_recognition_reward");
}

void XWorldRecDirectionAndObjectToColorTask::define_sen_temp_rules(
    SentenceTemplatePtr sen_temp, TeachingEnvPtr game) {
    XWorldTask::define_sen_temp_rules(sen_temp, game);
    sen_temp->add_rule(sen_temp->start_symbol(),
                       {"$G $X ?",
                        "$X of $G ?",
                        "Tell the $X of $G .",
                        "What $X does the $G have ?",
                        "What is the $X of $G ?",
                        "Identify the $X of $G .",
                        "Say the $X of $G ."});
    sen_temp->add_rule("$G", {"$W $R $O"});
    sen_temp->add_rule("$W", directions_, true /*must_bound*/);
    sen_temp->add_rule("$R", {"to", "of", "near", "by"});
    sen_temp->add_rule("$O", uni_objects_, true /*must_bound*/);
    sen_temp->add_rule("$X", {"color", "property"});
}

std::vector<std::vector<Entity>>
XWorldRecDirectionAndObjectToColorTask::find_goal(ScannerPtr scanner) {
    std::vector<std::vector<Entity>> goal_sets;
    for (const auto& g : scanner->unique_goals_) {
        auto g_around = surrounding_filter(g, "color_goal", scanner);
        if (g_around.empty()) {
            continue;
        }
        auto a = util::sample_set<Entity>(g_around);
        goal_sets.push_back(std::vector<Entity>({g, a}));
    }
    return goal_sets;
}

void XWorldRecDirectionAndObjectToColorTask::generate_sentence(
    const std::vector<std::vector<Entity>>& goal_sets,
    ScannerPtr scanner,
    SentenceTemplatePtr sen_temp,
    TeachingEnvPtr game) {
    auto goal_set = util::sample_set<std::vector<Entity>>(goal_sets);
    sen_temp->bind("$W",
                   get_direction(goal_set[0].location, goal_set[1].location));
    sen_temp->bind("$O", goal_set[0].property("name"));
    answer_ = goal_set[1].property("color");
}
}
}  // namespace simulator::xwd
