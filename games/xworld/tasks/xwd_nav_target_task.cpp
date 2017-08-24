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

class XWorldNavTargetTask : public XWorldTask {
  public:
    XWorldNavTargetTask(const std::string& name,
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

REGISTER_TASK(XWorldNavTargetTask);

void XWorldNavTargetTask::register_stages() {
    REGISTER_STAGE(idle);
    REGISTER_STAGE(simple_navigation_reward);
}

std::string XWorldNavTargetTask::idle(ScannerPtr scanner,
                                      SentenceTemplatePtr sen_temp,
                                      TeachingEnvPtr game) {
    return find_goal_and_generate_sentence(
        scanner, sen_temp, game, "simple_navigation_reward");
}

void XWorldNavTargetTask::define_sen_temp_rules(SentenceTemplatePtr sen_temp,
                                                TeachingEnvPtr game) {
    XWorldTask::define_sen_temp_rules(sen_temp, game);
    sen_temp->add_rule(sen_temp->start_symbol(),
                       {"$INSTRUCT", "$TIMEUP", "$END"},
                       true /*must_bound*/);
    sen_temp->add_rule("$END", {"Well done ."});
    sen_temp->add_rule("$TIMEUP", {"Time up ."});
    sen_temp->add_rule("$INSTRUCT",
                       {"$A $G please .",
                        "Please $A $G .",
                        "$A $G .",
                        "$G is your $D .",
                        "$G is the $D .",
                        "$Y $A $G ?"});
    sen_temp->add_rule("$G", uni_objects_, true /*must_bound*/);
    sen_temp->add_rule("$A", {"go to", "navigate to", "reach", "move to"});
    sen_temp->add_rule("$Y", {"Could you please", "Can you", "Will you"});
    sen_temp->add_rule("$D", {"destination", "target", "goal"});
}

std::vector<std::vector<Entity>> XWorldNavTargetTask::find_goal(
    ScannerPtr scanner) {
    std::vector<std::vector<Entity>> goal_sets;
    for (const auto& g : scanner->unique_goals_) {
        // Do not tell the agent go to where it is right now
        if (scanner->agent_.location == g.location) {
            continue;
        }
        goal_sets.push_back(std::vector<Entity>({g}));
    }
    return goal_sets;
}

void XWorldNavTargetTask::generate_sentence(
    const std::vector<std::vector<Entity>>& goal_sets,
    ScannerPtr scanner,
    SentenceTemplatePtr sen_temp,
    TeachingEnvPtr game) {
    auto goal_set = util::sample_set<std::vector<Entity>>(goal_sets);
    sen_temp->bind(sen_temp->start_symbol(), "$INSTRUCT");
    sen_temp->bind("$G", goal_set[0].property("name"));
    target_ = goal_set[0].location;
}
}
}  // namespace simulator::xwd
