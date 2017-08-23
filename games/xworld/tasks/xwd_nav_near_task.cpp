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

#include "teaching_task.h"
#include "../xworld_task.h"

namespace simulator { namespace xwd {

class XWorldNavNearTask : public XWorldTask {
  public:
    XWorldNavNearTask(const std::string& name,
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

REGISTER_TASK(XWorldNavNearTask);

void XWorldNavNearTask::register_stages() {
    REGISTER_STAGE(idle);
    REGISTER_STAGE(simple_navigation_reward);
}

std::string XWorldNavNearTask::idle(ScannerPtr scanner, SentenceTemplatePtr sen_temp,
                                    TeachingEnvPtr game) {
    return find_goal_and_generate_sentence(scanner, sen_temp, game, "simple_navigation_reward");
}

void XWorldNavNearTask::define_sen_temp_rules(SentenceTemplatePtr sen_temp, TeachingEnvPtr game) {
    XWorldTask::define_sen_temp_rules(sen_temp, game);
    sen_temp->add_rule(sen_temp->start_symbol(),
                       {"$INSTRUCT", "$TIMEUP", "$END"}, true/*must_bound*/);
    sen_temp->add_rule("$END", {"Well done ."});
    sen_temp->add_rule("$TIMEUP", {"Time up"});
    sen_temp->add_rule("$INSTRUCT", {"$G .",
                    "$A $G please .",
                    "Please $A $G .",
                    "$A $G .",
                    "$G is your $D .",
                    "$G is the $D .",
                    "$Y $A $G ?"});
    sen_temp->add_rule("$G", {"$W $R $O"});
    sen_temp->add_rule("$W", directions_, true/*must_bound*/);
    sen_temp->add_rule("$R", {"to", "of", "near", "by"});
    sen_temp->add_rule("$O", uni_objects_, true/*must_bound*/);
    sen_temp->add_rule("$A", {"go to",
                              "navigate to",
                              "reach",
                              "move to"});
    sen_temp->add_rule("$Y", {"Could you please",
                              "Can you",
                              "Will you"});
    sen_temp->add_rule("$D", {"destination", "target", "goal"});
}

std::vector<std::vector<Entity>> XWorldNavNearTask::find_goal(ScannerPtr scanner) {

    std::vector<std::vector<Entity>> goal_sets;
    for (auto& g : scanner->unique_goals_) {
        auto around = get_target_around_reachable(g, scanner->agent_, scanner);
        if (around.empty()) {
            continue;
        }
        Entity d;
        d.location = util::sample_set<Vec3>(around);
        // Do not tell the agent go to where it is right now
        if (d.location == scanner->agent_.location) {
            continue;
        }
        goal_sets.push_back(std::vector<Entity>({g, d}));
    }
    return goal_sets;
}

void XWorldNavNearTask::generate_sentence(
    const std::vector<std::vector<Entity>>& goal_sets,
    ScannerPtr scanner, SentenceTemplatePtr sen_temp, TeachingEnvPtr game) {

    auto goal_set = util::sample_set<std::vector<Entity>>(goal_sets);
    sen_temp->bind(sen_temp->start_symbol(), "$INSTRUCT");
    sen_temp->bind("$W", get_direction(goal_set[0].location, goal_set[1].location));
    sen_temp->bind("$O", goal_set[0].property("name"));
    target_ = goal_set[1].location;
}

}} // namespace simulator::xwd
