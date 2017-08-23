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

class XWorldRecBetweenToColorTask : public XWorldTask {
  public:
    XWorldRecBetweenToColorTask(const std::string& name,
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

REGISTER_TASK(XWorldRecBetweenToColorTask);

void XWorldRecBetweenToColorTask::register_stages() {
    REGISTER_STAGE(idle);
    REGISTER_STAGE(simple_recognition_reward);
    REGISTER_STAGE(conversation_wrapup);
}

std::string XWorldRecBetweenToColorTask::idle(ScannerPtr scanner, SentenceTemplatePtr sen_temp,
                                             TeachingEnvPtr game) {
    return find_goal_and_generate_sentence(
        scanner, sen_temp, game,
        FLAGS_task_mode == "arxiv_lang_acquisition"? "idle": "simple_recognition_reward");
}

void XWorldRecBetweenToColorTask::define_sen_temp_rules(SentenceTemplatePtr sen_temp, TeachingEnvPtr game) {
    XWorldTask::define_sen_temp_rules(sen_temp, game);
    sen_temp->add_rule(sen_temp->start_symbol(), {"$G $X ?",
                    "$X of $G ?",
                    "Tell the $X of $G .",
                    "What $X does the $G have ?",
                    "What is the $X of $G ?",
                    "Identify the $X of $G .",
                    "Say the $X of $G ."});
    sen_temp->add_rule("$G", {"the object between $O and $T"});
    sen_temp->add_rule("$O", uni_objects_, true/*must_bound*/);
    sen_temp->add_rule("$T", uni_objects_, true/*must_bound*/);
    sen_temp->add_rule("$X", {"color", "property"});
}

std::vector<std::vector<Entity>> XWorldRecBetweenToColorTask::find_goal(ScannerPtr scanner) {

    std::vector<std::vector<Entity>> goal_sets;
    std::vector<Entity> middle;
    std::vector<Entity> west_goals;
    std::vector<Entity> east_goals;
    between_two_goals(middle, west_goals, east_goals, scanner, true);
    for (size_t i = 0; i < middle.size(); i ++) {
        if (!XItem::item_color_defined(middle[i].property("color"))) {
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

void XWorldRecBetweenToColorTask::generate_sentence(
    const std::vector<std::vector<Entity>>& goal_sets,
    ScannerPtr scanner, SentenceTemplatePtr sen_temp, TeachingEnvPtr game) {

    auto goal_set = util::sample_set<std::vector<Entity>>(goal_sets);
    sen_temp->bind("$O", goal_set[1].property("name"));
    sen_temp->bind("$T", goal_set[2].property("name"));
    answer_ = goal_set[0].property("color");
}

}} // namespace simulator::xwd
