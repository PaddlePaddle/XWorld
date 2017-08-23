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

class XWorldRecObjectToColorTask : public XWorldTask {
  public:
    XWorldRecObjectToColorTask(const std::string& name,
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

REGISTER_TASK(XWorldRecObjectToColorTask);

void XWorldRecObjectToColorTask::register_stages() {
    REGISTER_STAGE(idle);
    REGISTER_STAGE(simple_recognition_reward);
    REGISTER_STAGE(conversation_wrapup);
}

std::string XWorldRecObjectToColorTask::idle(
    ScannerPtr scanner, SentenceTemplatePtr sen_temp, TeachingEnvPtr game) {
    return find_goal_and_generate_sentence(
        scanner, sen_temp, game,
        FLAGS_task_mode == "arxiv_lang_acquisition"? "idle": "simple_recognition_reward");
}

void XWorldRecObjectToColorTask::define_sen_temp_rules(SentenceTemplatePtr sen_temp, TeachingEnvPtr game) {
    XWorldTask::define_sen_temp_rules(sen_temp, game);
    sen_temp->add_rule(sen_temp->start_symbol(), {"$G $X ?",
                    "$X of $G ?",
                    "Tell the $X of $G .",
                    "What $X does the $G have ?",
                    "What is the $X of $G ?",
                    "Identify the $X of $G .",
                    "Say the $X of $G ."});
    sen_temp->add_rule("$G", uni_objects_, true/*must_bound*/);
    sen_temp->add_rule("$X", {"color", "property"});
}

std::vector<std::vector<Entity>> XWorldRecObjectToColorTask::find_goal(ScannerPtr scanner) {

    std::vector<std::vector<Entity>> goal_sets;
    auto object_to_color = scanner->get_property_mapping("name", "color");
    for (auto& g : scanner->color_goals_) {
        auto name = g.property("name");
        // we have unique answer of the color
        if (object_to_color[name].size() == 1) {
            goal_sets.push_back(std::vector<Entity>({g}));
        }
    }
    return goal_sets;
}

void XWorldRecObjectToColorTask::generate_sentence(
    const std::vector<std::vector<Entity>>& goal_sets,
    ScannerPtr scanner, SentenceTemplatePtr sen_temp, TeachingEnvPtr game) {

    auto goal_set = util::sample_set<std::vector<Entity>>(goal_sets);
    sen_temp->bind("$G", goal_set[0].property("name"));
    answer_ = goal_set[0].property("color");
}

}} // namespace simulator::xwd
