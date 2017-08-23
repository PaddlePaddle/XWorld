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

class XWorldRecDirectionToObjectTask : public XWorldTask {
  public:
    XWorldRecDirectionToObjectTask(const std::string& name,
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

REGISTER_TASK(XWorldRecDirectionToObjectTask);

void XWorldRecDirectionToObjectTask::register_stages() {
    REGISTER_STAGE(idle);
    REGISTER_STAGE(simple_recognition_reward);
    REGISTER_STAGE(conversation_wrapup);
}

std::string XWorldRecDirectionToObjectTask::idle(
    ScannerPtr scanner, SentenceTemplatePtr sen_temp, TeachingEnvPtr game) {
    return find_goal_and_generate_sentence(
        scanner, sen_temp, game,
        FLAGS_task_mode == "arxiv_lang_acquisition"? "idle": "simple_recognition_reward");
}

void XWorldRecDirectionToObjectTask::define_sen_temp_rules(SentenceTemplatePtr sen_temp, TeachingEnvPtr game) {
    XWorldTask::define_sen_temp_rules(sen_temp, game);
    sen_temp->add_rule(sen_temp->start_symbol(), {"$G what ?",
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
    sen_temp->add_rule("$G", directions_, true/*must_bound*/);
    sen_temp->add_rule("$Z", {"object", "thing", "block", "grid"});
}

std::vector<std::vector<Entity>> XWorldRecDirectionToObjectTask::find_goal(ScannerPtr scanner) {

    std::vector<std::vector<Entity>> goal_sets;
    std::vector<Entity> around;
    if (util::get_rand_range_val(1.0) < 0.9) { // tell surrounding object
        around = surrounding_filter(scanner->agent_, "goal", scanner);
    } else { // tell surrounding empty grid
        around = surrounding_filter(scanner->agent_, "empty", scanner);
    }
    for (const auto& a : around) {
        goal_sets.push_back(std::vector<Entity>({a}));
    }
    return goal_sets;
}

void XWorldRecDirectionToObjectTask::generate_sentence(
    const std::vector<std::vector<Entity>>& goal_sets,
    ScannerPtr scanner, SentenceTemplatePtr sen_temp, TeachingEnvPtr game) {

    auto goal_set = util::sample_set<std::vector<Entity>>(goal_sets);
    sen_temp->bind("$G", get_direction(scanner->agent_.location, goal_set[0].location));
    answer_ = goal_set[0].property("name");
}

}} // namespace simulator::xwd
