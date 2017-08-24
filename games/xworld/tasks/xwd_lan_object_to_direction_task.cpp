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

class XWorldLanObjectToDirectionTask : public XWorldTask {
  public:
    XWorldLanObjectToDirectionTask(const std::string& name,
                                   TeachingEnvPtr game,
                                   const std::vector<std::string>& held_out)
        : XWorldTask(name, game, held_out) {}

  private:
    void register_stages() override;

    std::string idle(ScannerPtr scanner,
                     SentenceTemplatePtr sen_temp,
                     TeachingEnvPtr game) override;

    // provide feedback in the form of both reward and verbal feedback
    std::string reward(ScannerPtr scanner,
                       SentenceTemplatePtr sen_temp,
                       TeachingEnvPtr game);

    void define_sen_temp_rules(SentenceTemplatePtr sen_temp,
                               TeachingEnvPtr game) override;

    // a private function for generating answers
    void define_sen_temp_rules_for_answer(SentenceTemplatePtr sen_temp);

    std::vector<std::vector<Entity>> find_goal(ScannerPtr scanner) override;

    void generate_sentence(const std::vector<std::vector<Entity>>& goal_sets,
                           ScannerPtr scanner,
                           SentenceTemplatePtr sen_temp,
                           TeachingEnvPtr game) override;

    // storing all possible correct answers to a question
    std::vector<std::string> answer_set_;
};

REGISTER_TASK(XWorldLanObjectToDirectionTask);

void XWorldLanObjectToDirectionTask::register_stages() {
    REGISTER_STAGE(idle);
    REGISTER_STAGE(reward);
    REGISTER_STAGE(conversation_wrapup);
}

std::string XWorldLanObjectToDirectionTask::idle(ScannerPtr scanner,
                                                 SentenceTemplatePtr sen_temp,
                                                 TeachingEnvPtr game) {
    // start_question is only a aux function and is not a stage function
    // define_sen_temp_rules is called when init,, but was later modified in
    // reward_and_answer
    // thus here need to re-establish the rules
    define_sen_temp_rules(sen_temp, game);
    return find_goal_and_generate_sentence(scanner, sen_temp, game, "reward");
}

std::string XWorldLanObjectToDirectionTask::reward(ScannerPtr scanner,
                                                   SentenceTemplatePtr sen_temp,
                                                   TeachingEnvPtr game) {
    // check if the reply from agent is correct
    is_reply_correct_ =
        (util::compare_sentences_multi(
             answer_set_, scanner->scan_agent_sent_from_env()) > 0.99);
    // one way of reproducing a single sentence
    // add generation rule from start symbol to $F
    sen_temp->add_rule(sen_temp->start_symbol(), {"$F", "$P $F"});

    // bind the correct answer to question
    sen_temp->add_rule("$F", {answer_}, true);
    if (is_reply_correct_) {
        sen_temp->add_rule("$P", {"yes"});
    } else {
        sen_temp->add_rule("$P", {"no"});
    }

    if (Task::teacher_speak(/*held_out*/ false, name_, sen_temp, game)) {
        // continue to compute the reward
        double speak_correct_reward = 1.0;
        double speak_wrong_reward = -speak_correct_reward;
        // use compare sentence
        if (is_reply_correct_) {
            give_reward(speak_correct_reward);
        } else {
            give_reward(speak_wrong_reward);
        }
    }
    return "conversation_wrapup";
}

void XWorldLanObjectToDirectionTask::define_sen_temp_rules(
    SentenceTemplatePtr sen_temp, TeachingEnvPtr game) {
    // will increase the variations of the questions later
    sen_temp->add_rule(sen_temp->start_symbol(), {"where is $O"});
    sen_temp->add_rule("$O", {"dummy"}, true /*must_bound*/);
}

void XWorldLanObjectToDirectionTask::define_sen_temp_rules_for_answer(
    SentenceTemplatePtr sen_temp) {
    sen_temp->add_rule(sen_temp->start_symbol(),
                       {"on the $D is $O", "$O is on the $D"});
    sen_temp->add_rule("$D", directions_, true /*must_bound*/);
    sen_temp->add_rule("$O", {"dummy"}, true /*must_bound*/);
}

std::vector<std::vector<Entity>> XWorldLanObjectToDirectionTask::find_goal(
    ScannerPtr scanner) {
    std::vector<std::vector<Entity>> goal_sets;
    std::vector<Entity> around;
    around = surrounding_filter(scanner->agent_, "goal", scanner);
    for (const auto& a : around) {
        goal_sets.push_back(std::vector<Entity>({a}));
    }
    return goal_sets;
}

void XWorldLanObjectToDirectionTask::generate_sentence(
    const std::vector<std::vector<Entity>>& goal_sets,
    ScannerPtr scanner,
    SentenceTemplatePtr sen_temp,
    TeachingEnvPtr game) {
    auto goal_set = util::sample_set<std::vector<Entity>>(goal_sets);
    std::string dir =
        get_direction(scanner->agent_.location, goal_set[0].location);
    sen_temp->add_rule(
        "$O", {goal_set[0].property("name")}, true /*must_bound*/);
    sen_temp->bind("$O", goal_set[0].property("name"));

    // could be a set of potential answers, use a local sen_temp for generating
    // diverse answers
    SentenceTemplatePtr sen_temp_answer;
    sen_temp_answer = std::make_shared<TeacherSentenceTemplate>();
    define_sen_temp_rules_for_answer(sen_temp_answer);
    sen_temp_answer->add_rule(
        "$O", {goal_set[0].property("name")}, true /*must_bound*/);
    sen_temp_answer->bind("$O", goal_set[0].property("name"));
    sen_temp_answer->bind("$D", dir);

    // store a set of correct replied in answer_set_ and use it for reward in
    // the
    // next step
    // rather than using gt_reply_ which is set by the return value of this
    // function
    answer_set_.clear();  // clear old answer_set
    sen_temp_answer->instantiate_all_sentences(answer_set_);
    // randomly sample one correct sentence from the set as the feedback from
    // teacher
    answer_ = answer_set_[util::get_rand_ind(answer_set_.size())];
}
}
}  // namespace simulator::xwd
