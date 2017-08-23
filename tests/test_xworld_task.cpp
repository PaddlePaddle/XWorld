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

#include "games/xworld/xworld_task.h"
#include "games/xworld/xworld_simulator.h"
#include "data_packet.h"
#include "dirent.h"
#include <gtest/gtest.h>
#include <gflags/gflags.h>
#include <memory>
#include <cmath>
#include <vector>
#include <functional>

DECLARE_bool(task_groups_exclusive);

using namespace std;
using namespace simulator;
using namespace simulator::xwd;

typedef std::function<bool (const std::string& sentence)> TestFunc;

enum ACTION { UP, DOWN, LEFT, RIGHT };

const double EPS = 1e-2;

const double time_penalty = XWorldTask::time_penalty;
const double correct_reward = XWorldTask::correct_reward;
const double wrong_reward = XWorldTask::wrong_reward;
const double failed_action_reward = XWorldTask::failed_action_reward;

const std::string EMPTY_STR = "";

std::string get_json_dir() {
    std::string path = __FILE__;
    int idx = path.find_last_of("/");
    return path.substr(0, idx) + "/../games/xworld/confs/test_xworld_task";
}

std::string get_teacher_sentence(shared_ptr<XWorldSimulator> game) {
    TeachingEnvPtr tg = game;
    return tg->get_teacher_sent_from_buffer();
}

void test_xworld_task(const std::string& xworld_json, TestFunc test_function) {
    auto game = make_shared<XWorldSimulator>(true, xworld_json, 0, "one_channel");
    game->add_agent("robot_0");

    TeacherPtr teacher = std::make_shared<Teacher>(xworld_json, game, false);

    // when teacher is not watching, we expect it returns false
    teacher->stop();
    EXPECT_EQ(teacher->teach(), false);

    teacher->start();
    teacher->teach();

    std::string sentence = get_teacher_sentence(game);
    EXPECT_EQ(test_function(sentence), true);

    teacher->stop();
}

StatePacket wrap_action(int action_id) {
    vector<int> actions = {action_id};
    StatePacket s;
    s.add_buffer_id("action", actions);
    s.add_buffer_str("pred_sentence", "");
    return s;
}

// check contain *any* word in key_words
bool contain_key_words(const std::vector<std::string>& key_words,
                       const std::string& sentence) {
    for (auto w : key_words) {
        if (sentence.find(w) != std::string::npos) {
            return true;
        }
    }
    return false;
}

/**
   The agent takes a sequence of actions, after which we check the
   immediate reward and teacher's sentence at the last step
**/
void go_one_way(shared_ptr<XWorldSimulator> game,
                TeacherPtr teacher,
                const vector<int>& actions,
                const string& expected_word,
                double expected_reward) {
    for (auto a : actions) {
        game->take_action(wrap_action(a));
    }
    teacher->teach(); // back to "idle" stage
    EXPECT_LT(fabs(teacher->give_reward() - expected_reward), EPS); // correct_reward + time_penalty
    teacher->teach(); // issue command
    EXPECT_LT(fabs(teacher->give_reward()), EPS);
    EXPECT_EQ(contain_key_words(vector<string>({expected_word}),
                                get_teacher_sentence(game)), true);
    // The agent hasn't moved, so the teacher thinks that it has a failed action
    teacher->teach();
    EXPECT_LT(fabs(teacher->give_reward()
                   - (failed_action_reward + time_penalty + wrong_reward)), EPS);
};

TEST(XWorldTask, RecDirectionToObject) {
    auto test_function = [] (const std::string& sentence) {
        bool test = contain_key_words(
            std::vector<std::string>({"north"}), sentence);
        if (sentence.find("nothing") == std::string::npos) {
            return test;
        } else {
            return !test;
        }
    };
    test_xworld_task(get_json_dir() + "/rec_direction_to_object.json",
                     test_function);
}

TEST(XWorldTask, RecObjectToColor) {
    auto test_function = [] (const std::string& sentence) {
        return contain_key_words(
            std::vector<std::string>({"banana"}), sentence);
    };
    test_xworld_task(get_json_dir() + "/rec_object_to_color.json",
                     test_function);
}

TEST(XWorldTask, RecBetweenToObject) {
    auto test_function = [] (const std::string& sentence) {
        return contain_key_words(
            std::vector<std::string>(
                {"between", "apple", "banana"}), sentence)
        || sentence == EMPTY_STR;  // 0.5 prob of telling empty between objects but find no such pair
    };
    test_xworld_task(get_json_dir() + "/rec_between_to_object.json",
                     test_function);
}

TEST(XWorldTask, LanDirectionToObject) {
    auto test_function = [] (const std::string& sentence) {
        bool test = contain_key_words(
            std::vector<std::string>({"north"}), sentence);
        if (sentence.find("nothing") == std::string::npos) {
            return test;
        } else {
            return !test;
        }
    };
    test_xworld_task(get_json_dir() + "/lan_direction_to_object.json",
                     test_function);
}

TEST(XWorldTask, NavNear) {
    auto test_function = [] (const std::string& sentence) {
        return contain_key_words(
            std::vector<std::string>({"east"}), sentence);
    };
    test_xworld_task(get_json_dir() + "/nav_near.json",
                     test_function);
}

TEST(XWorldTask, NavColorTarget) {
    auto test_function = [] (const std::string& sentence) {
        return contain_key_words(
            std::vector<std::string>({"red"}), sentence);
    };
    test_xworld_task(get_json_dir() + "/nav_color_target.json",
                     test_function);
}

TEST(XWorldTask, NavHeldOut) {
    auto test_function = [] (const std::string& sentence) {
        return sentence == EMPTY_STR; // held out "apple"
    };
    test_xworld_task(get_json_dir() + "/nav_heldout.json",
                     test_function);
}

TEST(XWorldTask, ContinuousTask) {
    auto json = get_json_dir() + "/nav_continuous.json";
    auto game = make_shared<XWorldSimulator>(true, json, 0, "one_channel");
    game->add_agent("robot_0");

    TeacherPtr teacher = std::make_shared<Teacher>(json, game, false);

    teacher->teach(); // issue a command
    auto sentence = get_teacher_sentence(game);

    teacher->teach();
    // the agent hasn't moved yet and thus has a failed action
    EXPECT_LT(fabs(teacher->give_reward() - (failed_action_reward + time_penalty)), EPS);

    // go to "apple"
    if (sentence.find("apple") != string::npos) {
        // -> "apple"
        go_one_way(game, teacher, vector<int>({UP, LEFT}), "banana",
                   correct_reward + time_penalty);
        // "apple" -> "banana"
        go_one_way(game, teacher, vector<int>({DOWN, DOWN, RIGHT, RIGHT}), "apple",
                   correct_reward + time_penalty);
    } else { // go to "banana"
        // -> "banana"
        go_one_way(game, teacher, vector<int>({DOWN, RIGHT}), "apple",
                   correct_reward + time_penalty);
        // "banana" -> "apple"
        go_one_way(game, teacher, vector<int>({UP, UP, LEFT, LEFT}), "banana",
                   correct_reward + time_penalty);
    }

    teacher->stop();
}

TEST(XWorldTask, MultiTask) {
    FLAGS_task_groups_exclusive = false;
    auto json = get_json_dir() + "/nav_multitask.json";
    auto game = make_shared<XWorldSimulator>(true, json, 0, "one_channel");
    game->add_agent("robot_0");

    TeacherPtr teacher = std::make_shared<Teacher>(json, game, false);

    // issue a navigation command; it occupies the language channel so
    // that there will be no rec question
    teacher->teach();
    auto sentence = get_teacher_sentence(game);
    EXPECT_EQ(sentence.find("banana") != string::npos, true);
    EXPECT_EQ(sentence.find("yellow") == string::npos, true);

    //// when the agent has not finished the task, the teacher will not issue new command
    game->take_action(wrap_action(UP));
    // Thus the teacher will definitely ask rec questions using the language channel
    teacher->teach();

    sentence = get_teacher_sentence(game);
    EXPECT_EQ(contain_key_words(vector<string>({"yellow"}), sentence), true);
    // the navigation task is still on
    EXPECT_LT(fabs(teacher->give_reward() - time_penalty), EPS); // time penalty

    // agent finishes nav task
    game->take_action(wrap_action(LEFT));

    // nav task back to "idle" but rec task goes to "wrapup"
    teacher->teach();

    // rec task back to "idle"
    teacher->teach();

    // agent stands on goal, no nav command will be issued. Instead, question is asked
    teacher->teach();
    sentence = get_teacher_sentence(game);
    EXPECT_EQ(contain_key_words(vector<string>({"yellow"}), sentence), true);

    EXPECT_LT(fabs(teacher->give_reward()), EPS);

    // teacher gives answer
    teacher->teach();
    // rec task from "wrapup" to "idle"
    teacher->teach();

    // both nav and rec tasks are already "idle"
    EXPECT_EQ(teacher->is_idle(), true);

    teacher->stop();
}

TEST(XWorldTask, TaskWeight) {
    auto json = get_json_dir() + "/task_weight.json";
    auto game = make_shared<XWorldSimulator>(true, json, 0, "one_channel");
    TeachingEnvPtr teach_game = game;
    game->add_agent("robot_0");

    TeacherPtr teacher = std::make_shared<Teacher>(json, game, false);

    const int total = 100000;
    int nav_target_count = 0;
    int nav_color_target_count = 0;
    int rec_count = 0;
    int empty_count = 0;
    for (int i = 0; i < total; i ++) {
        teacher->teach();
        auto task = teach_game->get_teacher_sent_type_from_buffer();
        if (task == "XWorldNavTargetTask") {
            nav_target_count ++;
        } else if (task == "XWorldNavColorTargetTask") {
            nav_color_target_count ++;
        } else if (task == "XWorldRecDirectionToColorTask") {
            rec_count ++;
        } else {
            empty_count ++;
        }
        teacher->reset_after_game_reset();
    }
    EXPECT_LT(fabs(double(nav_target_count) / total - 0.4), EPS);
    EXPECT_LT(fabs(double(nav_color_target_count) / total - 4.0/15), EPS);
    EXPECT_LT(fabs(double(rec_count) / total - 1.0/3), EPS);
    EXPECT_EQ(empty_count, 0);

    teacher->stop();
}

TEST(XWorldTask, PressureTest) {
    auto json = get_json_dir() + "/pressure_test.json";
    auto game = make_shared<XWorldSimulator>(true, json, 0, "one_channel");
    game->add_agent("robot_0");

    TeacherPtr teacher = std::make_shared<Teacher>(json, game, false);

    const int total = 10000;
    for (int i = 0; i < total; i ++) {
        teacher->teach();
    }

    teacher->stop();
}

TEST(XWorldTask, GameOver) {
    auto json = get_json_dir() + "/nav_near.json";
    auto game = make_shared<XWorldSimulator>(true, json, 0, "arxiv_lang_acquisition");
    game->add_agent("robot_0");
    TeacherPtr teacher = std::make_shared<Teacher>(json, game, false);
    teacher->teach();
    game->take_action(wrap_action(LEFT));
    teacher->teach();  // teacher should now record event "correct_goal"
    EXPECT_EQ(game->game_over(), SUCCESS);
    teacher->stop();
}

int main(int argc, char *argv[]) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
