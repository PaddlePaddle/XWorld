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

#include <cstdlib>
#include <gtest/gtest.h>
#include <games/deepmind_lab/deepmind_lab_simulator.h>

using namespace simulator;

DECLARE_string(level_script);
DECLARE_string(runfiles_path);

TEST(DMLAB_PLAYER, all_tests) {
    std::string level_script = __FILE__;
    int idx = level_script.find_last_of("/");
    level_script = level_script.substr(0, idx) + "/deepmind_lab_map.lua";
    FLAGS_level_script = level_script.c_str();

    FLAGS_runfiles_path = std::getenv("DEEPMIND_RUNFILES");
    // DeepMind Lab is a singleton, so all tests need to be sequential.
    std::shared_ptr<deepmind_lab_game::DeepmindLabSimulatorBase> world(
        deepmind_lab_game::DeepmindLabSimulatorBase::create());

    // game_over and reset_game
    EXPECT_NE(world->game_over(), 0);
    world->reset_game();
    EXPECT_EQ(world->game_over(), 0);
    // get_num_actions
    EXPECT_EQ(world->get_num_actions(), 11);

    // get_lives
    EXPECT_EQ(world->get_lives(), 1);

    // take_action
    // Look up blue sky to get rgb observation.
    StatePacket look_up;
    look_up.add_buffer_id("action", {2});
    for (int i = 0; i < 1000; i++) {
        EXPECT_DOUBLE_EQ(world->take_action(look_up), 0);
    }

    // get_screen
    StatePacket state;
    world->get_screen(state);
    auto screen = state.get_buffer("screen");
    auto fv = screen->get_value();
    EXPECT_EQ(screen->get_value_height(), 1);
    EXPECT_GT(screen->get_value_width(), 0);
    EXPECT_EQ(screen->get_value_width() % 3, 0);
    int length = screen->get_value_width() / 3;
    std::vector<double> avg_rgb(3);

    for (int i = 0; i < (int) avg_rgb.size(); i++) {
        for (int j = 0; j < length; j++) {
            avg_rgb[i] += fv->get_value(i * length + j);
        }

        avg_rgb[i] /= length;
    }

    // Check the observation is blue sky.
    EXPECT_LT(avg_rgb[0], 0.6*255);  // r
    EXPECT_GT(avg_rgb[1], 0.85*255); // g
    EXPECT_LT(avg_rgb[1], 0.9*255);  // g
    EXPECT_GT(avg_rgb[2], 0.95*255); // b

    // reward
    // Move forward until the agent touches item with reward 1.
    double reward = 0;
    StatePacket forward;
    forward.add_buffer_id("action", {6});
    for (int i = 0; i < 1000; i++) {
        reward = world->take_action(forward);

        if (reward > 0.5) {
            break;
        }
    }

    EXPECT_DOUBLE_EQ(reward, 1);
}

int main(int argc, char* argv[]) {
    srand(1);
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
