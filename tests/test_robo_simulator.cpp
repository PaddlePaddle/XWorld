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

#include <gtest/gtest.h>
#include <iostream>
#include <games/roboschool/robo_simulator.h>
#include <vector>

using simulator::robo_simulator::RoboSimulator;
using simulator::StatePacket;

TEST(RoboSimulator, Pass) {
    auto game = std::make_shared<RoboSimulator>(false);
    game->reset_game();
    EXPECT_EQ(game->game_over(), 0);
    EXPECT_EQ(game->get_num_actions(), 6);
    EXPECT_EQ(game->get_lives(), 1);
    StatePacket screen = StatePacket();
    game->get_screen(screen);
    auto* data = screen.get_buffer("screen")->get_value<uint8_t>();
    std::vector<int> action_vec;
    action_vec.push_back(0);
    StatePacket action;
    action.add_buffer_id("action", action_vec);
    double reward = game->take_action(action);
    EXPECT_LT(reward, 0);
}

void demo() {
    auto game = std::make_shared<RoboSimulator>(true);
    game->reset_game();
    StatePacket screen = StatePacket();
    while (1) {
        game->get_screen(screen);
        game->show_screen(0.0);
    }
}

int main(int argc, char *argv[]) {
    if (argc == 1) {
        testing::InitGoogleTest(&argc, argv);
        return RUN_ALL_TESTS();
    } else {
        demo();
        return 0;
    }
}
