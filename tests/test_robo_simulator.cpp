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
