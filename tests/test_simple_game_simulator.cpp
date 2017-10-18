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

#include <games/simple_game/simple_game_simulator.h>
#include <gtest/gtest.h>

using simulator::simple_game::SimpleGame;
using simulator::StatePacket;

TEST(SimpleGame, state_action_reward) {
    FLAGS_array_size = 8;
    auto game = std::make_shared<SimpleGame>();
    StatePacket screen;
    int pos = FLAGS_array_size / 2;
    for (int i = 0; i < (FLAGS_array_size - 1) / 2; ++i) {
        game->get_screen(screen);
        auto* data = screen.get_buffer("screen")->get_value<uint8_t>();
        // only the middle value is 1; all others should be 0s
        for (int j = 0; j < FLAGS_array_size; j++) {
            if (j != pos) {
                EXPECT_EQ(int(data[j]), 0);
            } else {
                EXPECT_EQ(int(data[pos]), 1);
            }
        }
        StatePacket a;
        a.add_buffer_id("action", {1});
        float reward = game->take_action(a);
        pos++;
        if (pos != FLAGS_array_size - 1) {
            EXPECT_NEAR(reward, -0.1, 1e-6);
        } else {
            EXPECT_NEAR(reward, 2.0, 1e-6);
        }
    }
}

int main(int argc, char* argv[]) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
