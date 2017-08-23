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
#include <games/simple_game/simple_game_simulator.h>

using simulator::simple_game::SimpleGame;
using simulator::StatePacket;

TEST(SimpleGame, state_action_reward) {
    int array_size = 8;
    auto game = std::make_shared<SimpleGame>(array_size);
    StatePacket screen;
    int pos = array_size / 2;
    for (int i = 0; i < (array_size-1)/2; ++i) {
        game->get_screen(screen);
        auto* data = screen.get_buffer("screen")->get_value<uint8_t>();
        // only the middle value is 1; all others should be 0s
        for (int j = 0; j < array_size; j ++) {
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
        if (pos != array_size - 1) {
            EXPECT_NEAR(reward, -0.1, 1e-6);
        } else {
            EXPECT_NEAR(reward, 2.0, 1e-6);
        }
    }

}

int main(int argc, char *argv[]) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
