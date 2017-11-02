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

#pragma once
#include <ale_interface.hpp>
#include <memory>
#include <vector>
#include "arcade_simulator.h"

namespace simulator {
namespace arcade_game {

// The game that wraps the ALE (arcade learning enviroment) interface.
// It plays Atari arcade games.
class Arcade : public ArcadeGame {
  public:
    Arcade(const std::string& ale_rom);

    virtual ~Arcade() {}

    virtual void reset_game() override;

    virtual int game_over() override;

    virtual int get_num_actions() override;

    virtual void show_screen(float reward) override;

    virtual int get_lives() override;

    virtual float take_action(const StatePacket& actions) override;

    virtual void get_screen(StatePacket& screen) override;

    virtual void get_screen_out_dimensions(size_t& height,
                                           size_t& width,
                                           size_t& channels) override;

    void define_state_specs(StatePacket& state);

  private:
    ALEInterface ale_;

    static const size_t IMG_HEIGHT = 210UL;
    static const size_t IMG_WIDTH = 160UL;

    static const size_t IMG_HEIGHT_OUT = 84UL;
    static const size_t IMG_WIDTH_OUT = 84UL;

    GameFrame screen_rgb_;

    int lives_;

    // ALE has different RGB layout than what we needs.
    // Using a member variable instead of a local one
    // avoid allocating memory everytime get_screen_rgb is called.
    std::vector<uint8_t> screen_ale_;

    ActionVect minimal_actions_;

    void random_start(int n) {
        int t = util::get_rand_ind(n) + 1;
        for (int i = 0; i < t; ++i) {
            ale_.act(Action::PLAYER_A_NOOP);
        }
        ale_.act(Action::PLAYER_A_FIRE);
    }

    // get ALE RGB screen to screen_ale_ and convert it into screen_rgb_
    void get_screen_rgb();

    inline int rgb2y(int r, int g, int b) {
        return 0.299 * r + 0.587 * g + 0.114 * b;
    }

    // downsample and convert to grays
    void down_sample_image(const GameFrame& screen, GameFrame& screen_out);
};
}
}  // namespace simulator::arcade_name
