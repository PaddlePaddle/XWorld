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

#pragma once
#include <memory>
#include <vector>
#include <ale_interface.hpp>
#include "arcade_simulator.h"

namespace simulator { namespace arcade_game {

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

    virtual void get_screen_out_dimensions(size_t& height, size_t& width, size_t& channels) override;

    void define_state_specs(StatePacket& state);

  private:
    ALEInterface ale_;

    static const size_t IMG_HEIGHT = 210UL;
    static const size_t IMG_WIDTH = 160UL;

    static const size_t IMG_HEIGHT_OUT = 84UL;
    static const size_t IMG_WIDTH_OUT = 84UL;

    GameFrame screen_rgb_;

    size_t lives_;

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
    void down_sample_image(const GameFrame& screen,
                           GameFrame& screen_out);
};

}} // namespace simulator::arcade_name
