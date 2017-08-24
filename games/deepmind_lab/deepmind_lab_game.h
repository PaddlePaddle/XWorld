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
#include <memory>
#include <vector>
#include "deepmind_lab_simulator.h"
#include "public/dmlab.h"

namespace simulator {
namespace deepmind_lab_game {

// This game wraps the DeepMind Lab interface.
class DeepmindLabGame : public DeepmindLabSimulatorBase {
  public:
    DeepmindLabGame();

    virtual ~DeepmindLabGame();

    void init();

    virtual void reset_game() override;

    virtual int game_over() override;

    virtual int get_num_actions() override;

    virtual void show_screen(float reward) override;

    virtual int get_lives() override;

    virtual float take_action(const StatePacket& actions) override;

    /**
     *  This method gets DeepMind Lab RGB observation and convert it to Paddle
     * input format.
     *
     * DeepMind Lab RGB observation:
     * Each value is unsigned int ranging [0, 255].
     * Values are stored in the order of height, width and color. color is the
     * innermost.
     * For example, a RGB image with Height = 2 and Width = 3 is stored in an
     * array like:
     * R(0,0), G(0,0), B(0,0), R(0,1), G(0,1), B(0,1), R(0,2), G(0,2), B(0,2),
     * R(1,0), G(1,0), B(1,0), R(1,1), G(1,1), B(1,1), R(1,2), G(1,2), B(1,2),
     * where, for X(H,W), X is one of RGB, H is height index and W is width
     * index.
     * For more details, please refer "Environment usage in Python" in
     * https://github.com/deepmind/lab/blob/master/docs/python_api.md, and
     * https://github.com/deepmind/lab/blob/master/python/dmlab_module.c#L410
     *
     * Paddle input format:
     * Each value is linearly normalized from [0, 255] to [0, 1].
     * Values are stored in the order of color, height and width. width is the
     * innermost.
     * For example, a RGB image with Height = 2 and Width = 3 is stored in an
     * array like:
     * R(0,0), R(0,1), R(0,2), R(1,0), R(1,1), R(1,2),
     * G(0,0), G(0,1), G(0,2), G(1,0), G(1,1), G(1,2),
     * B(0,0), B(0,1), B(0,2), B(1,0), B(1,1), B(1,2)
     * where, for X(H,W), X is one of RGB, H is height index and W is width
     * index.
     *
     */
    virtual void get_screen(StatePacket& screen) override;

    void define_state_specs(StatePacket& state);

    virtual void get_screen_out_dimensions(size_t& height,
                                           size_t& width,
                                           size_t& channels) override;

  protected:
    // Number of original actions.
    static const int N_ORIGINAL_ACTIONS = 7;

    // Set discrete actions. This is called in constructor.
    virtual void set_discrete_actions(
        std::vector<std::unique_ptr<int[]>>* actions_discrete);

  private:
    // Expected observation image size.
    static const int IMG_HEIGHT = 128;
    static const int IMG_WIDTH = 128;
    static const int IMG_DEPTH = 3;

    // Definition of discrete actions.
    std::vector<std::unique_ptr<int[]>> actions_discrete_;

    std::uniform_int_distribution<int> uniform_int_dist_;

    // Environment interface
    EnvCApi env_c_api_;

    // Environment status. This needs to be reset in reset_game().
    EnvCApi_EnvironmentStatus env_status_;

    // Opaque context to access environment. Details are defined in environment.
    void* context_;

    // Index of episode.
    int episode_;

    // Buffer for RGB values.
    std::vector<float> screen_vec_;

    static std::mutex s_dmlab_mutex_;
};
}
}  // namespace simulator::deepmind_lab_game
