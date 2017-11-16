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

#include "games/xworld3d/xworld3d_simulator.h"
#include "games/xworld3d/xworld3d_flags.h"

using namespace simulator::xworld3d;
using namespace simulator;

DECLARE_int32(context);
DECLARE_bool(pause_screen);
DECLARE_bool(task_groups_exclusive);

int main(int argc, char** argv) {
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    if (FLAGS_task_mode == "arxiv_lang_acquisition") {
        FLAGS_task_groups_exclusive = false;
    }

    FLAGS_x3_glsl_path = "../games/xworld3d/glsl";
    FLAGS_x3_conf = "../games/xworld3d/confs/empty_ground.json";
    const float gravity = 98;
    const float time_step = 0.00165;
    const int frame_skip = 5;

    X3Simulator game(FLAGS_x3_model_dir, false,
                     gravity, time_step, frame_skip, true/*big_screen*/);
    for (int e = 0; e < 100; ++e) {
        game.reset_game();
        auto num_actions = game.get_num_actions();
        int step = 0;
        while (game.game_over() == GameOverCode::ALIVE) {
            game.show_screen(0.0f);
            StatePacket actions;
            actions.add_buffer_id("action", {util::get_rand_ind(num_actions)});
            float r = game.take_actions(actions, 1);
            LOG(INFO) << "step: " << step++ << ", reward: " << r;
        }
    }
}
