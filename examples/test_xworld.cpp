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

#include "simulator_interface.h"
#include "games/xworld/xworld_simulator.h"

using namespace simulator::xwd;
using namespace simulator;

DECLARE_int32(context);
DECLARE_bool(task_groups_exclusive);
DECLARE_string(task_mode);

int main(int argc, char** argv) {
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    if (FLAGS_task_mode == "arxiv_lang_acquisition") {
        FLAGS_task_groups_exclusive = false;
    }

    FLAGS_xwd_conf_path = "../confs/walls.json";

    auto xwd = std::make_shared<SimulatorInterface>("xworld", false);
    xwd->reset_game();

    auto num_actions = xwd->get_num_actions();
    int act_rep = FLAGS_context;

    double reward = 0;
    double r = 0;
    bool show_screen = false;
    for (int i = 0; i < 100; i++) {
        auto game_over_str =
            GameSimulator::decode_game_over_code(xwd->game_over());
        auto state = xwd->get_state(r);

        StatePacket actions;
        actions.add_buffer_id("action", {util::get_rand_ind(num_actions)});
        actions.add_buffer_str("pred_sentence", "");
        r = xwd->take_actions(actions, act_rep, show_screen);

        if (game_over_str != "alive") {
            LOG(INFO) << "game over because of " + game_over_str;
            xwd->reset_game();
            continue;
        }

        reward += r;
        LOG(INFO) << r;
    }

    xwd->teacher_report_task_performance();

    LOG(INFO) << "total reward " << reward;
    return 0;
}
