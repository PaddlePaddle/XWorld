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

#include "games/xworld/xworld_simulator.h"

using namespace simulator::xwd;
using namespace simulator;

DECLARE_int32(context);
DECLARE_bool(task_groups_exclusive);
DECLARE_string(task_mode);
DECLARE_bool(pause_screen);

void game_reset_with_teacher(SimulatorPtr game, TeacherPtr teacher) {
    game->reset_game();
    teacher->reset_after_game_reset();
    teacher->teach();
}

int main(int argc, char** argv) {
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    if (FLAGS_task_mode == "arxiv_lang_acquisition") {
        FLAGS_task_groups_exclusive = false;
    }

    std::string conf_path = "../confs/empty_ground.json";
    int curriculum = 0;
    auto xwd = std::make_shared<XWorldSimulator>(
        true /*print*/, conf_path, curriculum);

    int agent_id = xwd->add_agent("robot0");
    auto game = std::make_shared<AgentSpecificSimulator>(xwd, agent_id);
    auto teacher = std::make_shared<Teacher>(conf_path, xwd, false /*print*/);
    game_reset_with_teacher(game, teacher);

    auto num_actions = game->get_num_actions();
    int act_rep = FLAGS_context;

    teacher->print_total_possible_sentences();

    double reward = 0;
    double reward_per_game = 0;
    double r = 0;
    for (int i = 0; i < 100; i++) {
        auto game_over_str =
            GameSimulator::decode_game_over_code(game->game_over());
        if (game_over_str != "alive") {
            LOG(INFO) << "game over because of " + game_over_str;
            game_reset_with_teacher(game, teacher);
            reward_per_game = 0;
            continue;
        }
        game->show_screen(reward_per_game);

        StatePacket state;
        // You can choose to store the immediate reward r in the state
        // Or you can just ignore the first argument
        game->get_state_data(r, state);

        StatePacket actions;
        actions.add_buffer_id("action", {util::get_rand_ind(num_actions)});
        r = game->take_actions(actions, act_rep);
        teacher->teach();
        r += teacher->give_reward();

        reward_per_game += r;
        reward += r;
        LOG(INFO) << r;
    }

    teacher->report_task_performance();

    LOG(INFO) << "total reward " << reward;
    return 0;
}
