// Copyright (c) 2017 Baidu Inc. All Rights Reserved.

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software // distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <stdlib.h>
#include <thread>
#include "simulator_interface.h"

using namespace simulator;

DECLARE_int32(context);
DEFINE_string(game, "simple_game", "name of game");
DEFINE_int32(num_agents, 1, "number of agents");
DEFINE_int32(port, 50000, "port number for socket communication");
DEFINE_int32(client_id, -1, "client id; -1 for server");

void run_simulation(SimulatorInterface* g) {
    g->start();
    g->reset_game();
    auto num_actions = g->get_num_actions();

    double reward = 0;
    double reward_per_game = 0;
    double r = 0;
    while (true) {
        auto game_over_str = GameSimulator::decode_game_over_code(g->game_over());
        if (game_over_str != "alive") {
            LOG(INFO) << "game over because of " + game_over_str;
            g->reset_game();
            reward_per_game = 0;
            continue;
        }
        g->show_screen(reward_per_game);

        StatePacket state;
        // You can choose to store the immediate reward r in the state
        // Or you can just ignore the first argument
        state = g->get_state(r);

        StatePacket actions;
        int a = util::get_rand_ind(num_actions);
        actions.add_buffer_id("action", {a});

        r = g->take_actions(actions, 1);
        reward_per_game += r;
        reward += r;
    }
    g->stop();

    LOG(INFO) << "total reward " << reward;
}

int main(int argc, char** argv) {
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    std::string name(FLAGS_game);

    int client_id = -1;
    for (int i = 0; i < FLAGS_num_agents; ++i) {
        if (fork() == 0) {
            client_id = i;
            break;
        }
    }

    if (client_id < 0) {
        LOG(INFO) << "server process id " << getpid();

        std::vector<std::thread> server_threads;
        for (int i = 0; i < FLAGS_num_agents; ++i) {
            int port = (FLAGS_port < 0) ? -1 : FLAGS_port + i;
            server_threads.emplace_back(
                [&](int port) {
                    SimulatorInterface* g = create_simulator(name, port);
                    run_simulation(g);
                },
                port
            );
        }
        for (auto& t : server_threads) { t.join(); }        
    } else {
        int port = FLAGS_port + client_id;
        LOG(INFO) << "client process id " << getpid();
        SimulatorClient client(name, port);
        client.start();
    }

    return 0;
}
