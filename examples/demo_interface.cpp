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

#include <stdlib.h>
#include <thread>
#include "simulator_interface.h"

using namespace simulator;

DEFINE_string(game, "simple_game", "name of the game");

void run_simulation(std::shared_ptr<SimulatorInterface> g) {
    g->start();
    g->reset_game();
    auto num_actions = g->get_num_actions();

    double reward = 0;
    double r = 0;
    size_t cnt = 0;
    bool show_screen = false;
    while (cnt < 100) {
        auto game_over_str = g->game_over_string();
        if (game_over_str != "alive") {
            LOG(INFO) << "game over because of " + game_over_str;
            g->reset_game();
            continue;
        }

        StatePacket state;
        // You can choose to store the immediate reward r in the state
        // Or you can just ignore the first argument
        state = g->get_state(r);

        StatePacket actions;
        int a = util::get_rand_ind(num_actions);
        actions.add_buffer_id("action", {a});

        r = g->take_actions(actions, 1, show_screen);
        reward += r;
        cnt ++;
    }
    g->stop();

    LOG(INFO) << "total reward " << reward;
}

int main(int argc, char** argv) {
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    std::string name = FLAGS_game;
    CHECK(name != "xworld" || name != "xworld3d")
            << " This program is for demostration of multiprocesses. It does"
            << " not support games with more than one type of actions.";


    int client_id = -1;
    const int num_agents = 1;
    const int base_port = 50000;
    for (int i = 0; i < num_agents; ++i) {
        if (fork() == 0) { // child process
            client_id = i;
            break;
        }
        // parent process will continue
    }

    if (client_id < 0) {
        LOG(INFO) << "server process id " << getpid();

        std::vector<std::thread> server_threads;
        for (int i = 0; i < num_agents; ++i) {
            int port = (base_port < 0) ? -1 : base_port + i;
            server_threads.emplace_back(
                [&](int port) {
                    auto g = std::make_shared<SimulatorServer>(name, port);
                    run_simulation(g);
                },
                port
            );
        }
        for (auto& t : server_threads) { t.join(); }
    } else {
        int port = base_port + client_id;
        LOG(INFO) << "client process id " << getpid();
        auto client = std::make_shared<SimulatorClient>(name, port);
        client->start();
    }

    return 0;
}
