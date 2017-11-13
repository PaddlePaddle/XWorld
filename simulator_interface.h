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

#include <gflags/gflags.h>
#include <iostream>

#include "games/simple_game/simple_game_simulator.h"
#include "games/simple_race/simple_race_simulator.h"
#include "games/xworld/xworld_simulator.h"
#ifdef ATARI
#include "games/arcade/arcade_simulator.h"
#endif
#include "simulator.h"
#include "simulator_communication.h"
#include "teacher.h"

namespace simulator {

/**
 * Interface for users to simulate games.
 *
 * It wraps `GameSimulator` and `Teacher`, and hide the simulation details.
 * Holders of a pointer to this interface do not need to know if the simulation
 * happens in the same process or in another process.
 */
class SimulatorInterface {
public:
    virtual void start() {
        running_ = true;
    }

    virtual void stop() {
        running_ = false;
    }

    virtual void reset_game() = 0;

    virtual std::string game_over_string() = 0;

    virtual int game_over() = 0;

    virtual int get_num_actions() = 0;

    virtual int get_lives() = 0;

    virtual int64_t get_num_steps() = 0;

    // return [h, w, c]
    virtual void get_screen_out_dimensions(
            size_t& height, size_t& width, size_t& channels) = 0;

    virtual void show_screen(float reward) = 0;

    virtual float take_actions(const StatePacket& actions, int act_rep) = 0;

    float take_action(const StatePacket& actions) {
        return take_actions(actions, 1);
    }

    virtual StatePacket get_state(const float reward) = 0;

    virtual void teacher_report_task_performance() = 0;

protected:
    bool running_;
};

/**
 * Creator function for `SimulatorInterface`
 */
SimulatorInterface* create_simulator(
        const std::string& name, const int port_no = -1);

/**
 * This class handles the simulation where users and simulation are in the same
 * process.
 */
class SingleProcInterface : public SimulatorInterface {
    friend SimulatorInterface* create_simulator(
            const std::string& name, const int port_no);
public:
    void reset_game() override;

    std::string game_over_string() override;

    int game_over() override;

    int get_num_actions() override;

    int get_lives() override;

    int64_t get_num_steps() override;

    // return [h, w, c]
    void get_screen_out_dimensions (
            size_t& height, size_t& width, size_t& channels) override;

    void show_screen(float reward) override;

    float take_actions(const StatePacket& actions, int act_rep) override;

    StatePacket get_state(const float reward) override;

    void teacher_report_task_performance() override;

private:
    // user cannot create SimulatorInterface himself
    SingleProcInterface(SimulatorPtr game, TeacherPtr teacher);

    SimulatorPtr game_;
    TeacherPtr teacher_;
};

/**
 * Server-side simulation interface. It is only responsible for conveying the
 * simulation command (e.g., resetting game, taking actions) issued by users to
 * the client, another process where the actual simulation happens.
 *
 * For efficiency, we do not need to issue a remote function call for every
 * simulation command. We summarize which functions are executed on server side
 * and which are not:
 *
 * Functions executed on server side:
 * - reset_game
 * - show_screen
 * - take_actions (and hence take_action)
 * - get_state
 * - teacher_report_task_performance
 *
 * Functions executed on client:
 * - game_over_string
 * - game_over
 * - get_num_actions
 * - get_lives
 * - get_num_steps
 * - get_screen_out_dimensions
 */
class SimulatorServer : public SimulatorInterface,
                        public communication::CommServer {
    friend SimulatorInterface* create_simulator(
            const std::string& name, const int port_no);
public:
    void start() override;

    void stop() override;
    /**
     * Besides establishing TCP/IP connection, it also makes sure it connects
     * with the right client by checking the name.
     */
    bool establish_connection() override;
    /**
     * Request client to reset the game. Client will return:
     * 1. number of valid actions
     * 2. game over code
     * 3. number of lives at the beginning of game
     * 4. game dimensions
     */
    void reset_game() override;
    /**
     * Return the game over string.
     *
     * No client function call is issued. It uses the game over code obtained
     * from `reset_game` or `take_actions`.
     */
    std::string game_over_string() override;
    /**
     * Return the game over code.
     *
     * No client function call is issued. It uses the game over code obtained
     * from `reset_game` or `take_actions`.
     */

    int game_over() override;
    /**
     * Return the number of valid actions.
     *
     * No client function call is issued. It uses the number returned by
     * `reset_game`.
     */
    int get_num_actions() override;
    /**
     * Return the number of lives remaining.
     *
     * No client function call is issued. It uses the number returned by
     * `reset_game` or `take_actions`.
     */
    int get_lives() override;
    /**
     * Return the number of steps taken so far.
     *
     * No client function call is issued. This class uses its own counter.
     * However, it will check its version with the one returned by
     * `take_actions`.
     */
    int64_t get_num_steps() override;

    /**
     * Return [h, w, c].
     *
     * No client function call is issued. It uses the number obtained from
     * `reset_game`.
     */
    void get_screen_out_dimensions(
            size_t& height, size_t& width, size_t& channels) override;

    /**
     * Request client to show current screens in the game.
     */
    void show_screen(float reward) override;

    /**
     * Request client to take actions. Client will also return:
     * 1. number of steps taken so far
     * 2. game over code
     * 3. number of lives remaining
     */
    float take_actions(const StatePacket& actions, int act_rep) override;
    /**
     * Request client to return the current state of the game.
     */
    StatePacket get_state(const float reward) override;

    void teacher_report_task_performance() override;

private:
    SimulatorServer(const std::string& name, const int port_no);

    std::string name_;
    int num_actions_;
    int64_t num_steps_;
    int game_over_code_;
    int lives_;
    int height_;
    int width_;
    int channels_;
};

/**
 * Client-side simulation interface. It executes simulation commands from, and
 * returns outcomes back to, server.
 */
class SimulatorClient : public communication::CommClient {
public:
    SimulatorClient(const std::string& name, const int port_no);

    ~SimulatorClient() {}

    void start();

    void stop();

    /**
     * Attempt to connect server. Once connected, send its name to server for
     * identification.
     */
    bool establish_connection() override;

private:
    /**
     * A while loop that alternatively:
     * 1. receive simulation command from server
     * 2. execute the command
     * 3. return outcomes back to server
     */
    void simulation_loop();

    void reset_game();

    void show_screen();

    void take_actions();

    void get_state();

    void teacher_report_task_performance();

    std::string name_;
    std::string host_;
    int port_;
    SimulatorPtr game_;
    TeacherPtr teacher_;
};

} // namespace simulator
