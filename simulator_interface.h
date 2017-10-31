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

/***************************** SimulatorInterface *****************************/
class SimulatorInterface {
public:
    virtual void reset_game() = 0;

    virtual void start() {
        running_ = true;
    }

    virtual void stop() {
        running_ = false;
    }

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

protected:
    bool running_;
};

SimulatorInterface* create_simulator(
        const std::string& name, const int port_no = -1);

/***************************** SingleProcInterface ****************************/
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

private:
    // user cannot create SimulatorInterface
    SingleProcInterface(SimulatorPtr game, TeacherPtr teacher);
    SimulatorPtr game_;
    TeacherPtr teacher_;
};

class SimulatorServer : public SimulatorInterface,
                        public communication::CommServer {
    friend SimulatorInterface* create_simulator(
            const std::string& name, const int port_no);
public:
    bool establish_connection() override;
    
    void reset_game() override;

    void start() override;

    void stop() override;

    std::string game_over_string() override;

    int game_over() override;

    int get_num_actions() override;

    int get_lives() override;

    int64_t get_num_steps() override;

    // return [h, w, c]
    void get_screen_out_dimensions(
            size_t& height, size_t& width, size_t& channels) override;

    void show_screen(float reward) override;

    float take_actions(const StatePacket& actions, int act_rep) override;

    StatePacket get_state(const float reward) override;

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

class SimulatorClient : public communication::CommClient {
public:
    SimulatorClient(const std::string& name, const int port_no);

    ~SimulatorClient() {}

    void start();

    void stop();

    bool establish_connection() override;

private:
    void simulation_loop();

    void reset_game();

    void show_screen();

    void take_actions();

    void get_state();

    std::string name_;
    std::string host_;
    int port_;
    SimulatorPtr game_;
    TeacherPtr teacher_;
};

} // namespace simulator
