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

#include "minecraft_simulator.h"
#include "minecraft_simulator_base.h"

#include <chrono>
#include <exception>
#include <random>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <simulator_util.h>

DECLARE_bool(pause_screen);
DEFINE_string(minecraft_client_ip, "127.0.0.1", "IP Address of minecraft mod");
DEFINE_int32(minecraft_client_port, 10000, "pport of minecraft mod");
// TODO: configure ms_per_tick for minecraft using this flag
DEFINE_int32(ms_per_tick, 10, "ms per tick for minecraft");

namespace simulator { namespace mcw {

using std::string;
using std::unique_ptr;

MinecraftSimulator* MinecraftSimulator::create(const std::string& mission, const std::string& conf_file) {
    return MinecraftSimulatorBase::create(mission, conf_file);
}

MinecraftSimulatorBase* MinecraftSimulatorBase::create(
    const std::string& mission, const string& conf_file) {
    if (mission == "demo") {
        return new MinecraftSimulatorDemo(conf_file);
    }
    LOG(FATAL) << "Unrecognized mission!";
    return nullptr;
}

MinecraftSimulatorBase::MinecraftSimulatorBase(const std::string& conf_file) {
    string xml = util::read_file(conf_file);

    try {
        // TODO: handle commandline args for Malmo
        int argc = 1;
        const char* argv[] = {"mcworld", nullptr};
        agent_host_.parseArgs(argc, argv);
    } catch (const std::exception& e) {
        LOG(FATAL)<< "ERROR: " << e.what() << std::endl
                  << agent_host_.getUsage();
    }
    client_pool_.add(malmo::ClientInfo(
        FLAGS_minecraft_client_ip, FLAGS_minecraft_client_port));
    // action_names_ = {"movenorth 1", "movesouth 1", "movewest 1", "moveeast 1"};
    action_names_ = {"turn 1", "turn -1", "move 1"};

    mission_ = util::make_unique<malmo::MissionSpec>(xml, true);
}

void MinecraftSimulatorBase::reset_game() {
    SIMULATOR_TIMER(reset_game);
    VLOG(1) << "Starting name game";
    new_mission();
    malmo::MissionRecordSpec mission_record;
    int max_retries = 5;
    for (int retry = 0; retry < max_retries; ++retry) {
        try {
            agent_host_.startMission(
                *mission_, client_pool_, mission_record,
                /* role= */0,
                /* unique_experiment_id= */"");
            break;
        } catch (std::exception& e) {
            if (retry == max_retries - 1) {
                LOG(FATAL) << "Error starting mission: " << e.what();
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    quited_ = false;

    VLOG(1) << "Waiting for the mission to start";
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    while (true) {
        world_state_ = agent_host_.getWorldState();
        for (auto error : world_state_.errors) {
            LOG(WARNING) << "Error: " << error->text;
        }
        if (world_state_.has_mission_begun) {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    VLOG(1) << "Mission started";
    {
        SIMULATOR_TIMER(observe0);
        observe();
    }
    // only after the mission begins we make screens
    GameSimulator::reset_game();
}

int MinecraftSimulatorBase::game_over() {
    int code = GameSimulator::game_over();
    if (quited_ || !world_state_.is_mission_running) {
        code |= DEAD;
    }
    if (code & MAX_STEP) {
        agent_host_.sendCommand("quit");
        quited_ = true;
        VLOG(3) << "quit game because of max step";
    }
    return code;
}

float MinecraftSimulatorBase::observe() {
    // The goal of this function is to get valid observation.
    // AgentHost::getWorldState() does not always get a video frame
    // or observation. We need to make sure we get these for the
    // next perceive_and_get_next_action() call.
    // And in some tasks (typically discretemovement tasks), we also
    // should check whether the agent is at the desired new state inicated
    // by reached_desired_state()

    float current_r = 0;
    auto video_frame = world_state_.video_frames.empty() ? nullptr
            : world_state_.video_frames.back();
    auto obs = world_state_.observations.empty() ? nullptr
            : world_state_.observations.back();
    bool reached = false;
    while (world_state_.is_mission_running) {
        {
            SIMULATOR_TIMER(sleep_for_observation);
            std::this_thread::sleep_for(
                std::chrono::milliseconds(2 * FLAGS_ms_per_tick));
        }
        world_state_ = agent_host_.getWorldState();
        for (auto& error : world_state_.errors) {
            LOG(WARNING) << "Error: " << error->text;
        }
        for (auto& reward : world_state_.rewards) {
            current_r += reward->getValue();
        }
        if (!world_state_.video_frames.empty()) {
            video_frame = world_state_.video_frames.back();
        }
        if (!world_state_.observations.empty()
            && world_state_.observations.back()->text != "{}") {
            obs = world_state_.observations.back();
        }
        if (!world_state_.is_mission_running) {
            // After game ends, the video_frames is empty.
            // But we need a video_frame for get_screen(). So
            // we use the previous video frame
            CHECK(video_frame);
            world_state_.video_frames.push_back(video_frame);
            CHECK(obs);
            world_state_.observations.push_back(obs);
        }
        if (reached_desired_state()) {
            reached = true;
        }
        if (reached &&
            !world_state_.observations.empty()
            && world_state_.observations.back()->text != "{}"
            && !world_state_.video_frames.empty()) {
            break;
        }
    }
    if (!reached) {
        VLOG(3) << "not reached";
    }

    return current_r;
}

float MinecraftSimulatorBase::take_action(const StatePacket& actions) {
    // TODO: Currently only assume one agent
    // This function should actually be called in AgentSpecificSimulator instead
    CHECK_EQ(actions.size(), 1);
    int action_id = *(actions.get_buffer("action")->get_id());
    {
        SIMULATOR_TIMER(sendCommand);
        agent_host_.sendCommand(action_names_[action_id]);
    }
    update_state(action_id);
    {
        SIMULATOR_TIMER(observe);
        return observe();
    }
}

void MinecraftSimulatorBase::get_screen(StatePacket& screen) {
    CHECK(!world_state_.video_frames.empty());
    auto& frame = *world_state_.video_frames.back();
    #if 0
    // Use x,z,yaw as input
    std::vector<float> screen_vec(3);
    screen_vec[0] = frame.xPos - 0.5;
    screen_vec[1] = frame.yPos - 0.5;
    screen_vec[2] = frame.yaw / 90;
    #else
    // Use color image as input
    CHECK_EQ(frame.channels, 3);
    int width = frame.width;
    int height = frame.height;
    cv::Mat img(height, width, CV_8UC3);
    std::copy(frame.pixels.begin(), frame.pixels.end(),
              img.ptr<uchar>());
    GameFrame screen_vec(3 * height * width);
    for (int h = 0; h < height; ++h) {
        for (int w = 0; w < width; ++w) {
            for (size_t c = 0; c < 3; c ++) {
                // normalize the pixel vals to [0,1]
                int offset = c * height * width + h * width + w;
                screen_vec[offset] =
                        img.at<cv::Vec3b>(cv::Point(w,h)).val[c];
            }
        }
    }
    #endif
    screen = StatePacket();
    screen.add_buffer_value("screen", screen_vec);
}

void MinecraftSimulatorBase::define_state_specs(StatePacket& state) {
    state = StatePacket();
    state.add_key("reward");
    state.add_key("screen");
}

void MinecraftSimulatorBase::show_screen(float rewards) {
    auto& frame = *world_state_.video_frames.back();
    cv::Mat img(cv::Size(frame.height, frame.width), CV_8UC3,
                frame.pixels.data());
    cv::imshow("MCWorld Game", img);
    if (FLAGS_pause_screen) {
        // The screen will pause at every step waiting for keyboard
        cv::waitKey(-1);
    } else {
        // Default mode: the screen will display continuously
        cv::waitKey(100);
    }
}

}}  // namespace simulator::mcw
