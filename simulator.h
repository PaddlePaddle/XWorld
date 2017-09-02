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
#include <ctime>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include "data_packet.h"
#include "simulator_util.h"

#ifndef GFLAGS_GFLAGS_H_
namespace gflags = google;
#endif

DECLARE_bool(lock_step);

namespace simulator {

// all possilbe game status
enum GameOverCode {
    ALIVE = 0,          // normal state
    MAX_STEP = 1 << 0,  // reach maximum number of steps
    DEAD = 1 << 1,      // lose all lives
    SUCCESS = 1 << 2,   // succeed the game
    LOST_LIFE = 1 << 3  // lose one life
};

typedef std::vector<uint8_t> GameFrame;

class GameSimulator {
  public:
    GameSimulator();

    virtual ~GameSimulator() {}

    /**
     * @brief Reset game
     *
     * This function is called everytime a new game begins
     */
    virtual void reset_game();

    /**
     * @brief return the game status
     */
    virtual int game_over() {
        if (max_steps_ > 0 && num_steps_ >= max_steps_) {
            return MAX_STEP;
        } else {
            return ALIVE;
        }
    }

    /**
     * @brief get the number of actions the game allowed
     */
    virtual int get_num_actions() = 0;

    /**
     * @brief get the number of lives
     */
    virtual int get_lives() = 0;

    /**
     * @brief visualize the current screen
     */
    virtual void show_screen(float reward) = 0;

    /**
     * @brief take a single-step action represent by `actions` and return the
     * reward
     *
     * @param[in] actions    action to take
     * @return reward by taking the action
     */
    virtual float take_action(const StatePacket& actions) = 0;

    /**
     * @brief get the current screen
     * Can't use GameFrame because some game has floating-number screen
     */
    virtual void get_screen(StatePacket& screen) = 0;

    /**
     * @briefn pack all info into a StatePacket.
     *
     * Derived simulators should override `define_state_specss`
     * to get the state structure they need, and fill in the values of all info
     * except "reward" and "screen", which are hanleded in this function.
     *
     * Consecutive calls of this function with no calls of `take_actions' in
     * between should return the same state.
     *
     * @param[in]  reward   the reward received when entering current state
     * @param[out] state    state with all info
     */
    void get_state_data(float reward, StatePacket& state) {
        define_state_specs(state);
        // All simulators have "reward" and "screen" in their state structures,
        // so we put common codes here.
        fill_in_reward_and_screen(reward, state);
    }

    /**
     * @brief Specify what types of information are needed in the state.
     *
     * TODO: Ideally, this function should not be public. But in
     * AgentSpecifiSimulator::define_state_specs, we need to access
     * this function from outside, so we make it public accessible.
     */
    virtual void define_state_specs(StatePacket& state) = 0;

    /**
     * @brief Extra information for debugging or displaying (at each time step)
     *
     * Derived classes should override this function to provide game-specific
     * extra info. Usually this info shouldn't be used by the agent for
     * learning. As a result, all values in the map are designed as strings.
     */
    virtual void get_extra_info(
        std::unordered_map<std::string, std::string>& info) {
        info.clear();
    }

    /**
     * @brief repeatedly take `actions` for `actrep` times, and return the
     * cumulative reward after taking actrep actions.
     *
     * screens_ will be updated after taking actions.
     */
    float take_actions(const StatePacket& actions, int actrep);

    int64_t get_num_steps() { return num_steps_; }

    virtual std::string last_action() { return last_action_; }

    bool last_action_success() { return last_action_success_; }

    static const int N_BUFFERS =
        2;  // "reward" and "screen" are two basic buffers

    /**
     * @brief get the image dimensions used by CNN
     */
    virtual void get_screen_out_dimensions(size_t& height,
                                           size_t& width,
                                           size_t& channels) = 0;

    static std::string decode_game_over_code(int code);

  protected:
    int64_t max_steps_;
    int64_t num_steps_;  // number of steps since the beginning of the game

    std::string last_action_;  // store the last action for the agent
    bool last_action_success_; // whether the last action was successful or not

    StatePacket screens_;  // maintain the most recent FLAGS_context screens

    // global mutex to serialize calls of cv::imshow, which is not thread-safe.
    static std::mutex s_display_mutex_;

    /**
     * @brief return the id of the thread that calls `show_screen` as the screen
     * name.
     *
     * In case of multiple concurrent games, we use thread ids to identify
     * imshow windows.
     */
    std::string get_screen_name();

    /**
     * @brief clear screens_ and initialize it using current screen.
     */
    void init_screen();

    /**
     * @brief allocate space for screens_
     */
    void init_context_screens(bool is_uint8);

    /**
     * @brief update screens_ by adding current screen at the beginning and
     * remove the oldest screen
     */
    void make_context_screens();

    /**
     * @brief Shift the context by one screen of size screen_sz to right.
     *
     * This function is designed to support screens of both types uint8_t and
     * float.
     *
     * @param[in]  cur_buf
     * @param[out] context_buf
     * @param[in]  n_contexts
     * @param[in]  screen_sz
     */
    template <typename T>
    void shift_context(BufferPtr cur_buf,
                       BufferPtr context_buf,
                       int n_contexts,
                       size_t screen_sz);

    /**
     * @brief Fill the content of "reward" and "screen" of `state`.
     *
     * @param[in]  reward    reward
     * @param[out] state     state to fill
     */
    void fill_in_reward_and_screen(float reward, StatePacket& state);
};

typedef std::shared_ptr<GameSimulator> SimulatorPtr;

/*
 * class that defines the interface supporting multi-agent simulator
 */

class GameSimulatorMulti : public GameSimulator {
  public:
    GameSimulatorMulti() : active_agent_id_(0) {}

    virtual ~GameSimulatorMulti() {}

    virtual int add_agent(std::string name) {
        agents_[name] = agents_.size() - 1;
        return agents_.size() - 1;
    }

    virtual void set_active_agent_id(int agent_id) {
        active_agent_id_ = agent_id;
    }

    // this function is called to return max_steps to AgentSpecificSimulator
    int64_t get_max_steps() { return max_steps_; }

  protected:
    int active_agent_id_;
    std::unordered_map<std::string, int> agents_;
};

typedef std::shared_ptr<GameSimulatorMulti> SimulatorMultiPtr;

class Entity;

// A buffer that stores the communication between agent and teacher,
// coordinated by the teaching environment
// It is SHARED by all the tasks; a teacher only has one buffer
struct TeachingEnvBuffer {
    ///////// teacher's buffer /////////
    std::string teacher_sent;
    std::string teacher_sent_type;    // for display only; not for agent
    std::string teacher_sent_answer;  // only useful for supervised task
    double reward;
    std::string event;  // stores an event's name during the session
                        // e.g., "correct_goal", "wrong_goal", "hit_wall"
                        // can be used by the environment to decide game_over()
    ///////// agent's buffer ////////
    std::string agent_sent;
    int agent_action;

    void clear_teacher_env_buffer() {
        teacher_sent = "";
        teacher_sent_type = "";
        teacher_sent_answer = "";
        reward = 0;
        event = "";
    }
    void clear_agent_env_buffer() {
        agent_sent = "";
        agent_action = -1;
    }
};

// TeachingEnvironment only supports single sentence
class TeachingEnvironment {
  public:
    TeachingEnvironment(int curriculum_learning = 0)
        : beginning_(true),
          num_games_since_simulation_(0),
          curriculum_learning_(curriculum_learning) {}

    virtual ~TeachingEnvironment() {}

    bool can_record_teacher_sent_in_buffer() const {
        return buffer_.teacher_sent.empty();
    }

    void record_teacher_sent_in_buffer(const std::string& sentence) {
        buffer_.teacher_sent = sentence;
    }

    const std::string& get_teacher_sent_from_buffer() const {
        return buffer_.teacher_sent;
    }

    void record_teacher_sent_type_in_buffer(const std::string& type) {
        buffer_.teacher_sent_type = type;
    }

    const std::string& get_teacher_sent_type_from_buffer() const {
        return buffer_.teacher_sent_type;
    }

    void record_teacher_sent_answer_in_buffer(const std::string& answer) {
        buffer_.teacher_sent_answer = answer;
    }

    const std::string& get_teacher_sent_answer_from_buffer() const {
        return buffer_.teacher_sent_answer;
    }

    void add_teacher_reward(double reward) { buffer_.reward += reward; }

    double get_teacher_reward() const { return buffer_.reward; }

    void record_event_in_buffer(const std::string& event) {
        buffer_.event = event;
    }

    const std::string& get_event_from_buffer() const { return buffer_.event; }

    void record_agent_sent_in_buffer(const std::string& sentence) {
        buffer_.agent_sent = sentence;
    }

    const std::string& get_agent_sent_from_buffer() const {
        return buffer_.agent_sent;
    }

    void record_agent_action_in_buffer(int action) {
        buffer_.agent_action = action;
    }

    int get_agent_action_from_buffer() const { return buffer_.agent_action; }

    // This function can be used to decide wheter some property of
    // an entity is valid for the game
    virtual bool entity_valid(const Entity& e) = 0;

    // If the entity's color is defined in the game
    virtual bool color_defined(std::string c) = 0;

    // get all the entities in the current game
    virtual void get_all_entities(std::vector<Entity>& entities) = 0;

    // get all possible objects that can appear in the game
    // this function is used for computing the total number of sentences by
    // teacher
    virtual void get_all_possible_objects(std::vector<Entity>& objects) = 0;

    // The linking node between the teacher and the agent
    virtual void apply_teacher_actions() = 0;

    // get the world size
    virtual size_t world_size() = 0;

    virtual void reset_game() {
        beginning_ = true;
        num_games_since_simulation_++;
    }

    virtual float take_action(const StatePacket& actions) {
        beginning_ = false;
        // clear teacher's buffer after agent takes an action
        clear_teacher_env_buffer();
        return 0;
    }

    void clear_agent_env_buffer() { buffer_.clear_agent_env_buffer(); }

    void clear_teacher_env_buffer() { buffer_.clear_teacher_env_buffer(); }

    bool beginning_of_the_game() { return beginning_; }

    int num_games_since_simulation() { return num_games_since_simulation_; }

    int curriculum_learning() { return curriculum_learning_; }

  protected:
    bool
        beginning_;  // whether any agent has taken action since the game starts
    int num_games_since_simulation_;  // how many games have been played
    int curriculum_learning_;         // how many games for curriculum learning
    TeachingEnvBuffer
        buffer_;  // a buffer to store exchanging information between
                  // the teacher and the agent
};

typedef std::shared_ptr<TeachingEnvironment> TeachingEnvPtr;

/**
   class that implements a agent-specific interface from a multi-agent simulator
   by maintaining the agent_id and SimulatorPtr
*/

class AgentSpecificSimulator : public GameSimulator {
  public:
    AgentSpecificSimulator(SimulatorMultiPtr simulator_ptr, int agent_id = 0);

    void reset_game() override;

    int game_over() override;

    int get_num_actions() override;

    int get_lives() override;

    void show_screen(float reward) override;

    std::string last_action() override;

    float take_action(const StatePacket& actions) override;

    void get_screen(StatePacket& screen) override;

    void get_extra_info(
        std::unordered_map<std::string, std::string>& info) override;

    void get_screen_out_dimensions(size_t& height,
                                   size_t& width,
                                   size_t& channels) override;

    void define_state_specs(StatePacket& state);

  private:
    void activate_my_agent() { simulator_ptr_->set_active_agent_id(agent_id_); }

    SimulatorMultiPtr simulator_ptr_;
    const int agent_id_;
};

}  // namespace simulator
