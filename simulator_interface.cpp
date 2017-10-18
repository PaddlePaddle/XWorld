#include <time.h>
#include <unistd.h>
#include "memory_util.h"
#include "simulator_interface.h"

DECLARE_bool(task_groups_exclusive);

namespace simulator {

using namespace simple_game;
using namespace simple_race;
using namespace xwd;
#ifdef ATARI
using namespace simulator::arcade_game;
#endif

using boost::asio::ip::tcp;
using util::BinaryBuffer;

typedef std::shared_ptr<BinaryBuffer> BinaryBufferPtr;

/***************************** SimulatorInterface *****************************/
SimulatorInterface* create_simulator(
        const std::string& name, const int port_no) {

    SimulatorInterface* g = NULL;

    if (port_no >= 0) {
        g = new SimulatorServer(name, port_no);
    } else {
        TeacherPtr teacher = nullptr;
        SimulatorPtr game = nullptr;
        if (name == "simple_game") {
            game = std::make_shared<SimpleGame>();
        } else if (name == "simple_race") {
            game = std::make_shared<SimpleRaceGame>();
        } else if (name == "xworld") {
            FLAGS_color = true;

            if (FLAGS_task_mode == "arxiv_lang_acquisition") {
                FLAGS_task_groups_exclusive = false;
            }

            auto xwd = std::make_shared<XWorldSimulator>(true /*print*/);

            int agent_id = xwd->add_agent();
            game = std::make_shared<AgentSpecificSimulator>(xwd, agent_id);
            teacher = std::make_shared<Teacher>(
                    xwd->conf_file(), xwd, false /*print*/);
        } 
#ifdef ATARI
        else if (name == "atari") {
            game.reset(ArcadeGame::create());
        }
#endif
        else {
            LOG(FATAL) << "Unrecognized game type: " << name;
        }
        g = new SingleProcInterface(game, teacher);
    }

    return g;
}

/***************************** SingleProcInterface ****************************/
SingleProcInterface::SingleProcInterface(SimulatorPtr game, TeacherPtr teacher)
    : game_(game), teacher_(teacher) {}

void SingleProcInterface::reset_game() {
    game_->reset_game();
    if (teacher_) {
        teacher_->reset_after_game_reset();
        teacher_->teach();
    }
}

std::string SingleProcInterface::game_over_string() {
    return GameSimulator::decode_game_over_code(game_->game_over());
}

int SingleProcInterface::game_over() {
    return game_->game_over();
}

int SingleProcInterface::get_num_actions() { return game_->get_num_actions(); }

int SingleProcInterface::get_lives() { return game_->get_lives(); }

int64_t SingleProcInterface::get_num_steps() { return game_->get_num_steps(); }

void SingleProcInterface::get_screen_out_dimensions(
        size_t& height, size_t& width, size_t& channels) {
    game_->get_screen_out_dimensions(height, width, channels);
}

void SingleProcInterface::show_screen(float reward) {
    game_->show_screen(reward);
}

float SingleProcInterface::take_actions(const StatePacket& actions, int act_rep) {
    float r = 0;
    r += game_->take_actions(actions, act_rep);
    if (teacher_) {
        teacher_->teach();  // teacher reacts to agent's action and evaluate the
                            // reward
        r += teacher_->give_reward();
    }

    return r;
}

StatePacket SingleProcInterface::get_state(const float reward) {
    StatePacket state;
    game_->get_state_data(reward, state);

    return state;
}

/******************************* SimulatorServer ******************************/
SimulatorServer::SimulatorServer(const std::string& name, const int port_no) : 
        CommServer(port_no),
        name_(name),
        num_actions_(-1),
        num_steps_(-1),
        game_over_code_(0),
        lives_(0),
        height_(0), width_(0), channels_(0) {
}

void SimulatorServer::reset_game() {
    send_cmd_and_get_reply("reset", NULL);
    msg_body_->read(num_actions_);
    msg_body_->read(game_over_code_);
    msg_body_->read(lives_);
    msg_body_->read(height_);
    msg_body_->read(width_);
    msg_body_->read(channels_);

    num_steps_ = 0;
}

void SimulatorServer::start() {
    establish_connection();
    SimulatorInterface::start();
}

void SimulatorServer::stop() { 
    LOG(INFO) << "[server " << name_ << "] stopping";
    msg_body_->clear();
    msg_body_->append(std::string("stop"));
    send_msg();

    close_connection();
    SimulatorInterface::stop();
    LOG(INFO) << "[server " << name_ << "] stopped";
}

bool SimulatorServer::establish_connection() {
    if (CommServer::establish_connection()) {
        // simple identification procedure
        std::string greeting;
        receive_msg();
        msg_body_->read(greeting);
        CHECK_EQ(greeting, name_);
        if (greeting == name_) {
            msg_body_->clear();
            msg_body_->append(std::string("accepted"));
            send_msg();
        } else {
            LOG(INFO) << "[server " << name_ << "] " 
                      << "connection failed: name not matched";
            return false;
        }
    } else {
        LOG(INFO) << "[server " << name_ << "] " 
                  << "cannot establish connection";
        return false;
    }
    LOG(INFO) << "[server " << name_ << "] connected to client";
    return true;
}

std::string SimulatorServer::game_over_string() {
    return GameSimulator::decode_game_over_code(game_over_code_);
}

int SimulatorServer::game_over() {
    // game_over_code_ is returned from take_actions
    return game_over_code_;
}

int SimulatorServer::get_num_actions() {
    // num_actions_ is returned from reset_game
    return num_actions_;
}

int SimulatorServer::get_lives() {
    return lives_;
}

int64_t SimulatorServer::get_num_steps() {
    // num_steps_ is maintained by server itself and double checked with the one
    // returned from take_actions
    return num_steps_;
}

// return [h, w, c]
void SimulatorServer::get_screen_out_dimensions (
        size_t& height, size_t& width, size_t& channels) {
    // height_, width_ and channels_ are returned from reset_game
    height = height_;
    width = width_;
    channels = channels_;
}

void SimulatorServer::show_screen(float reward) {
    send_cmd_and_get_reply("show_screen", NULL, reward);
}

float SimulatorServer::take_actions(const StatePacket& actions, int act_rep) {
    float r;
    decltype(num_steps_) num_steps_from_client;
    
    send_cmd_and_get_reply("take_actions", &actions, act_rep);

    num_steps_ ++;
    msg_body_->read(r);
    // num_steps_ from server should be equal to num_steps_ from client
    msg_body_->read(num_steps_from_client);
    msg_body_->read(game_over_code_);
    msg_body_->read(lives_);

    CHECK_EQ(num_steps_, num_steps_from_client);

    return r;
}

StatePacket SimulatorServer::get_state(const float reward) {
    send_cmd_and_get_reply("get_state", NULL, reward);
    StatePacket state;
    state.decode(*msg_body_);
    return state;
}

/******************************* SimulatorServer ******************************/
SimulatorClient::SimulatorClient(const std::string& name, const int port_no) : 
        CommClient("localhost", port_no),
        name_(name),
        game_(nullptr), teacher_(nullptr) {
    if (name == "simple_game") {
        game_ = std::make_shared<SimpleGame>();
    } else if (name == "simple_race") {
        game_ = std::make_shared<SimpleRaceGame>();
    } else if (name == "xworld") {
        FLAGS_color = true;
        if (FLAGS_task_mode == "arxiv_lang_acquisition") {
            FLAGS_task_groups_exclusive = false;
        }
        auto xwd = std::make_shared<XWorldSimulator>(true /*print*/);
        int agent_id = xwd->add_agent();
        game_ = std::make_shared<AgentSpecificSimulator>(xwd, agent_id);
        teacher_ = std::make_shared<Teacher>(
                xwd->conf_file(), xwd, false /*print*/);
    }
#ifdef ATARI
    else if (name == "atari") {
        game_.reset(ArcadeGame::create());
    }
#endif
    else {
        LOG(FATAL) << "Unrecognized game type: " << name;
    }
}

void SimulatorClient::start() {
    if (!establish_connection()) {
        LOG(INFO) << "[client " << name_ << "] "
                  << "stops due to connection error";
        return;
    }
    simulation_loop();
}

void SimulatorClient::stop() {
    LOG(INFO) << "[client " << name_ << "] stopping";
    close_connection();
    LOG(INFO) << "[client " << name_ << "] stopped";
}

bool SimulatorClient::establish_connection() {
    if (CommClient::establish_connection()) {
        // simple identification procedure
        msg_body_->clear();
        msg_body_->append(name_);
        send_msg();
        std::string reply;
        receive_msg();
        msg_body_->read(reply);
        if (reply != "accepted") {
            LOG(INFO) << "[client " << name_ << "] " 
                      << "connection failed: name not matched";
            return false;
        }
    } else {
        LOG(INFO) << "[client " << name_ << "] "
                  << "connection failed: cannot connect to server";
        return false;
    }
    LOG(INFO) << "[client " << name_ << "] connected to server";
    return true;
}

void SimulatorClient::simulation_loop() {
    std::string cmd = "";
    double time_comm = 0;
    clock_t t;
    while (true) {
        t = clock();
        receive_msg();
        LOG(INFO) << cmd;
        time_comm += clock() - t;
        msg_body_->read(cmd);
        if (cmd == "reset") {
            reset_game();
        } else if (cmd == "show_screen") {
            show_screen();
        } else if (cmd == "take_actions") {
            take_actions();
        } else if (cmd == "get_state") {
            get_state();
        } else if (cmd == "stop") {
            break;
        }

        t = clock();
        send_msg();
        time_comm += clock() - t;
    }
    LOG(INFO) << "[client " << name_ << "] "
              <<  "time comm: " << time_comm / CLOCKS_PER_SEC;
    stop();
}

void SimulatorClient::reset_game() {
    game_->reset_game();

    size_t height;
    size_t width;
    size_t channels;
    game_->get_screen_out_dimensions(height, width, channels);

    msg_body_->clear();
    msg_body_->append(std::string("reset"));
    msg_body_->append(game_->get_num_actions());
    msg_body_->append(game_->game_over());
    msg_body_->append(game_->get_lives());
    msg_body_->append(height);
    msg_body_->append(width);
    msg_body_->append(channels);
}

void SimulatorClient::show_screen() {
    float reward;
    msg_body_->read(reward);
    game_->show_screen(reward);    
    msg_body_->clear();
    msg_body_->append(std::string("show_screen"));
}

void SimulatorClient::take_actions() {
    int act_rep;
    StatePacket actions;
    msg_body_->read(act_rep);
    actions.decode(*msg_body_);

    float reward = game_->take_actions(actions, act_rep);
    if (teacher_) {
        CHECK(teacher_->teach());
        reward += teacher_->give_reward();
    }

    msg_body_->clear();
    msg_body_->append(std::string("take_actions"));
    msg_body_->append(reward);
    msg_body_->append(game_->get_num_steps());
    msg_body_->append(game_->game_over());
    msg_body_->append(game_->get_lives());
}

void SimulatorClient::get_state() {
    float reward;
    msg_body_->read(reward);
    StatePacket state;
    game_->get_state_data(reward, state);
    
    msg_body_->clear();
    msg_body_->append(std::string("get_state"));
    state.encode(*msg_body_);
}

} // namespace simulator

