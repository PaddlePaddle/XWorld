// MIT License

// Copyright (c) 2017 Baidu Inc. All rights reserved.

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
/////////////////////////////////////////////////////////////////////////////////

#include <simulator.h>
#include <teacher.h>
#include <data_packet.h>
#include <boost/python.hpp>
#include <boost/python/exception_translator.hpp>
#include <exception>
#include "games/simple_game/simple_game_simulator.h"
#include "games/xworld/xworld_simulator.h"
#include "games/arcade/arcade_simulator.h"
#include <iostream>
#include <gflags/gflags.h>

#ifdef PY_MALMO
#include "games/minecraft/minecraft_simulator.h"
using namespace simulator::mcw;
#endif

#ifdef PY_DEEPMIND_LAB
#include "games/deepmind_lab/deepmind_lab_simulator.h"
using namespace simulator::deepmind_lab_game;
#endif

using namespace simulator;
using namespace simulator::simple_game;
using namespace simulator::xwd;
using namespace simulator::arcade_game;
namespace py = boost::python;

DECLARE_bool(color);    // default false
DECLARE_int32(context); // default 1
DECLARE_bool(pause_screen); // default false

//// xworld
DECLARE_bool(task_groups_exclusive); // default true
DECLARE_string(task_mode); // default "one_channel"

#ifdef PY_MALMO
DECLARE_string(minecraft_client_ip);
DECLARE_int32(minecraft_client_port);
DECLARE_int32(ms_per_tick);
#endif

#ifdef PY_DEEPMIND_LAB
DECLARE_string(runfiles_path);
DECLARE_string(level_script);
#endif

struct PyException : std::exception {
    PyException(const std::string& msg) : msg_(msg) {}
    std::string error() const throw() {
        return msg_;
    }
    std::string msg_;
};

void error_translate(PyException const& e) {
    PyErr_SetString(PyExc_RuntimeError, e.error().c_str());
}

/** Extract a value by key from a boost::python dict
 *  If required is true, then this key must be contained in the dict provided by user
 *  Otherwise when the key is missing, it returns default_val
 **/
template <typename T>
T extract_py_dict_val(
    const py::dict& args, const std::string& key, bool required, T default_val) {
    T ret = default_val;
    if (required) {
        if (!args.has_key(key)) {
            throw PyException("Key '" + key + "' is required but not provided.");
        }
    }
    if (args.has_key(key)) {
        ret = py::extract<T>(args[key]);
    }
    return ret;
}

/**
 * A wrapper for exposing various CPP simulators to Python
 * We use SimulatorPtr as a class member and create simulators in
 * the factory mode.
 * In this way, we only need to expose a minimum number of functions in
 * GameSimulator
 **/
class SimulatorInterface {
  public:
    // args contains additional options for creating the simulator
    // The complete list of args:
    // SimpleGame      -- "array_size" -> int
    // XWorldSimulator -- ""
    static SimulatorInterface* create_simulator(
        const std::string& name,
        const py::dict& args);

    // print help info, showing all the games
    static void help();

    void reset_game();

    std::string game_over();

    int get_num_actions();

    int get_lives();

    void show_screen();

    // return [h, w, c]
    py::list get_screen_out_dimensions();

    float take_actions(const py::dict& actions, int act_rep);

    float take_action(const py::dict& actions);

    py::dict get_state();

    int get_num_steps();

    void print_teacher_total_possible_sentences();

    void teacher_report_task_performance();

  private:
    // user cannot create SimulatorInterface
    SimulatorInterface(SimulatorPtr game, TeacherPtr teacher);
    SimulatorPtr game_;
    TeacherPtr teacher_;
    float reward_;  // current accumulated reward (non-discounted)
};

SimulatorInterface::SimulatorInterface(SimulatorPtr game, TeacherPtr teacher)
        : game_(game), teacher_(teacher) {
}

SimulatorInterface* SimulatorInterface::create_simulator(const std::string& name,
                                                         const py::dict& args) {
    TeacherPtr teacher = nullptr;
    SimulatorPtr game = nullptr;
    FLAGS_pause_screen = extract_py_dict_val(args, "pause_screen", false, false);
    if (name == "simple_game") {
        auto size = extract_py_dict_val(args, "array_size", true, 0);
        game = std::make_shared<SimpleGame>(size);
    } else if (name == "xworld") {
        FLAGS_color = true;
        std::string conf_path = extract_py_dict_val(args, "conf_path", true, "");
        auto curriculum_learning = extract_py_dict_val(args, "curriculum", false, 0);
        std::string task_mode = extract_py_dict_val(args, "task_mode", false, "one_channel");
        FLAGS_task_mode = task_mode;
        FLAGS_task_groups_exclusive = extract_py_dict_val(args, "tg_exclusive", false, true);
        FLAGS_context = extract_py_dict_val(args, "context", false, 1);

        if (task_mode == "arxiv_lang_acquisition") {
            FLAGS_task_groups_exclusive = false;
        }

        auto xwd = std::make_shared<XWorldSimulator>(
            true/*print*/, conf_path, curriculum_learning, task_mode);

        int agent_id = xwd->add_agent("robot0");
        game = std::make_shared<AgentSpecificSimulator>(xwd, agent_id);
        teacher = std::make_shared<Teacher>(conf_path, xwd, false/*print*/);
    } else if (name == "atari") {
        std::string ale_rom = extract_py_dict_val(args, "ale_rom", true, "");
        FLAGS_context = extract_py_dict_val(args, "context", false, 4);
        game.reset(ArcadeGame::create(ale_rom));
    }
    #ifdef PY_MALMO
    else if (name == "minecraft") {
        FLAGS_color = true;
        FLAGS_context = extract_py_dict_val(args, "context", false, 1);
        std::string mission = extract_py_dict_val(args, "mission", true, "");
        std::string conf_file = extract_py_dict_val(args, "conf_path", true, "");
        FLAGS_minecraft_client_ip = extract_py_dict_val(args, "client_ip", false, "127.0.0.1");
        FLAGS_minecraft_client_port = extract_py_dict_val(args, "client_port", false, 10000);
        FLAGS_ms_per_tick = extract_py_dict_val(args, "ms_per_tick", false, 10);
        auto mcw = std::shared_ptr<MinecraftSimulator>(
            MinecraftSimulator::create(mission, conf_file));
        int agent_id = mcw->add_agent("robot0");
        game = std::make_shared<AgentSpecificSimulator>(mcw, agent_id);
    }
    #endif
    #ifdef PY_DEEPMIND_LAB
    else if (name == "deepmind_lab") {
        FLAGS_color = true;
        FLAGS_context = extract_py_dict_val(args, "context", false, 1);
        FLAGS_runfiles_path = extract_py_dict_val(args, "runfiles_path", true, "");
        FLAGS_level_script = extract_py_dict_val(args, "level_script", true, "");
        game.reset(DeepmindLabSimulatorBase::create());
    }
    #endif
    else {
        throw PyException("Unrecognized game type: " + name);
    }
    auto g = new SimulatorInterface(game, teacher);
    g->reset_game();
    return g;
}

void SimulatorInterface::help() {
    std::cout <<
            "             actions                    state                       create args\n"
            "simple_game: {action:int}               {screen:list}               {array_size:int:0}\n"
            "xworld     : {action:int, pred_sentence:str} {screen:list, sentence:str, event:str, task:str} {conf_path:str:'', curriculum:int:0, task_mode:str:'one_channel', tg_exclusive:bool:True}\n"
            "atari      : {action:int}               {screen:list}               {context:int:4, ale_rom:str:''}"
            "minecraft  : {action:int}               {screen:list}               {mission:str:'', conf_path:str:'', client_ip:str:'127.0.0.1', client_port:int:10000, ms_per_tick:int:10, context:int:1}"
            "deepmind_lab: {action:int}              {screen:list}               {context:int:1, runfiles_path:str:'', level_script:str:''}";
}

void SimulatorInterface::reset_game() {
    reward_ = 0;
    game_->reset_game();
    if (teacher_) {
        teacher_->reset_after_game_reset();
        teacher_->teach();
    }
}

std::string SimulatorInterface::game_over() {
    int code = game_->game_over();
    if (code == 0) {
        return "alive";
    }
    std::string code_str = "";
    if (code & simulator::MAX_STEP) {
        code_str += "max_step|";
    }
    if (code & simulator::DEAD) {
        code_str += "dead|";
    }
    if (code & simulator::SUCCESS) {
        code_str += "success|";
    }
    if (code & simulator::LOST_LIFE) {
        code_str += "lost_life|";
    }
    CHECK(!code_str.empty());
    return code_str.substr(0, code_str.length() - 1);
}

int SimulatorInterface::get_num_actions() {
    return game_->get_num_actions();
}

int SimulatorInterface::get_lives() {
    return game_->get_lives();
}

void SimulatorInterface::show_screen() {
    game_->show_screen(reward_);
}

// Convert a Python dict of actions to StatePacket so that
// game_ can take an action
void convert_py_act_to_state_packet(const py::dict& actions,
                                    StatePacket& act) {
    // A default action id is filled in
    // When there is "sentence" key in actions, this will be ignored
    std::vector<int> action(1, 0);
    act.add_buffer_id("action", action);
    // convert py::dict to StatePacket
    py::list keys = actions.keys();
    CHECK_GT(py::len(keys), 0) << "You can't take an empty action";
    for (int i = 0; i < py::len(keys); i ++) {
        std::string k = py::extract<std::string>(keys[i]);
        if (k == "action") {
            action[0] = py::extract<int>(actions[keys[i]]);
            act.get_buffer("action")->set_id(action.begin(), action.end());
        } else if (k == "pred_sentence") {
            act.add_buffer_str("pred_sentence",
                               py::extract<std::string>(actions[keys[i]]));
        } else {
            throw PyException("Unrecognized key '"  + k + "' for the actions");
        }
    }
}

float SimulatorInterface::take_actions(const py::dict& actions, int act_rep) {
    StatePacket act;
    convert_py_act_to_state_packet(actions, act);
    float r = 0;
    r += game_->take_actions(act, act_rep);
    if (teacher_) {
        teacher_->teach(); // teacher reacts to agent's action and evaluate the reward
        r += teacher_->give_reward();
    }
    reward_ += r; // accumulate for record
    return r;
}

float SimulatorInterface::take_action(const py::dict& actions) {
    return take_actions(actions, 1);
}

py::dict SimulatorInterface::get_state() {
    StatePacket state;
    py::dict d;
    game_->get_state_data(0, state);
    auto keys = state.get_keys();
    for (const auto& k : keys) {
        // user will get reward outside this function from take_action()
        if (k == "reward") {
            continue;
        }
        if (k == "sentence") {
            auto sent = *(state.get_buffer(k)->get_str());
            d[k] = sent;
        } else { // all the rest are vectors of floats
            auto buffer = state.get_buffer(k);
            std::vector<float> vec(buffer->get_value_size());
            state.get_buffer(k)->copy_value(vec.begin(), vec.end());
            py::list l;
            for (auto x : vec) {
                l.append(x);
            }
            d[k] = l;
        }
    }
    // extra info
    std::unordered_map<std::string, std::string> info;
    game_->get_extra_info(info);
    for (const auto& i : info) {
        d[i.first] = i.second;
    }
    return d;
}

int SimulatorInterface::get_num_steps() {
    return game_->get_num_steps();
}

py::list SimulatorInterface::get_screen_out_dimensions() {
    size_t height;
    size_t width;
    size_t channels;
    game_->get_screen_out_dimensions(height, width, channels);
    py::list dims;
    dims.append(height);
    dims.append(width);
    dims.append(channels);
    return dims;
}

void SimulatorInterface::print_teacher_total_possible_sentences() {
    if (teacher_) {
        teacher_->print_total_possible_sentences();
    } else {
        std::cout << "This game does not have a teacher." << std::endl;
    }
}

void SimulatorInterface::teacher_report_task_performance() {
    if (teacher_) {
        teacher_->report_task_performance();
    } else {
        std::cout << "This game does not have a teacher." << std::endl;
    }
}

void help() {
    std::cout <<
            "Games are created by calling Simulator.create()\n"
            "Example:\n"
            "  from py_simulator import Simulator\n"
            "  simple_game = Simulator.create(\"simple_game\", {\"array_size\":6})\n"
            "  simple_game.show_screen()\n"
            "For more help on how to create games, call Simulator.help()\n";
}

BOOST_PYTHON_MODULE(py_simulator) {
    py::register_exception_translator<PyException>(&error_translate);
    py::def("help", help);
    py::class_<SimulatorInterface, boost::noncopyable>("Simulator", py::no_init)
            .def("create", &SimulatorInterface::create_simulator,
                 py::return_value_policy<py::manage_new_object>())
            .staticmethod("create")
            .def("help", &SimulatorInterface::help)
            .staticmethod("help")
            .def("reset_game", &SimulatorInterface::reset_game)
            .def("game_over", &SimulatorInterface::game_over)
            .def("get_num_actions", &SimulatorInterface::get_num_actions)
            .def("get_lives", &SimulatorInterface::get_lives)
            .def("show_screen", &SimulatorInterface::show_screen)
            // intentionally bind take_actions to two functions so that it has optional args
            .def("take_actions", &SimulatorInterface::take_actions)
            .def("take_actions", &SimulatorInterface::take_action)
            .def("get_state", &SimulatorInterface::get_state)
            .def("get_num_steps", &SimulatorInterface::get_num_steps)
            .def("print_total_possible_sentences",
                 &SimulatorInterface::print_teacher_total_possible_sentences)
            .def("teacher_report_task_performance",
                 &SimulatorInterface::teacher_report_task_performance);
}
