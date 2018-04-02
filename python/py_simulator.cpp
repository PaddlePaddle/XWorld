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

#include <gflags/gflags.h>
#include <boost/python.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/python/exception_translator.hpp>
#include <exception>
#include <iostream>
#include "simulator_interface.h"
#include <unordered_map>

using namespace simulator;
namespace py = boost::python;

struct PyException : std::exception {
    PyException(const std::string& msg) : msg_(msg) {}
    std::string error() const throw() { return msg_; }
    std::string msg_;
};

void error_translate(PyException const& e) {
    PyErr_SetString(PyExc_RuntimeError, e.error().c_str());
}

/** Extract a value by key from a boost::python dict
 *  If required is true, then this key must be contained in the dict provided by
 *user
 *  Otherwise when the key is missing, it returns default_val
 **/
template <typename T>
T extract_py_dict_val(const py::dict& args,
                      const std::string& key,
                      bool required,
                      T default_val) {
    T ret = default_val;
    if (required) {
        if (!args.has_key(key)) {
            throw PyException("Key '" + key +
                              "' is required but not provided.");
        }
    }
    if (args.has_key(key)) {
        ret = py::extract<T>(args[key]);
    }
    return ret;
}


class PySimulatorInterface {
  public:
    // args contains options for creating the simulator
    // see https://github.com/PaddlePaddle/XWorld for instructions
    static PySimulatorInterface* create_simulator(const std::string& name,
                                                  const py::dict& args);

    void reset_game() { interface_.reset_game(); }

    std::string game_over() { return interface_.game_over_string(); }

    int get_num_actions() { return interface_.get_num_actions(); }

    int get_lives() { return interface_.get_lives(); }

    // return [h, w, c]
    py::list get_screen_out_dimensions();

    float take_actions(const py::dict& actions, int act_rep, bool show_screen);

    float take_action(const py::dict& actions, bool show_screen);

    py::dict get_state();

    int get_num_steps() { return interface_.get_num_steps(); }

  private:
    PySimulatorInterface(const std::string& name);
    // wrap a single-process simulator interface
    SimulatorInterface interface_;
};

///////////////////// pass flags in python dict to GFlags ///////////////////
///////////////////// simple game
void init_simple_game_gflags(const py::dict& args) {
    auto array_size = extract_py_dict_val(args, "array_size", true, 0);
    FLAGS_array_size = array_size;
}

///////////////////// pass flags in python dict to GFlags ///////////////////
///////////////////// simple race
void init_simple_race_gflags(const py::dict& args) {
    std::string track_type =
            extract_py_dict_val(args, "track_type", false, "straight");
    FLAGS_track_type = track_type;
    FLAGS_track_width =
            extract_py_dict_val(args, "track_width", true, 20.0f);
    FLAGS_track_length =
            extract_py_dict_val(args, "track_length", true, 100.0f);
    FLAGS_track_radius =
            extract_py_dict_val(args, "track_radius", true, 30.0f);
    FLAGS_race_full_manouver =
            extract_py_dict_val(args, "race_full_manouver", false, false);
    FLAGS_random = extract_py_dict_val(args, "random", false, false);
    std::string difficulty =
            extract_py_dict_val(args, "difficulty", false, "easy");
    FLAGS_difficulty = difficulty;
}

///////////////////// pass flags in python dict to GFlags ///////////////////
///////////////////// xworld
void init_xworld_gflags(const py::dict& args) {
    FLAGS_xwd_conf_path =
            extract_py_dict_val(args, "xwd_conf_path", true, "");
    FLAGS_curriculum = extract_py_dict_val(args, "curriculum", false, 0.0f);
    std::string task_mode =
            extract_py_dict_val(args, "task_mode", false, "one_channel");
    FLAGS_task_mode = task_mode;
    FLAGS_task_groups_exclusive =
            extract_py_dict_val(args, "task_groups_exclusive", false, true);
    FLAGS_context = extract_py_dict_val(args, "context", false, 1);
    FLAGS_visible_radius = extract_py_dict_val(args, "visible_radius", false, 0);
    FLAGS_color = extract_py_dict_val(args, "color", false, false);
}

///////////////////// pass flags in python dict to GFlags ///////////////////
///////////////////// xworld3d
void init_xworld3d_gflags(const py::dict& args) {
#ifdef XWORLD3D
    FLAGS_context = extract_py_dict_val(args, "context", false, 1);
    FLAGS_curriculum = extract_py_dict_val(args, "curriculum", false, 0.0f);
    FLAGS_x3_conf = extract_py_dict_val(args, "x3_conf", true, "");
    FLAGS_x3_task_mode = extract_py_dict_val(args, "x3_task_mode", false, "one_channel");
    FLAGS_x3_training_img_width
            = extract_py_dict_val(args, "x3_training_img_width", false, 64);
    FLAGS_x3_training_img_height
            = extract_py_dict_val(args, "x3_training_img_height", false, 64);
    FLAGS_x3_move_speed = extract_py_dict_val(args, "x3_move_speed", false, 25.0f);
    FLAGS_x3_turning_rad = extract_py_dict_val(args, "x3_turning_rad", false, M_PI / 8);
    FLAGS_x3_big_screen = extract_py_dict_val(args, "x3_big_screen", false, false);
    FLAGS_color = extract_py_dict_val(args, "color", false, false);
#endif
}

///////////////////// pass flags in python dict to GFlags ///////////////////
///////////////////// atari
void init_atari_gflags(const py::dict& args) {
#ifdef ATARI
    FLAGS_ale_rom = extract_py_dict_val(args, "ale_rom", true, "");
    FLAGS_context = extract_py_dict_val(args, "context", false, 4);
#endif
}

PySimulatorInterface::PySimulatorInterface(const std::string& name)
        : interface_(name, false) {}

PySimulatorInterface* PySimulatorInterface::create_simulator(
    const std::string& name, const py::dict& args) {

    FLAGS_pause_screen =
        extract_py_dict_val(args, "pause_screen", false, false);
    if (name == "simple_game") {
        init_simple_game_gflags(args);
    } else if (name == "simple_race") {
        init_simple_race_gflags(args);
    } else if (name == "xworld") {
        init_xworld_gflags(args);
    } else if (name == "xworld3d") {
        init_xworld3d_gflags(args);
    } else if (name == "atari") {
        init_atari_gflags(args);
    } else {
        throw PyException("Unrecognized game type: " + name);
    }

    auto g = new PySimulatorInterface(name);
    return g;
}

// Convert a Python dict of actions to StatePacket so that
// game_ can take an action
void convert_py_act_to_state_packet(const py::dict& actions, StatePacket& act) {
    std::vector<int> action(1, 0);
    act.add_buffer_id("action", action);
    // convert py::dict to StatePacket
    py::list keys = actions.keys();
    CHECK_GT(py::len(keys), 0) << "You can't take an empty action";
    for (int i = 0; i < py::len(keys); i++) {
        std::string k = py::extract<std::string>(keys[i]);
        if (k == "action") {
            action[0] = py::extract<int>(actions[keys[i]]);
            act.get_buffer("action")->set_id(action.begin(), action.end());
        } else if (k == "pred_sentence") {
            act.add_buffer_str("pred_sentence",
                               py::extract<std::string>(actions[keys[i]]));
        } else {
            throw PyException("Unrecognized key '" + k + "' for the actions");
        }
    }
}

float PySimulatorInterface::take_actions(const py::dict& actions, int act_rep, bool show_screen) {
    StatePacket act;
    convert_py_act_to_state_packet(actions, act);
    return interface_.take_actions(act, act_rep, show_screen);
}

float PySimulatorInterface::take_action(const py::dict& actions, bool show_screen) {
    return take_actions(actions, 1, show_screen);
}

void parse_extra_sim_info(std::string info, int& pid,
                          std::unordered_map<std::string, std::string>& parsed_info) {
    if (info == "") {
        return;
    }

    size_t idx = info.find("|");
    CHECK(idx != std::string::npos);
    pid = std::stoi(info.substr(0, idx));
    info = info.substr(idx + 1);

    parsed_info.clear();
    std::vector<std::string> tokens;
    boost::split(tokens, info, boost::is_any_of(","));
    for (const auto& t : tokens) {
        idx = t.find(":");
        CHECK(idx != std::string::npos);
        auto key = t.substr(0, idx);
        auto val = t.substr(idx + 1);
        parsed_info[key] = val;
    }
}

py::dict PySimulatorInterface::get_state() {
    auto state = interface_.get_state(0);

    py::dict d;
    auto keys = state.get_keys();
    for (const auto& k : keys) {
        // user will get reward outside this function from take_action()
        if (k == "reward") {
            continue;
        }
        if (k == "sentence") {
            auto sent = *(state.get_buffer(k)->get_str());
            d[k] = sent;
        } else {  // all the rest are vectors of floats
            auto buffer = state.get_buffer(k);
            std::vector<float> vec(buffer->get_value_size());
            float scale = 1.0;
            // we need to scale uint8 pixel values to [0,1]
            if (buffer->get_value()->is_uint8()) {
                scale = 1 / 255.0;
            }
            buffer->copy_value(vec.begin(), vec.end());
            py::list l;
            for (auto x : vec) {
                l.append(x * scale);
            }
            d[k] = l;
        }
    }
    // extra info
    std::string info;
    interface_.get_extra_info(info);
    int pid;
    std::unordered_map<std::string, std::string> parsed_info;
    parse_extra_sim_info(info, pid, parsed_info);
    for (const auto& i : parsed_info) {
        d[i.first] = i.second;
    }
    return d;
}

py::list PySimulatorInterface::get_screen_out_dimensions() {
    size_t height;
    size_t width;
    size_t channels;
    interface_.get_screen_out_dimensions(height, width, channels);
    py::list dims;
    dims.append(height);
    dims.append(width);
    dims.append(channels);
    return dims;
}

void help() {
    std::cout
        << "Games are created by calling Simulator.create()\n"
           "Example:\n"
           "  from py_simulator import Simulator\n"
           "  simple_game = Simulator.create(\"simple_game\", "
           "{\"array_size\":6})\n"
           "For more help on how to create games, call Simulator.help()\n";
}

BOOST_PYTHON_MODULE(py_simulator) {
    py::register_exception_translator<PyException>(&error_translate);
    py::def("help", help);
    py::class_<PySimulatorInterface, boost::noncopyable>("Simulator", py::no_init)
        .def("create",
             &PySimulatorInterface::create_simulator,
             py::return_value_policy<py::manage_new_object>())
        .staticmethod("create")
        .def("reset_game", &PySimulatorInterface::reset_game)
        .def("game_over", &PySimulatorInterface::game_over)
        .def("get_num_actions", &PySimulatorInterface::get_num_actions)
        .def("get_lives", &PySimulatorInterface::get_lives)
        // intentionally bind take_actions to two functions so that it has
        // optional args
        .def("take_actions", &PySimulatorInterface::take_actions)
        .def("take_action", &PySimulatorInterface::take_action)
        .def("get_state", &PySimulatorInterface::get_state)
        .def("get_num_steps", &PySimulatorInterface::get_num_steps);
}
