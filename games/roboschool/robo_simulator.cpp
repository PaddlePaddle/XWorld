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

#include "robo_simulator.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <glog/logging.h>
#include "numpy/ndarrayobject.h"

DEFINE_int32(robo_height, 64, "The height of the roboschool");
DEFINE_int32(robo_width, 64, "The width of the roboschool");

namespace py = boost::python;

namespace simulator { namespace robo_simulator {

std::string parse_python_exception();

void RoboSimulator::show_screen(float reward) {
    cv::Mat img(_img_height, _img_width, CV_8UC3);
    for (size_t h = 0, base = 0; h < _img_height; ++h) {
        for (size_t w = 0; w < _img_width; ++w, base += 3) {
            cv::Vec3b& color = img.at<cv::Vec3b>(cv::Point(w, h));
            color[0] = _screen_vec[base + 2];
            color[1] = _screen_vec[base + 1];
            color[2] = _screen_vec[base];
        }
    }
    std::lock_guard<std::mutex> guard(GameSimulator::s_display_mutex_);
    cv::imshow("show_screen", img);

    if (_human_control) {
        char user_input = cv::waitKey(40);
        std::vector<int> action_vec;
        int action_id = 4; // 4: stay
        if (_action_to_id.count(user_input) > 0) {
            action_id = _action_to_id[user_input];
        }
        action_vec.push_back(action_id);
        StatePacket action;
        action.add_buffer_id("action", action_vec);
        take_action(action);
    } else {
        cv::waitKey(1);
    }
}

RoboSimulator::RoboSimulator(bool human_control) {
    LOG(INFO) << "robo simulator start";
    _img_height = FLAGS_robo_height; _img_width = FLAGS_robo_width;
    _screen_vec.resize(_img_height * _img_width * 3);
    LOG(INFO) << "height: " << _img_height << " width: " << _img_width;
    _human_control = human_control;
    if (_human_control) {
        LOG(INFO) << "program controlled by human in show_screen";
    }
    int action_id = 0;
    // move forward
    _action_space.push_back('w'); _action_to_id['w'] = action_id++;
    // turn left
    _action_space.push_back('a'); _action_to_id['a'] = action_id++;
    // turn right
    _action_space.push_back('d'); _action_to_id['d'] = action_id++;
    // collect
    _action_space.push_back('c'); _action_to_id['c'] = action_id++;
    // stay
    _action_space.push_back('s'); _action_to_id['s'] = action_id++;
    // jump (if allowed in the env config script)
    _action_space.push_back('j'); _action_to_id['j'] = action_id++;
    if (_human_control) {
        // change the viewpoint
        _action_space.push_back('z'); _action_to_id['z'] = action_id;
    }
}

void RoboSimulator::reset_game() {
    LOG(INFO) << "robo simulator reset";
    if (not _reset_init) {
        Py_Initialize();
        py::object main_module = py::import("__main__");
        py_space = main_module.attr("__dict__");
        try {
            py::exec("import gym", py_space);
            py::exec("import roboschool", py_space);
            py::exec("import numpy as np", py_space);
            py::exec("env = gym.make('RoboschoolDemo-v0')", py_space);
            py::exec("obs = env.reset(); a = np.array(['s', ])", py_space);
            py::exec("obs, r, done, _ = env.step(a)", py_space);
            py::exec("rgb = env.render('rgb_array')", py_space);
        } catch (py::error_already_set const &) {
            LOG(ERROR) << parse_python_exception();
            exit(1);
        }
        _reset_init = true;
    }
    try {
        py::exec("obs = env.reset(); a = np.array(['s', ])", py_space);
        py::exec("obs, r, done, _ = env.step(a)", py_space);
    } catch (py::error_already_set const &) {
        LOG(ERROR) << parse_python_exception();
        exit(1);
    }
    GameSimulator::reset_game();
}

int RoboSimulator::game_over() {
    // max_step is handled by the roboschool env
    py::object done_obj = py_space["done"];
    int done = py::extract<int>(done_obj);
    if (done == 2) {
        LOG(INFO) << "found the goal";
        return SUCCESS;
    }
    else if (done == 1) {
        return ALIVE;
    }
    else {
        return DEAD;
    }
}

float RoboSimulator::take_action(const StatePacket& actions) {
    CHECK_EQ(actions.size(), 1);
    int action_id = *(actions.get_buffer("action")->get_id());
    try {
        if (action_id < _action_space.size()) {
            std::string py_command("a = np.array(['");
            py_command += _action_space[action_id];
            py_command += "', ])";
            py::exec(py_command.c_str(), py_space);
        } else {
            LOG(ERROR) << "Unknown action_id" << action_id;
        }
        py::exec("obs, r, done, _ = env.step(a)", py_space);
    } catch (py::error_already_set const &) {
        LOG(ERROR) << parse_python_exception();
        exit(1);
    }
    py::object reward_obj = py_space["r"];
    double reward = py::extract<double>(reward_obj);
    return float(reward);
}

int RoboSimulator::get_num_actions() {
    LOG(INFO) << "robo simulator get num actions";
    return _action_space.size();
}

int RoboSimulator::get_lives() {
    LOG(INFO) << "robo simulator get lives";
    return _max_lives;
}

void RoboSimulator::get_screen(StatePacket& screen) {
    // get data from roboschool
    try {
        py::exec("rgb = env.render('rgb_array')", py_space);
    } catch (py::error_already_set const &) {
        LOG(ERROR) << parse_python_exception();
        exit(1);
    }
    py::object rgb_obj = py_space["rgb"];
    PyObject* rgb_array = rgb_obj.ptr();
    // Py_INCREF(rgb_array);
    CHECK_EQ(PyArray_TYPE(rgb_array), NPY_UBYTE);
    CHECK_EQ(PyArray_NDIM(rgb_array), 3);
    const npy_intp* _sizes = PyArray_DIMS(rgb_array);
    CHECK_EQ(_img_height, _sizes[0]);
    CHECK_EQ(_img_width, _sizes[1]);
    uint8_t* rgb_data = (uint8_t*) PyArray_DATA(rgb_array);
    for (int i = 0; i < 3 * _img_height * _img_width; ++i) {
        _screen_vec[i] = rgb_data[i];
    }
    // prepare data
    screen = StatePacket();
    screen.add_buffer_value("screen", _screen_vec);
}

void RoboSimulator::define_state_specs(StatePacket& state) {
    state = StatePacket();
    state.add_key("reward");
    state.add_key("screen");
    // do something here
}

void RoboSimulator::get_screen_out_dimensions(size_t& height, size_t& width,
        size_t& channels) {
    height = _img_height;
    width = _img_width;
    channels = 3;
}

// Parses the value of the active python exception
// NOTE SHOULD NOT BE CALLED IF NO EXCEPTION
// https://github.com/josephturnerjr/boost-python-tutorial/
std::string parse_python_exception() {
    PyObject *type_ptr = NULL;
    PyObject *value_ptr = NULL;
    PyObject *traceback_ptr = NULL;
    // Fetch the exception info from the Python C API
    PyErr_Fetch(&type_ptr, &value_ptr, &traceback_ptr);

    // Fallback error
    std::string ret("Unfetchable Python error");
    // If the fetch got a type pointer, parse the type into the exception string
    if (type_ptr != NULL){
        py::handle<> h_type(type_ptr);
        py::str type_pstr(h_type);
        // Extract the string from the boost::python object
        py::extract<std::string> e_type_pstr(type_pstr);
        // If a valid string extraction is available, use it
        //  otherwise use fallback
        if (e_type_pstr.check())
            ret = e_type_pstr();
        else
            ret = "Unknown exception type";
    }
    // Do the same for the exception value the stringification of the exception
    if (value_ptr != NULL){
        py::handle<> h_val(value_ptr);
        py::str a(h_val);
        py::extract<std::string> returned(a);
        if (returned.check())
            ret +=  ": " + returned();
        else
            ret += std::string(": Unparseable Python error: ");
    }
    // Parse lines from the traceback using the Python traceback module
    if (traceback_ptr != NULL){
        py::handle<> h_tb(traceback_ptr);
        // Load the traceback module and the format_tb function
        py::object tb(py::import("traceback"));
        py::object fmt_tb(tb.attr("format_tb"));
        // Call format_tb to get a list of traceback strings
        py::object tb_list(fmt_tb(h_tb));
        // Join the traceback strings into a single string
        py::object tb_str(py::str("\n").join(tb_list));
        // Extract the string, check the extraction, and fallback in necessary
        py::extract<std::string> returned(tb_str);
        if (returned.check())
            ret += ": " + returned();
        else
            ret += std::string(": Unparseable Python traceback");
    }
    return ret;
}

}} // namespace simulator::robo_simulator
