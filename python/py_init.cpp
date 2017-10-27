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

#include <Python.h>
#include <gflags/gflags.h>
#include <exception>
#include <functional>

DECLARE_string(task_mode);            // default "one_channel"
DECLARE_int32(curriculum);

struct PyException : std::exception {
    PyException(const std::string& msg) : msg_(msg) {}
    std::string error() const throw() { return msg_; }
    std::string msg_;
};

PyObject* get_gflag(PyObject* self, PyObject* args) {
    const char* c_name;
    if (!PyArg_ParseTuple(args, "s", &c_name)) {
        throw PyException("get_gflag args incorrect");
    }
    std::string flag_name = c_name;
    if (flag_name == "task_mode") {
        return PyString_FromString(FLAGS_task_mode.c_str());
    } else if (flag_name == "curriculum") {
        return Py_BuildValue("i", FLAGS_curriculum);
    } else {
        throw PyException("GFlag '" + flag_name + "' is not recognized");
    }
    return NULL;
}

// Used for executing global statements
// e.g., static InitFunction(__lambda);
class InitFunction {
  public:
    InitFunction(std::function<void()> init_f) { init_f(); }
};

static InitFunction py_init_interpreter([](){
    // initialize the python interpreter
    if (!Py_IsInitialized()) {
        Py_Initialize();
    }
});

static PyMethodDef cpp_methods[] = {
    {"get_flag", get_gflag, METH_VARARGS, "get gflags used by C++"},
    {NULL, NULL, 0, NULL}
};

static InitFunction py_init_modules([]() {
    Py_InitModule("py_gflags", cpp_methods);
});
