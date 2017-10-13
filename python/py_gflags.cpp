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
#include <boost/python/exception_translator.hpp>
#include <exception>

namespace py = boost::python;

//// xworld
DECLARE_bool(task_groups_exclusive);  // default true
DECLARE_string(task_mode);            // default "one_channel"

DECLARE_int32(curriculum);

struct PyException : std::exception {
    PyException(const std::string& msg) : msg_(msg) {}
    std::string error() const throw() { return msg_; }
    std::string msg_;
};

void error_translate(PyException const& e) {
    PyErr_SetString(PyExc_RuntimeError, e.error().c_str());
}

/* For C++ objects whose type has been exposed via boost::python::class_,
   one can construct a Python object with an instance of a C++ object using
   the following constructor:

   template <class T>
   explicit object(T const& x);
*/
py::object get_flag(const std::string& flag_name) {
    if (flag_name == "task_mode") {
        return py::object(FLAGS_task_mode);
    } else if (flag_name == "curriculum") {
        return py::object(FLAGS_curriculum);
    } else {
        throw PyException("GFlag '" + flag_name + "' is not recognized");
    }
    return py::object();
}

BOOST_PYTHON_MODULE(py_gflags) {
    py::register_exception_translator<PyException>(&error_translate);
    py::def("get_flag", get_flag);
}
