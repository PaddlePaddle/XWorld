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

#include "teaching_task.h"

namespace simulator {

namespace py = boost::python;

void PyTask::register_stages() {
    CHECK(PyObject_HasAttrString(py_task_.ptr(), "get_stage_names"))
            << "Python function get_stage_names() is undefined!";
    py::list py_stage_names = py::extract<py::list>(py_task_.attr("get_stage_names")());

    std::vector<std::string> stage_names;
    for (int i = 0; i < py::len(py_stage_names); i ++) {
        stage_names.push_back(py::extract<std::string>(py_stage_names[i]));
    }

    for (const auto& name : stage_names) {
        stages_[name] = [this, name]() {
            return py_stage(sen_temp_, game_, name);
        };
    }
}

py::dict PyTask::convert_entity_to_py_entity(const Entity& e) {
    py::dict py_e;
    py_e["id"] = e.id;
    py_e["type"] = e.type;
    py_e["location"] = py::make_tuple(e.location.x, e.location.y, e.location.z);
    auto keys = e.all_properties();
    for (const auto& k : keys) {
        py_e[k] = e.property(k);
    }
    return py_e;
}

std::string PyTask::py_stage(SentenceTemplatePtr sen_temp, TeachingEnvPtr game,
                             const std::string& stage_name) {
    CHECK(PyObject_HasAttrString(py_task_.ptr(), stage_name.c_str()))
            << "Python task stage is undefined: " << stage_name;

    std::vector<Entity> entities;
    game->get_all_entities(entities);

    py::list py_entities;  // a list of py::dict, each being a py_entity
    for (const auto& e : entities) {
        py_entities.append(convert_entity_to_py_entity(e));
    }

    double X, Y, Z;
    game->get_world_dimensions(X, Y, Z);
    py::list dims;
    dims.append(X);
    dims.append(Y);
    dims.append(Z);
    // run stage_name
    py::list ret = py::extract<py::list>(py_task_.attr(stage_name.c_str())(
        py_entities, dims));

    CHECK_EQ(py::len(ret), 3) << "Incorrect length of stage returns";
    std::string next_stage = py::extract<std::string>(ret[0]);
    double reward = py::extract<double>(ret[1]);
    std::string sentence = py::extract<std::string>(ret[2]);

    give_reward(reward);
    // We directly "generate" the returned sentence from the python stage function
    sen_temp->add_rule(sen_temp->start_symbol(), {sentence});
    Task::teacher_speak(false, name_, sen_temp, game);
    return next_stage;
}

}
