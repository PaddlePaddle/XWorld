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

#include <math.h>
#include "xworld3d_flags.h"

DEFINE_string(x3_conf, "", "XWorld 3D configuration file");
DEFINE_int32(x3_training_img_width, 64, "width of the training input image");
DEFINE_int32(x3_training_img_height, 64, "height of the training input image");
DEFINE_bool(x3_big_screen, true, "whether show big screen");

DEFINE_double(x3_unit, 1.0, "unit scale in xworld 3d");
//// NOTE: all the distance, velocity, and accelaration are scaled by x3_unit.
//// (i.e., adjacent blocks have a distance of x3_unit).
//// If you want to change the scale of the world, you need to change x3_unit.
//// All the relevant quantities will automatically scale by it.
DEFINE_double(x3_reaching_distance, 1.42,
              "two objects are considered touched if the distance between them"
              " is smaller than this value");
DEFINE_double(x3_move_speed, 25.0, "magnitude of moving speed");
DEFINE_double(x3_jump_speed, 20.0, "magnitude of jumping speed");
DEFINE_double(x3_turning_rad, M_PI/8, "turning degree in radian");
DEFINE_double(x3_gravity, 9.8, "gravitiy of the world");
DEFINE_double(x3_time_step, 0.0066, "time step for one simulation step");
//DEFINE_int32(x3_frame_skip, 1, "time step for one simulation step");
DEFINE_string(
    x3_task_mode,
    "arxiv_lang_acquisition",
    "arxiv_lang_acquisition|arxiv_interactive|one_channel");
