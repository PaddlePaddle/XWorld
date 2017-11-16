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

#include "xworld3d_flags.h"

DEFINE_string(x3_conf, "", "XWorld 3D configuration file");
DEFINE_int32(x3_training_img_width, 64, "width of the training input image");
DEFINE_int32(x3_training_img_height, 64, "height of the training input image");


//// NOTE: all the distance, velocity, and accelaration are specified by the user
//// with a *unit* scale (i.e., adjacent blocks have a distance of 1).
//// If you want to change the scale of the world, you need to change the UNIT variable
//// in x3item.h. All the relevant quantities will automatically scale by it.
DEFINE_double(x3_reaching_distance, 1.2,
              "two objects are considered touched if the distance between them"
              " is smaller than this value");
DEFINE_double(x3_move_speed, 50.0, "magnitude of moving speed");
DEFINE_double(x3_jump_speed, 20.0, "magnitude of jumping speed");
DEFINE_int32(x3_orientation_bins, 8, "number of orientation bins");
