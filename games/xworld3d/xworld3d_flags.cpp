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

DEFINE_string(x3_glsl_path, "", "directory path to OpenGL shader scripts");
DEFINE_string(x3_model_dir, "", "3D model directory");
DEFINE_string(x3_conf, "", "XWorld 3D configuration file");
DEFINE_int32(x3_img_width, 256, "width of the rendering image");
DEFINE_int32(x3_img_height, 256, "height of the rendering image");
DEFINE_double(x3_reaching_distance, 0.12,
              "two objects are considered touched if the distance between them"
              " is smaller than this value");
DEFINE_double(x3_speed_norm, 10.0, "magnitude of moving speed");
DEFINE_double(x3_jump_speed, 3.0, "magnitude of jumping speed");
DEFINE_int32(x3_orientation_bins, 8, "number of orientation bins");
