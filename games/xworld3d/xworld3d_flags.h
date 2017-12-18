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

#pragma once

#include <gflags/gflags.h>

DECLARE_string(x3_conf);
DECLARE_double(x3_unit);
DECLARE_int32(x3_training_img_width);
DECLARE_int32(x3_training_img_height);
DECLARE_double(x3_reaching_distance);
DECLARE_double(x3_move_speed);
DECLARE_double(x3_jump_speed);
DECLARE_double(x3_turning_rad);
DECLARE_double(x3_gravity);
DECLARE_double(x3_time_step);
//DECLARE_int32(x3_frame_skip);
DECLARE_string(x3_task_mode);
DECLARE_bool(x3_big_screen);
