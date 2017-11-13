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
#include "simulator.h"

DECLARE_string(ale_rom);
DECLARE_int32(ale_random_starts);

namespace simulator {
namespace arcade_game {

// This class is used to hide ale_interface.hpp dependency from game_player,
// so that cmake of game_player does not need to include xitari path
class ArcadeGame : public GameSimulator {
  public:
    static ArcadeGame* create();
};
}
}  // namespace simulator::arcade_game
