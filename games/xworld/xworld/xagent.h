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
#include <functional>
#include <string>
#include <vector>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "xitem.h"
#include "xmap.h"

namespace simulator { namespace xwd {

/**
*  class implements an agent interface
*
*/

class XAgent : public XItem {

public:

    // constructor
    XAgent(std::string name, Loc loc, const XMap* map, std::string img_name = "");

    // destructor
    virtual ~XAgent() {}

    // perform action specified by action_id
    virtual bool act(int action_id);

    // get the total number of valid actions
    virtual unsigned int get_num_of_actions();

    // update agents's location as [x, y]
    void update_location(int x, int y);

    // speak something (sentence)
    virtual std::string speak(const std::string& command); //communication channel

    // listen
    virtual std::string listen(const std::string& command);

private:

    // register agent's actions
    void register_actions();

    // agent's action set
    bool move_up();
    bool move_down();
    bool move_left();
    bool move_right();
    bool move_up_two_steps();
    bool move_down_two_steps();
    bool move_left_two_steps();
    bool move_right_two_steps();

    // a reference to the world map
    const XMap* map_;

    // action callback list
    std::vector< std::function<bool()> > action_callbacks_;
};

}} // namespace simulator::xwd
