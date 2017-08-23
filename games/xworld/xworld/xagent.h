// MIT License

// Copyright (c) 2017 Baidu Inc. All rights reserved.

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
/////////////////////////////////////////////////////////////////////////////////

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
