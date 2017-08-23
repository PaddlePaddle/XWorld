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

#include <gtest/gtest.h>
#include <algorithm>
#include "games/xworld/xworld/xworld.h"
#include <vector>
#include <cmath>
#include "simulator_entity.h"
#include "dirent.h"

using namespace simulator;
using namespace simulator::xwd;

const static int RANDOM_RESTARTS = 100;

std::string get_json_dir() {
    std::string path = __FILE__;
    int idx = path.find_last_of("/");
    return path.substr(0, idx) + "/../games/xworld/confs/test_xworld_parser";
}

TEST(XWorldParser, location1) {
    XWorldParser xp(false, get_json_dir() + "/location.json");
    WorldUnitList ul;
    int H;
    int W;
    xp.get_configuration(ul, H, W);
    EXPECT_EQ(H, 7);
    EXPECT_EQ(W, 7);
    EXPECT_EQ(ul.size(), 2);
    EXPECT_EQ(ul[0].type == GOAL
              && !ul[0].location.defined(), true);
    EXPECT_EQ(ul[1].type == GOAL
              && !ul[1].location.defined(), true);
}


TEST(XWorldParser, location2) {
    XWorldParser xp(false, get_json_dir() + "/partially_init.json");
    WorldUnitList ul;
    int H;
    int W;
    xp.get_configuration(ul, H, W);
    EXPECT_EQ(H, 3);
    EXPECT_EQ(W, 3);
    EXPECT_EQ(ul.size(), 7);

    EXPECT_EQ(ul[0].type == GOAL
              && ul[0].name == "apple_0"
              && ul[0].location.x == 1
              && ul[0].location.y == 2, true);
    EXPECT_EQ(ul[1].type == GOAL
              && !ul[1].location.defined(), true);
    EXPECT_EQ(ul[5].type == DUMMY
              && ul[5].name == "dummy_5"
              && ul[5].location.x == 0
              && ul[5].location.y == 1, true);
    EXPECT_EQ(ul[6].type == DUMMY
              && !ul[6].location.defined(), true);
}

TEST(XWorldParser, partiallyInit) {
    bool verify = true;
    std::vector<Entity> entities;
    for (int i = 0; i < RANDOM_RESTARTS; i ++) {
        XWorld xw(false, get_json_dir() + "/partially_init.json", 0);
        xw.get_all_items(entities);
        EXPECT_EQ(entities.size(), 5);  // not including agents
        for (auto& e : entities) {
            verify = verify && (e.type != "agent");
            if (e.location.x == 1 && e.location.y == 2) {
                verify = verify && e.id == "apple_0";
            }
            if (e.location.x == 0 && e.location.y == 1) {
                verify = verify && e.id == "dummy_5";
            }
        }
        xw.add_agent("robot_4");
        xw.add_agent("robot_100");
        xw.get_all_items(entities);
        for (auto& e : entities) {
            if (e.id == "robot_4") {
                verify = verify && e.location.x == 0 && e.location.y == 0;
            }
            if (e.id == "robot_100") {
                verify = verify && e.location.x == 1 && e.location.y == 0;
            }
        }
    }
    EXPECT_EQ(verify, true);
}

TEST(XWorldParser, unreachableGoal) {
    bool verify = true;
    std::vector<Entity> entities;
    XWorld xw(false, get_json_dir() + "/unreachable_goal.json", 0);
    xw.add_agent("robot_11");
    xw.get_all_items(entities);
    std::vector<Entity> temp;
    int n_blocks = 0;
    for (auto& e : entities) {
        if (e.type == "block")
            n_blocks ++;
        if (e.type == "goal")
            verify = verify && e.location == Vec3(3,1,0);
        if (e.type == "agent")
            verify = verify && e.location == Vec3(0,0,0);
        if (e.location == Vec3(1,0,0)
            || e.location == Vec3(1,1,0)
            || e.location == Vec3(1,2,0)) {
            temp.push_back(e);
        }
    }
    EXPECT_EQ(temp.size(), 3);
    for (auto& e : temp) {
        EXPECT_EQ(e.type, "block");
    }
    EXPECT_EQ(verify, true);
}

TEST(XWorldParser, reachableGoal) {
    bool verify = true;
    std::vector<Entity> entities;
    for (int i = 0; i < RANDOM_RESTARTS; i ++) {
        XWorld xw(false, get_json_dir() + "/reachable_goal.json", 0);
        xw.add_agent("robot_11");
        xw.get_all_items(entities);
        int n_blocks = 0;
        for (auto& e : entities) {
            if (e.type == "block")
                n_blocks ++;
            if (e.type == "goal")
                verify = verify && e.location == Vec3(3,1,0);
            if (e.type == "agent")
                verify = verify && e.location == Vec3(0,0,0);
        }
        verify = verify && n_blocks <= 7;
    }
    EXPECT_EQ(verify, true);
}

TEST(XWorldParser, category) {
    bool verify = true;
    std::vector<Entity> entities;
    for (int i = 0; i < RANDOM_RESTARTS; i ++) {
        XWorld xw(false, get_json_dir() + "/category.json", 0);
        xw.get_all_items(entities);
        for (auto& e : entities) {
            if (e.type == "goal") {
                std::string name = e.property("name");
                verify = verify
                        && (name == "blue"
                            || name == "green"
                            || name == "red"
                            || name == "yellow");
            }
        }
    }
    EXPECT_EQ(verify, true);
}

TEST(XWorldParser, randomClass) {
    bool verify = true;
    std::vector<Entity> entities;
    std::unordered_map<std::string, int> counter;
    std::vector<std::string> colors = {"blue", "green", "red", "yellow"};
    for (auto c : colors) {
        counter[c] = 0;
    }
    int random_restarts = RANDOM_RESTARTS * 500;
    for (int i = 0; i < random_restarts; i ++) {
        XWorld xw(false, get_json_dir() + "/random_class.json", 0);
        xw.get_all_items(entities);
        for (auto& e : entities) {
            if (e.type == "goal") {
                std::string name = e.property("name");
                counter[name] ++;
                verify = verify & (e.location == Vec3(1,1,0));
            }
        }
    }
    EXPECT_EQ(verify, true);
    for (auto c : colors) {
        EXPECT_LT(fabs(double(counter[c])/random_restarts - 1.0/colors.size()), 1e-2);
    }
}

int main(int argc, char *argv[]) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
