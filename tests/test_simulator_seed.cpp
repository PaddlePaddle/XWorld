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
#include <simulator_util.h>
#include <iostream>
#include <thread>

DECLARE_int32(simulator_seed);

std::vector<int> vals;
const std::vector<int> sequence1 = {266148, 605992, 817626, 635637, 393423};
const std::vector<int> sequence2 = {258945, 847424, 238883, 918571, 875562};
const std::vector<int> sequence3 = {757419, 229641, 758547, 933608, 901914};

void rand_val() { vals.push_back(simulator::util::get_rand_ind(1000000)); }

TEST(Simulator, fixed_seed1) {
    FLAGS_simulator_seed = 1;  // fixed seed
    vals.clear();
    for (int i = 0; i < 5; i++) {
        auto th = std::thread(rand_val);
        th.join();
    }
    for (size_t i = 0; i < vals.size(); i++) {
        EXPECT_EQ(vals[i], sequence1[i]);
    }
}

TEST(Simulator, fixed_seed2) {
    FLAGS_simulator_seed = 2;  // fixed seed
    vals.clear();
    for (int i = 0; i < 5; i++) {
        auto th = std::thread(rand_val);
        th.join();
    }
    for (size_t i = 0; i < vals.size(); i++) {
        EXPECT_EQ(vals[i], sequence2[i]);
    }
}

TEST(Simulator, random_seed) {
    FLAGS_simulator_seed = 0;  // random seed
    vals.clear();
    std::vector<std::thread> ths;
    for (int i = 0; i < 5; i++) {
        ths.push_back(std::thread(rand_val));
    }
    for (size_t i = 0; i < 5; i++) {
        ths[i].join();
        EXPECT_NE(vals[i], sequence1[i]);
        EXPECT_NE(vals[i], sequence2[i]);
        // sequence3 is a previous arbitrary run of this test case
        EXPECT_NE(vals[i], sequence3[i]);
    }
}

int main(int argc, char *argv[]) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
