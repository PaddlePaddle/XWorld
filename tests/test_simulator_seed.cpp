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

#include <gtest/gtest.h>
#include <thread>
#include <simulator_util.h>
#include <iostream>

DECLARE_int32(simulator_seed);

std::vector<int> vals;
const std::vector<int> sequence1 = {266148, 605992, 817626, 635637, 393423};
const std::vector<int> sequence2 = {258945, 847424, 238883, 918571, 875562};
const std::vector<int> sequence3 = {757419, 229641, 758547, 933608, 901914};

void rand_val() {
    vals.push_back(simulator::util::get_rand_ind(1000000));
}

TEST(Simulator, fixed_seed1) {
    FLAGS_simulator_seed = 1;  // fixed seed
    vals.clear();
    for (int i = 0; i < 5; i ++) {
        auto th = std::thread(rand_val);
        th.join();
    }
    for (size_t i = 0; i < vals.size(); i ++) {
        EXPECT_EQ(vals[i], sequence1[i]);
    }
}

TEST(Simulator, fixed_seed2) {
    FLAGS_simulator_seed = 2;  // fixed seed
    vals.clear();
    for (int i = 0; i < 5; i ++) {
        auto th = std::thread(rand_val);
        th.join();
    }
    for (size_t i = 0; i < vals.size(); i ++) {
        EXPECT_EQ(vals[i], sequence2[i]);
    }
}

TEST(Simulator, random_seed) {
    FLAGS_simulator_seed = 0;  // random seed
    vals.clear();
    std::vector<std::thread> ths;
    for (int i = 0; i < 5; i ++) {
        ths.push_back(std::thread(rand_val));
    }
    for (size_t i = 0; i < 5; i ++) {
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
