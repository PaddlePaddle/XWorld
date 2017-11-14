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

#include <memory_util.h>
#include <gtest/gtest.h>
#include <math.h>
#include <string>
#include <vector>

using namespace std;
using namespace simulator::util;

template <typename T>
bool equal(BinaryBuffer& buf, const std::vector<T>& v) {
    buf.rewind();
    T i;
    for (auto& e : v) {
        if (buf.eof()) {
            return false;
        }
        buf.read(i);
        if (i != e) {
            return false;
        }
    }
    return buf.eof();
}

template <typename T>
bool equal(BinaryBuffer& buf, const T* v, int num) {
    buf.rewind();
    T x;
    for (int i = 0; i < num; ++i) {
        if (buf.eof()) {
            return false;
        }
        buf.read(x);
        if (x != v[i]) {
            return false;
        }
    }
    return buf.eof();
}

bool equal(BinaryBuffer& b1,  BinaryBuffer& b2) {
    b1.rewind();
    b2.rewind();
    int i, j;
    while (!b1.eof()) {
        if (b2.eof()) {
            return false;
        }
        b1.read(i);
        b2.read(j);
        if (i != j) {
            return false;
        }
    }
    return b2.eof();
}

TEST(BinaryBuffer, constructor) {
    BinaryBuffer float_dup(4, 10);
    EXPECT_EQ(float_dup.size(), 4);
    EXPECT_EQ(float_dup.capacity(), 10);

    std::vector<int> v({1,2,3,4,5});
    BinaryBuffer int_buf(v.data(), sizeof(int)*v.size());
    EXPECT_EQ(int_buf.size(), sizeof(int)*v.size());
    EXPECT_EQ(int_buf.capacity(), sizeof(int)*v.size());
    EXPECT_TRUE(equal(int_buf, v));

    BinaryBuffer int_dup(int_buf);
    EXPECT_EQ(int_dup.size(), sizeof(int)*v.size());
    EXPECT_EQ(int_dup.capacity(), sizeof(int)*v.size());
    EXPECT_TRUE(equal(int_buf, int_dup));

    const int N = 10;
    float* f = new float [N];
    for (int i = 0; i < N; ++i) {
        f[i] = i;
    }

    BinaryBuffer float_buf(f, sizeof(float) * 4);
    EXPECT_EQ(float_buf.size(), sizeof(float) * 4);
    EXPECT_EQ(float_buf.capacity(), sizeof(float) * 4);
    EXPECT_TRUE(equal(float_buf, f, 4));

    delete [] f;
}

TEST(BinaryBuffer, assignment) {
    // operator=
    std::vector<int> v({1,2,3,4,5});
    BinaryBuffer int_buf(v.data(), sizeof(int)*v.size());
    BinaryBuffer int_dup;
    int_dup = int_buf;
    EXPECT_EQ(equal(int_buf, int_dup), true);

    const int N = 10;
    float* f = new float [N];
    for (int i = 0; i < N; ++i) {
        f[i] = i;
    }

    // take over
    BinaryBuffer float_buf(4);
    float_buf.take_over(f, 5*sizeof(float));
    EXPECT_EQ(float_buf.size(), sizeof(float)*5);
    EXPECT_EQ(float_buf.capacity(), sizeof(float)*5);
    EXPECT_TRUE(equal(float_buf, f, 5));

    // assignment with capacity unchanged
    float_buf.assign(f, 4*sizeof(float));
    EXPECT_EQ(float_buf.size(), sizeof(float)*4);
    EXPECT_EQ(float_buf.capacity(), sizeof(float)*5);
    EXPECT_TRUE(equal(float_buf, f, 4));

    // assignment with capacity increased
    float_buf.assign(f, N*sizeof(float));
    EXPECT_EQ(float_buf.size(), sizeof(float)*N);
    EXPECT_EQ(float_buf.capacity(), sizeof(float)*N);
    EXPECT_TRUE(equal(float_buf, f, N));
}

TEST(BinaryBuffer, resize_reserve) {
    std::vector<int> v({1,2,3,4});
    BinaryBuffer int_buf(v.data(), sizeof(int)*v.size());

    // reserve
    EXPECT_TRUE(int_buf.reserve(10*sizeof(int)));
    EXPECT_EQ(int_buf.size(), v.size()*sizeof(int));
    EXPECT_EQ(int_buf.capacity(), 16*sizeof(int));

    // resize
    int_buf.resize(v.size()*sizeof(int));
    EXPECT_EQ(int_buf.size(), v.size()*sizeof(int));
    EXPECT_EQ(int_buf.capacity(), 16*sizeof(int));
    EXPECT_TRUE(equal(int_buf, v));
}

TEST(BinaryBuffer, read_write) {
    BinaryBuffer b1;
    BinaryBuffer b2;
    BinaryBuffer b3;
    BinaryBuffer b4;
    BinaryBuffer b5;
    std::vector<int> v({1,2,3,4});
    float f[3] = {4, 5, 6};
    float ff[6];
    {
        // vector
        b1.append(v);
        b1.append(std::vector<float>(0));
        b1.rewind();
        std::vector<int> v1, v2;
        b1.read(v1);
        b1.read(v2);
        EXPECT_EQ(v1.size(), v.size());
        EXPECT_EQ(v1[0], 1);
        EXPECT_EQ(v1[1], 2);
        EXPECT_EQ(v1[2], 3);
        EXPECT_EQ(v1[3], 4);
        EXPECT_EQ(v2.size(), 0);
    }

    {
        // T*
        b2.append(f, 3);
        b2.append(f, 3);
        b2.rewind();
        b2.read(ff, 6);
        EXPECT_EQ(b2.size(), 6 * sizeof(float));
        EXPECT_LE(fabs(ff[0] - 4), 1e-4);
        EXPECT_LE(fabs(ff[1] - 5), 1e-4);
        EXPECT_LE(fabs(ff[2] - 6), 1e-4);
        EXPECT_LE(fabs(ff[3] - 4), 1e-4);
        EXPECT_LE(fabs(ff[4] - 5), 1e-4);
        EXPECT_LE(fabs(ff[5] - 6), 1e-4);
    }

    {
        // string
        std::string str("789");
        b3.append(str);
        b3.append(std::string(""));
        std::string tmp;
        b3.rewind();
        b3.read(tmp);
        EXPECT_EQ(tmp.length(), 3);
        EXPECT_EQ(tmp, str);
        b3.read(tmp);
        EXPECT_EQ(tmp.length(), 0);
        EXPECT_EQ(tmp, "");
    }

    // BinaryBuffer
    {
        b1.append(b2);
        b1.rewind();
        std::size_t sz;
        b1.read(sz);
        EXPECT_EQ(sz, v.size());
        int x;
        for (size_t i = 0; i < sz; ++i) {
            b1.read(x);
            EXPECT_EQ(x, v[i]);
        }
        b1.read(sz);
        EXPECT_EQ(sz, 0);
        float f;
        for (int i = 0; i < 3; ++i) {
            b1.read(f);
            EXPECT_LE(fabs(f - ff[i]), 1e-5);
        }
        b1.read(f);
    }

    // insert
    {
        std::string s("a");

        b5.append(int(1));
        b5.append(s);
        vector<int> a({2,3});
        b5.insert(sizeof(int), a.data(), a.size());
        b5.rewind();
        for (int i = 1; i <= 3; ++i) {
            int v;
            b5.read(v);
            EXPECT_EQ(v, i);
        }
        b5.read(s);
        EXPECT_EQ(s, "a");
    }
}

int main(int argc, char *argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
