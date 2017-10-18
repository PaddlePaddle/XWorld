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

#include <data_packet.h>
#include <gtest/gtest.h>
#include <vector>

using namespace std;
using namespace simulator;

TEST(StatePacket, add_state_and_copy_value) {
    StatePacket s1;
    vector<int> a = {1, 2, 3, 4};
    vector<float> b = {1.5, 2.5, 3.5, 4.5, 5.5, 6.5};
    s1.add_buffer_value("screen", a);
    EXPECT_EQ(s1.size(), 1);
    auto buffer = s1.get_buffer("screen");
    EXPECT_EQ(buffer->get_value_size(), 4);
    buffer->copy_value(b.begin(), b.begin() + 4);
    EXPECT_FLOAT_EQ(1, b[0]);
    EXPECT_FLOAT_EQ(2, b[1]);
    EXPECT_FLOAT_EQ(3, b[2]);
    EXPECT_FLOAT_EQ(4, b[3]);
    EXPECT_FLOAT_EQ(5.5, b[4]);
    EXPECT_FLOAT_EQ(6.5, b[5]);
}

TEST(StatePacket, constructor_and_assignment) {
    StatePacket s1;
    s1.add_key("screen");
    s1.add_key("internal_state");
    vector<int> a = {1, 2, 3, 4};
    vector<float> b = {1.5, 2.5, 3.5, 4.5, 5.5, 6.5};
    vector<float> c = {5, 6, 7};

    BufferPtr buffer;
    s1.get_buffer("screen")->set_value(a.begin(), a.end());
    s1.get_buffer("internal_state")->set_value(b.begin(), b.end());

    buffer = s1.get_buffer("internal_state");

    StatePacket s2;
    StatePacket s3;
    s2.copy_from(s1);
    s3.copy_from(s2);
    buffer = s2.get_buffer("internal_state");
    buffer->set_value(c.begin(), c.end());
    EXPECT_EQ(buffer->get_value_size(), 3);

    buffer = s1.get_buffer("internal_state");
    EXPECT_EQ(buffer->get_value_size(), 6);

    buffer = s3.get_buffer("internal_state");
    EXPECT_EQ(buffer->get_value_size(), 6);

    auto data = s2.get_buffer("internal_state")->get_value<float>();
    EXPECT_FLOAT_EQ(5, data[0]);
    EXPECT_FLOAT_EQ(6, data[1]);
    EXPECT_FLOAT_EQ(7, data[2]);
    data = s3.get_buffer("internal_state")->get_value<float>();
    EXPECT_FLOAT_EQ(1.5, data[0]);
    EXPECT_FLOAT_EQ(2.5, data[1]);
    EXPECT_FLOAT_EQ(3.5, data[2]);
}

TEST(StatePacket, serialization) {
    StatePacket s1;
    s1.add_key("screen");
    s1.add_key("internal_state");
    vector<uint8_t> a = {1, 2, 3, 4};
    vector<int> id = {10, 11};
    vector<float> b = {1.5, 2.5, 3.5, 4.5, 5.5, 6.5};
    std::string str = "abc";
    s1.get_buffer("screen")->set_value(a.begin(), a.end());
    s1.get_buffer("screen")->set_id(id.begin(), id.end());
    s1.get_buffer("internal_state")->set_value(b.begin(), b.end());
    s1.get_buffer("internal_state")->set_str(str);
    util::BinaryBuffer buf;
    s1.encode(buf);

    buf.rewind();
    StatePacket s2;
    s2.decode(buf);
    EXPECT_TRUE(buf.eof());
    EXPECT_EQ(s2.contain_key("screen"), true);
    EXPECT_EQ(s2.contain_key("internal_state"), true);
    auto b1 = dynamic_pointer_cast<StateBuffer>(s1.get_buffer("screen"));
    auto b2 = dynamic_pointer_cast<StateBuffer>(s2.get_buffer("screen"));
    EXPECT_TRUE(*b1 == *b2);
    b1 = dynamic_pointer_cast<StateBuffer>(s1.get_buffer("internal_state"));
    b2 = dynamic_pointer_cast<StateBuffer>(s2.get_buffer("internal_state"));
    EXPECT_TRUE(*b1 == *b2);
}

int main(int argc, char *argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
