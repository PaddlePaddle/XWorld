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
#include "teacher_sentence_generator.h"

using namespace simulator;

class SenTempDemo : public TeacherSentenceTemplate {
  public:
    SenTempDemo(const std::vector<std::string>& held_out)
            : TeacherSentenceTemplate(held_out) {
        define_rules();
    }
  private:
    void define_rules() {
        add_rule(start_symbol_, {"$Statement", "$Question"}, true /*bound*/);
        add_rule("$Statement", {"Earth is $Shape ."}, false);
        add_rule("$Shape", {"round", "flat"}, true);
        add_rule("$Question", {"Can you say a wrong statement ?",
                               "Can you say a correct statement ?"}, false);
        check_rules();
    }
};

class SenTempInf : public TeacherSentenceTemplate {
  public:
    SenTempInf() {
        define_rules();
    }

  private:
    void define_rules() {
        // make a loop in the rules
        add_rule(start_symbol_, {"$A $B", "$A"}, true);
        add_rule("$A", {"D E", "$B F"});
        add_rule("$B", {"$A C"});
        check_rules();
    }
};

class SenTempNoInf : public TeacherSentenceTemplate {
  public:
    SenTempNoInf() {
        define_rules();
    }
  private:
    void define_rules() {
        // make a loop in the rules
        add_rule(start_symbol_, {"$A $B", "$A"});
        add_rule("$A", {"D E", "$B F"});
        add_rule("$B", {"A C"});
        check_rules();
    }
};

TEST(SentenceTemplate, MustBound) {
    auto sen_temp = std::make_shared<SenTempDemo>(std::vector<std::string>());
    // directly instantiate without binding would cause error
    EXPECT_DEATH(sen_temp->instantiate(), ".*must_bound.*");
    // only bind one rule is still not enough
    sen_temp->bind("$S", "$Statement");
    EXPECT_DEATH(sen_temp->instantiate(), ".*must_bound.*");

    // if previous failure, the already bound symbol is still bound
    sen_temp->bind("$Shape", "round");
    EXPECT_EQ(sen_temp->instantiate(), "Earth is round .");

    sen_temp->bind("$S", "$Question");
    auto str = sen_temp->instantiate();
    EXPECT_TRUE(str == "Can you say a wrong statement ?"
                || str == "Can you say a correct statement ?");
}

TEST(SentenceTemplate, BindingBacktrack) {
    auto sen_temp = std::make_shared<SenTempDemo>(std::vector<std::string>());
    sen_temp->bind("$S", "$Statement");
    sen_temp->bind("$Shape", "flat");
    EXPECT_EQ(sen_temp->instantiate(), "Earth is flat .");
    // after the instantiation, none is bound
    EXPECT_DEATH(sen_temp->instantiate(), ".*must_bound.*");
    sen_temp->bind("$S", "$Statement");
    sen_temp->bind("$Shape", "round");
    EXPECT_EQ(sen_temp->instantiate(), "Earth is round .");
}

TEST(SentenceTemplate, HeldOut) {
    std::vector<std::string> held_out = {"flat", "wrong"};
    auto sen_temp = std::make_shared<SenTempDemo>(held_out);
    sen_temp->bind("$S", "$Statement");
    sen_temp->bind("$Shape", "flat");
    EXPECT_EQ(sen_temp->instantiate(), "Earth is flat .");
    sen_temp->bind("$S", "$Statement");
    sen_temp->bind("$Shape", "flat");
    EXPECT_EQ(sen_temp->instantiate(true), ""); // banned

    for (int i = 0; i < 100; i ++) {
        sen_temp->bind("$S", "$Question");
        auto sen = sen_temp->instantiate(true);
        EXPECT_TRUE(sen.find("wrong") == std::string::npos);
    }
}

TEST(SentenceTemplate, InfiniteRecursion) {
    EXPECT_DEATH(std::make_shared<SenTempInf>(), ".*infinite.*");
    std::make_shared<SenTempNoInf>(); // shouldn't crash
}

int main(int argc, char *argv[]) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
