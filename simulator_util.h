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
#include <vector>
#include <random>
#include <ctime>
#include <algorithm>
#include <mutex>
#include <queue>
#include <memory>
#include <condition_variable>
#include <glog/logging.h>

namespace simulator { namespace util {

// Used for executing global statements
// e.g., static InitFunction(__lambda);
class InitFunction {
  public:
    InitFunction(std::function<void()> init_f) {
        init_f();
    }
};

// A timer that calculates running time for a code block
// Call REGISTER_TIMER() at the beginning of the block
class Timer {
  public:
    Timer(const std::string& name) {
        start_ = std::clock();
        name_ = name;
    }
    ~Timer() {
        LOG(INFO) << name_ << " uses "
                  << (std::clock() - start_) / (double)CLOCKS_PER_SEC
                  << " seconds.";
    }

  private:
    std::string name_;
    std::clock_t start_;
};

#define SIMULATOR_TIMER(name) auto __simulator_timer_##name__ = util::Timer(#name)

std::default_random_engine& thread_local_reng();

// get a random floating value in [0, upper_range]
float get_rand_range_val(const float upper_range);

// get a random int value in [0, size - 1]
int get_rand_ind(const int size);

// sample an index according to accumulated weights
int simple_importance_sampling(const std::vector<double>& acc_weights);

// this function should be used as a callback function in OpenCV's
// cv::setMouseCallback() function
// It will save the current screen on double clicks
void save_screen(int e, int x, int y, int d, void *ptr);

// generate a random string of length len
std::string generate_random_str(int len);

// remove the instance id from a word (e.g., apple_1 -> apple) if possible
std::string remove_instance_id(const std::string& word);

// remove the instance ids of all the words in a sentence
std::string remove_instance_ids(const std::string& sentence);

// compare the matching rate of one sentence (pred_sent) with a set of sentences
// by counting the ratio between number of matched words and total number of words
// and return the highest matching rate
double compare_sentences_multi(const std::vector<std::string>& sent_set,
                               const std::string& pred_sent);

// check a vector contains a unique value and has size >= 2
bool check_unique_and_different(const std::vector<std::string>& lst,
                                const std::string& value);

template<typename T>
void random_shuffle(std::vector<T>& array) {
    auto& reng = thread_local_reng();
    std::shuffle(array.begin(), array.end(), reng);
}

template <typename T>
T sample_set(const std::vector<T>& vec) {
    CHECK_GT(vec.size(), 0);
    return vec[get_rand_ind(vec.size())];
}

// copied from dl/util
std::string read_file(const std::string& fileName);

// copied from dl/util
template<typename T, typename... Args>
        std::unique_ptr<T> make_unique(Args&&... args) {
    return std::unique_ptr<T>(new T(args...));
}

} } // namespace simulator::util
