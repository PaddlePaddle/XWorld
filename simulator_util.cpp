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

#include "memory_util.h"
#include "simulator_util.h"
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <boost/algorithm/string.hpp>
#include <fstream>
#include <map>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <random>
#include <thread>

DEFINE_int32(simulator_seed, 0, "simulator rand engine seed");

namespace simulator {
namespace util {

/**
 * We wish to generate deterministic seeds for simulator threads if a global
 * FLAGS_simulator_seed is provided by the user.
 * The current solution is to use a global int (shared by different threads) to
 * count the order of threads.
 **/
static int __num_threads = 0;
class ThreadCounter {
  public:
    ThreadCounter() {
        reng_ = std::default_random_engine(
            std::hash<std::thread::id>()(std::this_thread::get_id()));
        if (FLAGS_simulator_seed) {
            // this might have race condition
            int seed = std::hash<std::string>()(
                std::to_string(FLAGS_simulator_seed + (++__num_threads)));
            reng_.seed(seed);
        }
    }
    std::default_random_engine reng_;
};
static thread_local ThreadCounter __tc;

std::default_random_engine& thread_local_reng() { return __tc.reng_; }

float get_rand_range_val(const float upper_range) {
    auto& reng = thread_local_reng();
    CHECK_GE(upper_range, 0);
    std::uniform_real_distribution<float> rand_outer(0, upper_range);
    float val = float(rand_outer(reng));
    CHECK_LE(val, upper_range);
    return val;
}

int get_rand_ind(const int size) {
    auto& reng = thread_local_reng();
    CHECK_GE(size, 1);
    std::uniform_int_distribution<int> rand_outer(0, size - 1);
    int ind = int(rand_outer(reng));
    CHECK_LE(ind, size - 1);
    return ind;
}

int simple_importance_sampling(const std::vector<double>& acc_weights) {
    CHECK_GT(acc_weights.size(), 0);
    // sample a weight
    double weight = get_rand_range_val(acc_weights.back());
    // find the interval
    for (size_t i = 0; i < acc_weights.size(); i++) {
        if (weight <= acc_weights[i]) {
            return i;
        }
    }
    LOG(FATAL) << "Weight out of range";
}

void save_screen(int e, int x, int y, int d, void* ptr) {
    if (e == cv::EVENT_LBUTTONDBLCLK) {
        cv::Mat* frame_ptr = (cv::Mat*)ptr;
        CHECK(frame_ptr != nullptr);
        std::string write_path = "/tmp/" + generate_random_str(6) + ".png";
        cv::imwrite(write_path,
                    *frame_ptr,
                    std::vector<int>({CV_IMWRITE_PNG_COMPRESSION, 0})
                    /*best*/);
        LOG(INFO) << "Current frame saved to: " << write_path;
    }
}

std::string generate_random_str(int len) {
    static const std::string alphanum =
        "0123456789"
        "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
        "abcdefghijklmnopqrstuvwxyz";
    std::string str = "";
    for (int i = 0; i < len; i++) {
        str += alphanum[get_rand_ind(alphanum.length())];
    }
    return str;
}

std::string remove_instance_ids(const std::string& sentence) {
    std::vector<std::string> words;
    boost::split(words, sentence, boost::is_any_of(" "));
    // remove all the instance ids in the command
    std::string ret = "";
    for (size_t i = 0; i < words.size(); i++) {
        if (i > 0) {
            ret += " ";
        }
        ret += remove_instance_id(words[i]);
    }
    return ret;
}

std::string remove_instance_id(const std::string& word) {
    size_t idx = word.find('_');
    return word.substr(0, idx);
}

double compare_sentences_multi(const std::vector<std::string>& sent_set,
                               const std::string& pred_sent) {
    auto count_matched_word = [](std::vector<std::string> src_words,
                                 std::vector<std::string> dst_words) {
        // count the number of words in src_words matched with dst_words
        std::map<std::string, int> mapOfWords;

        int matched_word_cnt = 0;  // number of matched between gt and predicted
        for (size_t i = 0; i < dst_words.size(); i++) {
            if (mapOfWords.find(dst_words[i]) != mapOfWords.end()) {
                mapOfWords[dst_words[i]]++;
            } else {
                mapOfWords[dst_words[i]] = 1;
            }
        }
        for (size_t i = 0; i < src_words.size(); i++) {
            if (mapOfWords.find(src_words[i]) != mapOfWords.end() &&
                mapOfWords[src_words[i]] > 0) {
                matched_word_cnt++;
                mapOfWords[src_words[i]]--;
            }
        }
        return matched_word_cnt;
    };

    double rate = 0.0;
    std::vector<std::string> words, pred_words;
    boost::split(pred_words, pred_sent, boost::is_any_of(" "));
    for (size_t i = 0; i < sent_set.size(); i++) {
        if (sent_set[i] == "" && pred_sent == "") {
            rate = 1.0;
            return rate;
        } else {
            boost::split(words, sent_set[i], boost::is_any_of(" "));
            int matched_word_cnt = count_matched_word(words, pred_words) +
                                   count_matched_word(pred_words, words);
            double single_rate = double(matched_word_cnt) /
                                 double(pred_words.size() + words.size());
            rate = std::max(rate, single_rate);
        }
    }
    return rate;
}

bool check_unique_and_different(const std::vector<std::string>& lst,
                                const std::string& value) {
    return lst.size() > 1 && std::count(lst.begin(), lst.end(), value) == 1;
}

std::string read_file(const std::string& fileName) {
    std::ifstream is(fileName);

    // get length of file:
    is.seekg(0, is.end);
    size_t length = is.tellg();
    is.seekg(0, is.beg);
    std::string str(length, (char)0);
    CHECK(is.read(&str[0], length)) << "Fail to read file: " << fileName;
    return str;
}

std::string path_join(std::initializer_list<std::string> list) {
    std::string ret = "";
    if (list.size() > 0) {
        ret = *list.begin();
        for (auto it = list.begin() + 1; it != list.end(); ++it) {
            ret += "/" + *it;
        }
    }
    return ret;
}

}}  // namespace simulator::util
