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

#include "teacher_sentence_generator.h"
#include <boost/algorithm/string.hpp>
#include <functional>

namespace simulator {

std::string SenTempRHS::sample(int num_games, int curriculum_games) {
    CHECK(!must_bound_) << "RHS must be bound first";
    int total = rhs_.size() - 1;
    if (curriculum_games) {
        total = curriculum_number(0, total, num_games, curriculum_games);
    }
    return rhs_[util::get_rand_ind(total + 1)];
}

std::string SenTempRHS::bound_value() {
    CHECK_EQ(rhs_.size(), 1) << "Must be bound; otherwise use sample()";
    return rhs_[0];
}

void SenTempRHS::bind(const std::string& r) {
    CHECK(must_bound_) << "RHS shouldn't be bound";
    unbind();
    CHECK_GT(std::count(rhs_.begin(), rhs_.end(), r), 0) << r;
    rhs_ = std::vector<std::string>({r});
}

void SenTempRHS::unbind() { rhs_ = rhs_backup_; }

int SenTempRHS::curriculum_number(int low,
                                  int high,
                                  int num_games,
                                  int curriculum_games) {
    CHECK_GT(curriculum_games, 0);
    // gradually increase the difficulty
    // we assume the bigger the number, the more difficulty there is
    double progress = std::min(double(num_games) / curriculum_games, 1 - 1e-5);
    return low + int((high - low + 1) * progress);
}

void SenTempRHS::sort_by_num_words() {
    std::sort(rhs_.begin(),
              rhs_.end(),
              [](const std::string& str1, const std::string& str2) {
                  std::vector<std::string> words1, words2;
                  boost::split(words1, str1, boost::is_any_of(" "));
                  boost::split(words2, str2, boost::is_any_of(" "));
                  return words1.size() < words2.size();
              });
}

void TeacherSentenceTemplate::add_rule(const std::string& lhs,
                                       const std::vector<std::string>& rhs,
                                       bool bound) {
    rules_[lhs] = SenTempRHS(rhs, bound);
}

void TeacherSentenceTemplate::check_rules() {
    // check start_symbol_ is on the lhs
    CHECK(rules_.count(start_symbol_));
    // check the symbols in the rules are all valid
    for (const auto& r : rules_) {
        CHECK(is_symbol(r.first)) << r.first << " must be a symbol";
    }
    check_infinite_recursion();
}

void TeacherSentenceTemplate::check_infinite_recursion() {
    // Each node i has three status: not in visited, visited[i]=false,
    // visited[i]=true
    // The first means that the node has not been visited;
    // The second means that the node is a parent of the current node
    // The third means that the node is a brother of the current node
    std::unordered_map<std::string, bool> visited;
    std::function<void(const std::string&)> dfs = [&](const std::string& cur) {
        visited[cur] = false;
        if (rules_.count(cur)) {
            // we enumerate every possible rhs of the current symbol
            for (const auto& rhs : rules_[cur].all_values()) {
                // make a copy of visited
                auto visited_backup = visited;
                std::vector<std::string> children;
                boost::split(children, rhs, boost::is_any_of(" "));
                for (const auto& c : children) {
                    if (!visited.count(c)) {
                        dfs(c);
                    } else {
                        CHECK(visited[c])
                            << "There is an infinite recursion in the grammar";
                    }
                }
                visited = visited_backup;  // backtracking
            }
        }
        visited[cur] = true;
    };
    dfs(start_symbol_);
}

void TeacherSentenceTemplate::bind(const std::string& lhs,
                                   const std::string& rhs) {
    CHECK(is_symbol(lhs)) << lhs << " must be a symbol";
    CHECK(rules_.count(lhs)) << "Invalid left hand side: " << lhs;
    rules_[lhs].bind(rhs);
}

std::string TeacherSentenceTemplate::instantiate(bool held_out,
                                                 int num_games,
                                                 int curriculum_games) {
    std::string sentence =
        instantiate(start_symbol_, num_games, curriculum_games);
    if (held_out && has_held_out_pattern(sentence)) {
        sentence = "";
    }
    unbind_all();
    return sentence;
}

void TeacherSentenceTemplate::unbind_all() {
    for (auto& r : rules_) {
        r.second.unbind();
    }
}

std::string TeacherSentenceTemplate::instantiate(
    const std::string& current_template, int num_games, int curriculum_games) {
    std::string ret_str = "";
    if (is_symbol(current_template)) {
        // first check that the symbol is in rules_
        CHECK(rules_.count(current_template)) << "symbol missing: "
                                              << current_template;
        auto& rhs = rules_[current_template];
        // then check if the symbol is bound
        if (rhs.bound()) {
            ret_str =
                instantiate(rhs.bound_value(), num_games, curriculum_games);
        } else {  // sample a string
            // rhs is already sorted wrt word number
            ret_str = instantiate(rhs.sample(num_games, curriculum_games),
                                  num_games,
                                  curriculum_games);
        }
    } else {
        std::vector<std::string> words;
        boost::split(words, current_template, boost::is_any_of(" "));
        if (words.size() == 1U) {  // leaf node not being a symbol
            return words[0];
        }
        for (size_t i = 0; i < words.size() - 1; i++) {
            ret_str += instantiate(words[i], num_games, curriculum_games) + " ";
        }
        ret_str += instantiate(words.back(), num_games, curriculum_games);
    }
    return ret_str;
}

void TeacherSentenceTemplate::instantiate_all_sentences(
    std::vector<std::string>& all_sent) {
    // generate all possible sentences from start_symbol_ (currently support
    // only
    // bound case)
    all_sent.push_back(start_symbol_);
    instantiate_all(all_sent);
}

void TeacherSentenceTemplate::instantiate_all(
    std::vector<std::string>& all_sent) {
    // generate all possible sentences (currently support only bound case)
    bool any_symbol_flag = false;  // is there any symbol across all templates
    std::vector<std::string> all_sent_new;
    for (auto it = all_sent.begin(); it < all_sent.end(); it++) {
        std::string current_template = *it;
        std::vector<std::string> words;
        boost::split(words, current_template, boost::is_any_of(" "));

        std::string ret_str = "";
        bool symbol_flag =
            false;  // whether a symbol has been encountered already
        for (size_t i = 0; i < words.size(); i++) {
            if (!is_symbol(words[i]) || symbol_flag) {
                if (i == words.size() - 1) {
                    all_sent_new.push_back(ret_str + words[i]);
                } else {
                    ret_str += words[i] + " ";
                }
            } else {
                symbol_flag = true;
                auto& rhs = rules_[words[i]];
                if (rhs.bound()) {
                    if (i == words.size() - 1) {
                        all_sent_new.push_back(ret_str + rhs.bound_value());
                    } else {
                        ret_str += rhs.bound_value() + " ";
                    }
                } else {
                    std::string rest_sent = "";
                    for (size_t j = i + 1; j < words.size(); j++) {
                        if (j == words.size() - 1) {
                            rest_sent += words[j];
                        } else {
                            rest_sent += words[j] + " ";
                        }
                    }
                    for (const auto& t : rhs.all_values()) {
                        if (i == words.size() - 1) {
                            all_sent_new.push_back(ret_str + t);
                        } else {
                            all_sent_new.push_back(t + " " + rest_sent);
                        }
                    }
                    break;
                }
            }
        }
        any_symbol_flag |= symbol_flag;
    }
    all_sent = all_sent_new;
    if (!any_symbol_flag) {
        return;
    } else {
        instantiate_all(all_sent);
    }
}

size_t TeacherSentenceTemplate::total_possible_sentences() {
    return total_possible_sentences(start_symbol_);
}

size_t TeacherSentenceTemplate::total_possible_sentences(
    const std::string& current_template) {
    size_t total = 0;
    if (is_symbol(current_template)) {
        // first check that the symbol is in rules_
        CHECK(rules_.count(current_template)) << "invalid symbol: "
                                              << current_template;
        auto& rhs = rules_[current_template];
        for (const auto& t : rhs.all_values()) {
            total += total_possible_sentences(t);
        }
    } else {
        std::vector<std::string> words;
        boost::split(words, current_template, boost::is_any_of(" "));
        if (words.size() == 1U) {  // leaf node not being a symbol
            return 1;
        }
        total = 1;
        for (const auto& w : words) {
            total *= total_possible_sentences(w);
        }
    }
    CHECK_GT(total, 0) << current_template;
    return total;
}

bool TeacherSentenceTemplate::has_held_out_pattern(
    const std::string& sentence) {
    std::vector<std::string> words;
    boost::split(words, sentence, boost::is_any_of(" "));
    for (const auto& h : held_out_) {
        std::vector<std::string> tokens;
        boost::split(tokens, h, boost::is_any_of(" "));
        bool flag = true;
        for (const auto& t : tokens) {
            if (std::count(words.begin(), words.end(), t) == 0) {
                flag = false;
                break;
            }
        }
        if (flag) {
            return true;
        }
    }
    return false;
}

}  // namespace simulator
