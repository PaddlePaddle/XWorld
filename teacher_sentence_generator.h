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
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include "simulator_entity.h"

namespace simulator {

/**
 * This header file describes an implementation of a grammar
 * The grammer consists of a list of rules
 * Each rule has a left hand side and a right hand side, where
 * the symbol of lhs can be instantiated as the rhs
 * The rhs might be a collection of selections
 * If the rhs is required to be bound, then before the rule can be applied,
 * the user must manually select one output for rhs within the collection;
 * otherwise an output will be selected randomly from rhs
 *
 * The whole sentence generation process is an instantiation process of
 * a sentence template according to the grammar, recursively, until there is
 * no non-terminal symbols in the string that is being expanded
 *
 * For example, given a starting symbol "$S", and a set of rules
 * $S -> $O is in the $P
 * $O -> apple, pear
 * $P -> west, east
 * If the rhs of $O and $P is not required to be bound, then every time
 * the user calls instantiate(), one of the following four sentences
 *   - apple is in the west
 *   - apple is in the east
 *   - pear is in the west
 *   - pear is in the east
 * will be generated with prob of 0.25
 * If the rhs of $O is bound to "apple", then only two sentences will be
 * possibly generated:
 *   - apple is in the west
 *   - apple is in the east
 * each with prob of 0.5
 **/

// This class implements the Right Hand Side of a rule
class SenTempRHS {
  public:
    // Do not remove this since SenTempRule will create an default object if key
    // does not exist, when using [] operator
    SenTempRHS() {}

    // If must_bound=true, then during instantiation bound() must be true
    SenTempRHS(const std::vector<std::string>& rhs,
               bool must_bound = false)
            : rand_(0,1), must_bound_(must_bound), rhs_(rhs) {
        sort_by_num_words();
        rhs_backup_ = rhs_;
    }

    std::string sample(int num_games, int curriculum_games);

    std::vector<std::string> all_values() { return rhs_; }

    // return the value that is bound to the rhs
    std::string bound_value();

    // return if the rhs has been bound or not
    bool bound() { return rhs_.size() == 1; }

    // bind the rhs to a string r
    // r must be one of the possible values of rhs
    // the binding is automatically dropped after each instantiation
    void bind(const std::string& r);

    void unbind();

  private:
    int curriculum_number(int low, int high, int num_games, int curriculum_games);

    void sort_by_num_words();

    std::uniform_real_distribution<double> rand_;
    bool must_bound_;
    std::vector<std::string> rhs_;
    std::vector<std::string> rhs_backup_;
};

// This map contains all the rules of a generator
// The lhs is just a string
// The rhs is a SenTempRHS object
typedef std::unordered_map<std::string, SenTempRHS> SenTempRule;

class TeacherSentenceTemplate {
  public:
    // "held_out" contains sentence patterns that are banned for the teacher
    // Different task groups have different "held_out"
    TeacherSentenceTemplate(std::vector<std::string> held_out = std::vector<std::string>(),
                            std::string start_symbol = "$S")
            : start_symbol_(start_symbol), held_out_(held_out) {
    }

    void bind(const std::string& lhs, const std::string& rhs);

    void unbind_all();

    void add_rule(const std::string& lhs, const std::vector<std::string>& rhs, bool bound = false);

    void check_rules();

    void clear_rules() { rules_.clear(); }

    const std::string& start_symbol() const { return start_symbol_; }

    // the beginning of instantiation
    // generate a sentence by instantiation from the start_symbol_
    // An instantiation is a process of applying rules to a template string repetitively
    // until all terminals (plain sentence)
    std::string instantiate(bool held_out = false, int num_games = 0, int curriculum_games = 0);

    void instantiate_all_sentences(std::vector<std::string>& all_sent);

    void instantiate_all(std::vector<std::string>& all_sent);

    // return the number of all possible sentences
    size_t total_possible_sentences();

  protected:
    const std::string start_symbol_;

  private:
    // symbol -> string
    // This function generates a sentence from the template given a set of bindings
    // If a symbol is not bound, then it will sample a string, either randomly or
    // according to the curriculum
    std::string instantiate(const std::string& current_template,
                            int num_games, int curriculum_games);
    // detects if there is any loop in the grammar
    void check_infinite_recursion();

    size_t total_possible_sentences(const std::string& current_template);

    bool has_held_out_pattern(const std::string& sentence);

    bool is_symbol(const std::string& str) {
        return str[0] == '$' && str.find(' ') == std::string::npos;
    }

    int curriculum_number(int low, int high, int num_games, int curriculum_games);

    // each mapping is a template rule
    // symbol -> sentences
    SenTempRule rules_;
    std::vector<std::string> held_out_;
};

typedef std::shared_ptr<TeacherSentenceTemplate> SentenceTemplatePtr;

} // namespace simulator
