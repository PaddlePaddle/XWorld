"""
Copyright (c) 2017 Baidu Inc. All Rights Reserved.

 Licensed under the Apache License, Version 2.0 (the "License");
 you may not use this file except in compliance with the License.
 You may obtain a copy of the License at

     http://www.apache.org/licenses/LICENSE-2.0

 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an "AS IS" BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License.
"""

import copy
import random
import pprint
from operator import mul

def is_terminal(symbol):
    """
    A terminal is a single-quoted string literal; otherwise it's a nontermimal
    """
    return len(symbol) >= 2 and symbol[0] == "'" and symbol[-1] == "'"

class RHS:
    def __init__(self, items=None, must_bound=False):
        assert isinstance(items, list)
        self.must_bound = must_bound
        self.items = items
        self.items_backup = copy.deepcopy(items)

    def all_values(self):
        """
        Return the all values given the current bindings
        """
        return self.items

    def value(self):
        """
        Return a value.
        If must_bound is True, them self.items must only contain one value;
        otherwise a value is randomly sampled from the items list.
        """
        assert not self.must_bound or len(self.items) == 1, \
            "RHS must be bound first"
        return random.choice(self.items)

    def unbind(self):
        """
        Restore the items list.
        """
        self.items = copy.deepcopy(self.items_backup)

    def set_must_bound(self):
        """
        If called, then an RHS object with multiple values must be bound
        before value() can be called.
        """
        self.must_bound = True

    def bind(self, item):
        """
        Narrow down the items list to only one element
        """
        # print self.items
        # print item
        assert item in self.items, "Invalid RHS item: " + item
        self.items = [item]

class CFG:
    def __init__(self, string, start_symbol='S'):
        self.grammar_str = string
        rules = [r for r in string.splitlines() if not r.strip() == ""]
        self.start_symbol = start_symbol
        self.productions = {}
        for r in rules:
            self.set_production_rule(r)

    def __parse_rule(self, rule_str):
        """
        A production rule must be in the form "X -> Y1 | Y2 | ...", where X
        is a non-terminal lhs symbol and the rhs contains a list of choices.
        Y could be a terminal symbol (indicated by single quotes around), a non-terminal
        symbol (literal), or a mixture of the two.
        The user can use '-->' instead of '->' to require that the rhs must be
        bound before a sentence can be generated.

        The output is a tuple (lhs, rhs, must_bound)
        """
        separator = "->"
        if "-->" in rule_str:   ## '-->' indicates a must_bound
            separator = "-->"
        strs = rule_str.split(separator)
        assert len(strs) == 2, "invalid format of the rule: " + rule_str
        lhs = strs[0].strip()
        assert not is_terminal(lhs), "LHS cannot be a terminal:" + lhs
        rhs_items = [i.strip() for i in strs[1].split("|")]
        return lhs, rhs_items, separator=="-->"

    def show(self):
        """
        Print the grammar as a string
        """
        pprint.pprint(self.grammar_str)

    def bind(self, binding_str):
        """
        Bind a production rule. The binding_str should be in the form "X -> Y".
        This will bind Y to X.
        """
        lhs, rhs_items, _ = self.__parse_rule(binding_str)
        assert lhs in self.productions, "No such production rule: " + lhs
        assert len(rhs_items) == 1, "ambiguous binding: " + binding_str
        self.productions[lhs].bind(rhs_items[0])

    def __unbind_all(self):
        """
        Unbind the rhs of all production rules
        """
        for lhs, rhs in self.productions.iteritems():
            rhs.unbind()

    def set_start_symbol(self, start_symbol):
        assert not is_terminal(start_symbol), "start_symbol cannot be a terminal!"
        self.start_symbol = start_symbol

    def set_production_rule(self, string):
        """
        Add a new rule or modify an existing rule.
        """
        lhs, rhs_items, must_bound = self.__parse_rule(string)
        self.productions[lhs] = RHS(rhs_items)
        if must_bound:
            self.productions[lhs].set_must_bound()
        self.check_recursion()

    def check_recursion(self):
        """
        Each node i has three status: not in visited, visited[i]=false,
        visited[i]=true
        The first means that the node has not been visited;
        The second means that the node is a parent of the current node
        The third means that the node is a brother of the current node
        """
        def _dfs(symbol, visited):
            visited[symbol] = False
            if symbol in self.productions:
                for item in self.productions[symbol].all_values():
                    visited_backup = copy.deepcopy(visited)
                    rhs_symbols = item.split()
                    for s in rhs_symbols:
                        if not s in visited:
                            _dfs(s, visited)
                        else:
                            assert visited[s], "Recursion in the grammar!"
                    visited = visited_backup
            visited[symbol] = True
        _dfs(self.start_symbol, {})

    def clear_grammar(self):
        self.productions = {}
        self.grammar_str = ""
        self.start_symbol = ""

    def generate(self, start_symbol=None):
        """
        Generate a sentence given the grammar and bindings.
        If for a production rule, the rhs has multiple unbound values, then a value is
        randomly sampled. This will raise errors if a rhs is not bound but its must_bound
        is True.
        """
        if start_symbol == None:
            start_symbol = self.start_symbol
        assert not is_terminal(start_symbol), "start_symbol must be a nonterminal"

        def _generate(symbol):
            if is_terminal(symbol):
                return symbol[1:-1]  # remove the single quotes
            else:
                assert symbol in self.productions, "Ungrounded nonterminal: " + symbol
                rhs = self.productions[symbol].value()
                rhs_symbols = rhs.split()
                return " ".join([_generate(s) for s in rhs_symbols])

        sentence = _generate(start_symbol)
        self.__unbind_all()
        return sentence

    def generate_all(self, start_symbol=None):
        """
        Generate all possible sentences given the grammar and the bindings.
        This will use the existing bindings, but will ignore must_bound.
        """
        if start_symbol == None:
            start_symbol = self.start_symbol
        assert not is_terminal(start_symbol), "start_symbol must be a nonterminal"

        def _generate(symbols):
            assert isinstance(symbols, list)
            if len(symbols) == 0:
                yield []
            else:
                for frag1 in _generate_one(symbols[0]):
                    for frag2 in _generate(symbols[1:]):
                        yield frag1 + frag2

        def _generate_one(symbol):
            assert isinstance(symbol, str)
            if is_terminal(symbol):
                yield [symbol[1:-1]]
            else:
                assert symbol in self.productions, "Ungrounded nonterminal: " + symbol
                for rhs in self.productions[symbol].all_values():
                    for frag in _generate(rhs.split()):
                        yield frag

        sentences = [" ".join(words) for words in list(_generate_one(start_symbol))]
        self.__unbind_all()
        return sentences

    def total_possible_sentences(self, start_symbol=None):
        """
        Count the total number of possible sentences for the grammar.
        The total number will be affected by the existing bindings,
        but will ignore must_bound.
        """
        if not self.productions:
            return 0

        if start_symbol == None:
            start_symbol = self.start_symbol
        assert not is_terminal(start_symbol), "start_symbol must be a nonterminal"

        def _count(symbol):
            if is_terminal(symbol):
                return 1
            else:
                ## ignore ungrounded nonterminal
                if not symbol in self.productions:
                    return 0
                total = 0
                for rhs in self.productions[symbol].all_values():
                    total += reduce(mul, map(lambda i: _count(i), rhs.split()), 1)
                return total

        num = _count(start_symbol)
        self.__unbind_all()
        return num

## test
if __name__ == "__main__":
    cfg = CFG("""
    S -> 'we' |   N 'you'
    N --> 'us' | O
    O -> 'dead' W
    W -> 'sdfxc' | 'xcvxc' 'sdfscx' 'xcvx'
    """)
    print(cfg.total_possible_sentences())

    cfg.bind("N -> 'us'")
    print(cfg.generate_all())   ## this will unbind
    cfg.bind("S -> N 'you'")
    try:
        print(cfg.generate())   ## this will raise error because of must_bound
        assert False, "This shouldn't happen"
    except:
        pass
