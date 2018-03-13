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

from context_free_grammar import CFG
from py_gflags import get_flag
from maze2d import bfs, print_env

class XWorldTask(object):
    ## some static class variables
    ## that shoule be shared by all derived classes
    time_penalty = -0.1
    correct_reward = 1.0
    wrong_reward = -1.0
    failed_action_penalty = -0.2

    ## the window size for recording the performance
    performance_window_size = 200

    def __init__(self, env):
        ## define all the spatial relations
        self.directions = {
            (1, 0) : "east",
            (-1, 0) : "west",
            (0, 1) : "south",
            (0, -1) : "north",
            (1, 1) : "southeast",
            (1, -1) : "northeast",
            (-1, 1) : "southwest",
            (-1, -1) : "northwest"
        }
        self.env = env
        self.event = ""
        self.success_seq = []
        self.num_successes = 0
        self.num_failures = 0
        self.reset()
        self.cfg = CFG(*self._define_grammar())

    ################ internal functions ####################
    def _define_grammar(self):
        """
        The derived class can override this function to define a grammar
        for the teacher. To do so, the function should return a tuple of a grammar
        string and a start symbol.
        By default the grammar will be empty and the teacher generates
        a empty sentence each time step.
        """
        return "", ""

    def _get_all_directions(self):
        """
        Return all the spatial-relation words in xworld.
        """
        return self.directions.values()

    def _get_all_colors(self):
        """
        Return all colors defined in xworld.
        """
        return self.env.get_all_colors()

    def _get_direction(self, l1, l2):
        """
        Get the direciton of l2 wrt l1. For example, if the result is 'X'
        then 'l2 is in the X of l1'.
        When obtaining the direction, we use (l2 - l1)
        """
        diff = (l2[0] - l1[0], l2[1] - l1[1])
        if not diff in self.directions:
            return ""
        else:
            return self.directions[diff]

    def __record_result(self, res):
        self.success_seq.append(res)
        if len(self.success_seq) > XWorldTask.performance_window_size:
            self.success_seq.pop(0)
        self._record_env_usage()

    def _record_success(self):
        self.__record_result(1)
        self.num_successes += 1

    def _record_failure(self):
        self.__record_result(0)
        self.num_failures += 1

    def _record_env_usage(self):
        self.env.record_environment_usage(
            self.__class__.__name__, self.success_seq)

    def _record_answer(self, answer):
        """
        Record the answer for later evaluation and rewarding
        """
        self.answer = answer

    def _record_target(self, target):
        """
        Record the target for later evaluation and rewarding
        """
        self.target = target

    def _record_event(self, event, next=False):
        """
        Record an event at every time step if necessary
        Every event has a lifespan of only one time step
        """
        if not next:
            self.event = event
        else:
            self.prev_event = event

    ############# public APIs #############
    def reset(self):
        self.steps_in_cur_task = 0
        self.target = (-1, -1)
        self.answer = ""

    def get_event(self):
        """
        Return the triggered event at the current time step
        Automatically reset the event to empty after getting
        """
        ret = self.event
        self.event = ""
        return ret

    def obtain_performance(self):
        return (self.num_successes, self.num_failures)

    def print_grammar(self):
        self.cfg.show()

    def total_possible_sentences(self):
        """
        Return the number of total possible sentences *given* the current
        bindings.
        """
        return self.cfg.total_possible_sentences()

    def conversation_wrapup(self):
        """
        This dummpy stage simply adds an additional time step after the
        conversation is over, which enables the agent to learn language model
        from teacher's last sentence.
        """
        self._record_event(self.prev_event)
        self.prev_event = None
        return ["idle", 0, ""]

    def simple_recognition_reward(self):
        """
        A simple recognition reward stage. It gives reward according to the
        single-word answer. The agent has to exactly match the answer word.
        """
        _, agent_sent, _ = self._get_agent()
        self._bind("S -> answer")
        self._set_production_rule("answer -> '%s'" % self.answer)
        sentence = self._generate()
        if agent_sent == self.answer:
            reward = XWorldTask.correct_reward / 2
            self._record_success()
            self._record_event("correct_reply", next=True)
        else:
            reward = XWorldTask.wrong_reward / 2
            self._record_failure()
            self._record_event("wrong_reply", next=True)
        return ["conversation_wrapup", reward, sentence]

    def simple_navigation_reward(self):
        """
        A simple navigation reward stage. If the agent reaches the correct
        goal, it gets a positive reward. If it steps on an incorrect goal,
        it gets a negative reward. There will also be penalties if it has
        a failed action. The task returns to 'idle' stage when the correct
        goal is reached or the time is up for this task. The time-up setting
        is only for "one_channel" mode.
        """
        reward = XWorldTask.time_penalty

        agent, _, action_successful = self._get_agent()
        if not action_successful:
            reward += XWorldTask.failed_action_penalty

        goal_locs = [g.loc for g in self._get_goals()]
        next_stage = "simple_navigation_reward"
        sentence = ""

        self.steps_in_cur_task += 1
        h, w = self.env.get_max_dims()
        if get_flag("task_mode") == "one_channel" \
           and self.steps_in_cur_task >= h*w / 2:
            self.steps_in_cur_task = 0
            self._record_failure()
            self._bind("S -> timeup")
            next_stage = "idle"
            sentence = self._generate()
        elif agent.loc == self.target:
            self.steps_in_cur_task = 0
            self._record_success()
            self._record_event("correct_goal")
            reward += XWorldTask.correct_reward
            self._bind("S -> finish")
            next_stage = "idle"
            sentence = self._generate()
        elif agent.loc in goal_locs:
            reward += XWorldTask.wrong_reward

        return [next_stage, reward, sentence]

    ############ functions that wrap self.env and self.cfg #############
    def _list_of_strs_to_rhs(self, strs):
        """
        Converting a list of strings to a string as the right hand side of a
        production rule. For example, if the list is ['foo', 'bar'], then the
        conversion result is "'foo' | 'bar'"
        """
        return "|".join(["'" + s + "'" for s in strs])

    def _get_all_goal_names_as_rhs(self):
        """
        Get all possible goal names in xworld and convert the list to a rhs string
        """
        return self._list_of_strs_to_rhs(self.env.get_all_possible_names("goal"))

    def _get_all_directions_as_rhs(self):
        """
        Get all possible spatial-relation words in xworld and convert the list to a rhs string
        """
        return self._list_of_strs_to_rhs(self._get_all_directions())

    def _get_all_colors_as_rhs(self):
        """
        Get all possible color words in xworld and convert the list to a rhs string
        """
        return self._list_of_strs_to_rhs(self._get_all_colors())

    def _get_goals(self):
        """
        Get all the goals on the current map
        """
        return self.env.get_goals()

    def _get_blocks(self):
        """
        Get all the blocks on the current map
        """
        return self.env.get_blocks()

    def _get_entities(self):
        """
        Get all the entities on the current map
        """
        return self.env.get_entities()

    def _color_defined(self, color):
        """
        Decide if a color is defined or not. An undefined color is "na"
        """
        return color != "na"

    def _get_colored_goals(self):
        """
        Get all the goals on the current map that have defined colors
        """
        return [g for g in self._get_goals() if self._color_defined(g.color)]

    def _get_agent(self):
        """
        Get the agent information; see XWorldEnv.get_agent()
        """
        return self.env.get_agent()

    def _set_entity_inst(self, e):
        self.env.set_entity_inst(e)

    def _delete_entity(self, e):
        self.env.delete_entity(e)

    def _set_property(self, e, property_value_dict):
        self.env.set_property(e, property_value_dict)

    def __within_boundary(self, loc):
        """
        Determine if a location is out of boundary of the map
        """
        h, w = self.env.get_dims()
        x, y = loc
        return y >= 0 and y < h and x >= 0 and x < w

    def _get_surrounding_goals(self, refer_loc=None):
        """
        Given a reference location, return all goals in its 3x3 neighborhood
        """
        goals = self._get_goals()
        if refer_loc is None:
            agent, _, _ = self._get_agent()
            refer_loc = agent.loc
        ret = []
        for g in goals:
            d = self._get_direction(refer_loc, g.loc)
            if d != "":
                ret.append((g, d))
        return ret

    def _get_surrounding_empty_grids(self, refer_loc=None):
        """
        Given a reference location, return all empty grids in its 3x3 neighborhood
        """
        if refer_loc is None:
            agent, _, _ = self._get_agent()
            refer_loc = agent.loc
        surrounding = [((refer_loc[0] + d[0], refer_loc[1] + d[1]), s) \
                       for d, s in self.directions.iteritems()]
        entities = [e.loc for e in self._get_entities()]
        empty_grids = [l for l in surrounding if not l[0] in entities]
        return empty_grids

    def _get_between_pair_goals(self):
        """
        Return all pairs of goals that are separated horizontally by exactly
        one grid that is not a wall block
        """
        goals = self._get_goals()
        blocks = [b.loc for b in self._get_blocks()]
        return [(g1, g2) for g1 in goals for g2 in goals \
                 if (2, 0) == (g2.loc[0] - g1.loc[0], g2.loc[1] - g1.loc[1]) \
                and not (g1.loc[0] + 1, g1.loc[1]) in blocks]

    def _reachable(self, start, end):
        """
        Use BFS to determine that if location 'end' can be reached from location 'start'
        The obstacles are the wall blocks on the current map.
        """
        if start == end:
            return True
        obstacles = set([b.loc for b in self._get_blocks()])
        assert not start in obstacles, "start pos should not be in obstacles"
        Y, X = self.env.get_dims()
        return (bfs(start, end, X, Y, obstacles) is not None)

    def _bind(self, binding_str):
        """
        Bind a production rule; see CFG.bind() for details
        """
        self.cfg.bind(binding_str)

    def _generate(self):
        """
        Generate a sentence according the grammar and current bindings;
        see CFG.generate() for details
        """
        return self.cfg.generate()

    def _generate_all(self):
        """
        Generate all possible sentences given current bindings; in this case,
        the bindings are not necessary.
        See CFG.generate_all() for details.
        """
        return self.cfg.generate_all()

    def _set_production_rule(self, rule):
        """
        Set a new production rule or overwrite an existing one.
        See CFG.set_production_rule() for details.
        """
        self.cfg.set_production_rule(rule)
