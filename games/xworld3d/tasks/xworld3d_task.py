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
from math import cos
from math import acos
from math import sin
from math import asin
from math import copysign
from math import sqrt
import random
import itertools
from context_free_grammar import CFG
from py_gflags import get_flag
from maze2d import bfs, flood_fill

class XWorld3DTask(object):
    ## some static class variables
    ## that shoule be shared by all derived classes
    time_penalty = -0.05
    correct_reward = 10.0
    wrong_reward = -10.0
    collision_penalty = 0.00

    ## If the agent is near the target, we increase the penalty
    ## to prevent a trivial TargetSide solution
    target_side_penalty = time_penalty * 3

    failed_action_penalty = -0.1
#    max_steps = 500

    navigation_max_steps_factor = 10

    PI = 3.1415926
    PI_2 = PI / 2    # 90
    PI_4 = PI / 4    # 45
    PI_8 = PI / 8    # 22.5
    PI_12 = PI / 12  # 15

    ## the window size for recording the performance
    performance_window_size = 200

    def __init__(self, env):
        ## define all the spatial relations
        self.directions = {
            ( self.PI_2-self.PI_8,  self.PI_2+self.PI_8)    : "right",
            (-self.PI_2-self.PI_8, -self.PI_2+self.PI_8)    : "left",
            (          -self.PI_8,            self.PI_8)    : "front",
            # two intervals of "behind" cannot be merged, so we need to specify
            # them separately
            (   self.PI-self.PI_8,              self.PI)    : "behind",
            (            -self.PI,   -self.PI+self.PI_8)    : "behind",
            (           self.PI_8,  self.PI_4+self.PI_8)    : "front-right",
            ( self.PI_2+self.PI_8,    self.PI-self.PI_8)    : "behind-right",
            (-self.PI_2+self.PI_8,           -self.PI_8)    : "front-left",
            (  -self.PI+self.PI_8, -self.PI_2-self.PI_8)    : "behind-left"
        }
#        self.distance_threshold = get_flag("x3_reaching_distance")
        self.orientation_threshold = self.PI_4
        self.env = env
        self.event = ""
        self.success_seq = []
        self.num_successes = 0
        self.num_failures = 0
        self.reset()
        self.cfg = CFG(*self._define_grammar())
        self.sentence = ""

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

    def _get_direction_and_distance(self, p1, p2, p1_yaw=None):
        """
        Get the direciton of p2 wrt p1's yaw, and the distance from p1 to p2.
        Return:
        theta:      relative angle from p2 to p1 wrt p1's yaw
        dist:       distance from p1 to p2
        direction:  name of the direction
        """
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        dist = sqrt(dx ** 2 + dy ** 2)
        if p1_yaw is None:
            return dist
        v1 = (cos(p1_yaw), sin(p1_yaw))
        v2 = (dx / dist, dy / dist)
        # theta is the angle from p2 to p1 wrt p1's orientation
        cos_theta = max(-1, min(1, v1[0]*v2[0] + v1[1]*v2[1]))
        sin_theta = max(-1, min(1, v1[1]*v2[0] - v1[0]*v2[1]))
        theta = acos(cos_theta) * copysign(1, asin(sin_theta))
        direction = ""
        for r in self.directions.keys():
            if (theta >= r[0] and theta < r[1]):
                direction = self.directions[r]
        return theta, dist, direction

    def _get_distance(self, p1, p2):
        return sqrt((p2[0]-p1[0]) ** 2 + (p2[1]-p1[1]) ** 2)

    def __record_result(self, res):
        self.success_seq.append(res)
        if len(self.success_seq) > XWorld3DTask.performance_window_size:
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
        Record the navigation target
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

    def _parse_collision_event(self, events):
        """
        Extract collision events from game event message
        """
        collision_events = [e for e in events.strip().split('\n') if e.startswith("collision")]
        hits = set()
        for e in collision_events:
            o = e.split(':')[1].split('|')
            hits.update(o)

        return hits

    def __within_boundary(self, loc):
        """
        Determine if a location is out of boundary of the map
        """
        h, w = self.env.get_dims()
        x, y = loc
        return y >= 0 and y < h and x >= 0 and x < w

    def _get_surrounding_goals(self, distance_threshold=1.5, refer=None):
        """
        Given a reference, return all goals within a circular neighborhood
        with a radius of distance_threshold (excluding the refer itself)
        Note that distance_threshold is inclusive
        """
        goals = self._get_goals()
        if refer is None:
            refer, _, _ = self._get_agent()
            refer = refer.loc
        ret = []
        for g in goals:
            if g.loc == refer:
                continue
            dist = self._get_direction_and_distance(refer, g.loc)
            if dist < distance_threshold + 1e-3:
                ret.append(g)
        return ret

    def _get_surrounding_empty_grids(self, distance_threshold=1.5, refer=None):
        """
        Given a reference location, return all empty grids in its neighborhood
        (excluding the refer itself)
        """
        if refer is None:
            refer, _, _ = self._get_agent()
            refer = refer.loc
        ret = []
        for g in self.env.get_available_grids():
            if g == refer: ## refer itself might be empty
                continue
            dist = self._get_direction_and_distance(refer, g)
            if dist < distance_threshold + 1e-3:
                ret.append(g)
        return ret

    def _get_p_tiles(self):
        """
        Return all the "pair" tiles (length=2) on the map.
        """
        Y, X = self.env.get_dims()
        p_tiles = []
        available = set(self.env.get_available_grids())

        def test_pair(p1, p2):
            if p1 in available and p2 in available:
                around_p2 = self._get_surrounding_empty_grids(
                    distance_threshold=1.0, refer=p2)
                if set(around_p2) - set([p1]):
                    p_tiles.append((p1, p2))
                around_p1 = self._get_surrounding_empty_grids(
                    distance_threshold=1.0, refer=p1)
                if set(around_p1) - set([p2]):
                    p_tiles.append((p2, p1))

        for y in range(Y):
            for x in range(X):
                test_pair((x, y, 0), (x + 1, y, 0))
                test_pair((x, y, 0), (x, y + 1, 0))
                test_pair((x, y, 0), (x + 1, y + 1, 0))

        return p_tiles

    def _get_t_tiles(self):
        """
        Return all the empty T-shaped tiles on the map. For our purpose, only return
        the two ends of the longer segment (length=3).
        """
        Y, X = self.env.get_dims()
        t_tiles = []
        available = set(self.env.get_available_grids())
        for y in range(Y):
            for x in range(X):
                if (x, y, 0) in available:
                    ## horizontal
                    if (x - 1, y, 0) in available \
                       and (x + 1, y, 0) in available \
                       and ((x, y - 1, 0) in available \
                            or (x, y + 1, 0) in available):
                        t_tiles.append(((x - 1, y, 0), (x + 1, y, 0)))
                    ## vertical
                    if (x, y - 1, 0) in available \
                       and (x, y + 1, 0) in available \
                       and ((x - 1, y, 0) in available \
                            or (x + 1, y, 0) in available):
                        t_tiles.append(((x, y - 1, 0), (x, y + 1, 0)))
        return t_tiles

    def _get_s_tiles(self):
        """
        Return all the square tiles (area=4) no the map.
        """
        Y, X = self.env.get_dims()
        s_tiles = []
        available = set(self.env.get_available_grids())
        neighbors = [(0, 0), (1, 0), (0, 1), (1, 1)]
        for y in range(Y):
            for x in range(X):
                flag = True
                for n in neighbors:
                    nx = x + n[0]
                    ny = y + n[1]
                    if not (nx, ny, 0) in available:
                        flag = False
                        break
                if flag: # the whole square is empty
                    s_tiles.append(((x, y, 0), (x + 1, y, 0)))
                    s_tiles.append(((x, y + 1, 0), (x + 1, y + 1, 0)))
                    s_tiles.append(((x, y, 0), (x + 1, y + 1, 0)))
                    s_tiles.append(((x + 1, y, 0), (x, y + 1, 0)))
        return s_tiles

    def _get_l_tiles(self):
        """
        Return all the l-shaped tiles (length=3) no the map
        """
        Y, X = self.env.get_dims()
        l_tiles = []
        available = set(self.env.get_available_grids())

        def test_triple(p1, p2, p3):
            if p1 in available:
                if p2 in available and p3 in available:
                    l_tiles.append((p1, p2))
                    l_tiles.append((p2, p3))

        for y in range(Y):
            for x in range(X):
                test_triple((x, y, 0), (x, y + 1, 0), (x, y + 2, 0))
                test_triple((x, y, 0), (x + 1, y, 0), (x + 2, y, 0))
                test_triple((x, y, 0), (x + 1, y + 1, 0), (x + 2, y + 2, 0))

        return l_tiles

    def _middle_loc(self, l1, l2, fl=False):
        half = 2.0 if fl else 2
        return ((l1[0]+l2[0])/half, (l1[1]+l2[1])/half, (l1[2]+l2[2])/half)

    def _reachable(self, start, end):
        """
        Use BFS to determine that if location 'end' can be reached from location 'start'
        The obstacles are the wall blocks and goals on the current map.
        """
        if start == end:
            return True
        blocks = [b.loc for b in self._get_blocks()]
        goals = [g.loc for g in self._get_goals()]
        if end in goals: # end could be occupied by a goal
            goals.remove(end)
        obstacles = blocks + goals
        assert not start in obstacles, "start pos should not be in obstacles"
        Y, X = self.env.get_dims()
        return (bfs(start, end, X, Y, obstacles) is not None)

    def _propagate_agent(self, seeds, inclusive=False):
        """
        Given a list of goals, propagate them through the maze to find possible agent
        positions
        """
        obstacles = [b.loc for b in self._get_blocks()]
        goals = [g.loc for g in self._get_goals()]
        Y, X = self.env.get_dims()
        filled = flood_fill(seeds, obstacles + goals, X, Y)
        if inclusive:
            filled += seeds
        return filled

    ############# public APIs #############
    def reset(self):
        self.steps_in_cur_task = 0
        self.target = None
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
        ### prev_event should have been recorded by self._record_event(); otherwise crash
        self._record_event(self.prev_event)
        self.prev_event = None
        return ["idle", 0, ""]

    def simple_recognition_reward(self):
        """
        A simple recognition reward stage. It gives reward according to the
        single-word answer. The agent has to exactly match the answer word.
        """
        reward = XWorld3DTask.time_penalty
        _, agent_sent, _ = self._get_agent()

        collisions = self._parse_collision_event(self.env.game_event)
        if collisions:
            reward += XWorld3DTask.collision_penalty

        self.steps_in_cur_task += 1

        session_end = True
        if self.steps_in_cur_task >= XWorld3DTask.max_steps / 2:
            self.steps_in_cur_task = 0
            self._record_failure()
            self._record_event("time_up", next=True)
        elif agent_sent != "-":  # if the agent answers
            if agent_sent == self.answer:
                reward += XWorld3DTask.correct_reward
                self._record_success()
                self._record_event("correct_reply", next=True)
            else:
                reward += XWorld3DTask.wrong_reward
                self._record_failure()
                self._record_event("wrong_reply", next=True)
        else:
            session_end = False

        if session_end:
            self._bind("S -> answer")
            self._set_production_rule("answer -> '%s'" % self.answer)
            self.sentence = self._generate()
            next_stage = "conversation_wrapup"
        else:
            self.sentence = ""
            next_stage = "simple_recognition_reward"

        return [next_stage, reward, self.sentence]

    def _reach_object(self, agent, yaw, object):
        collisions = self._parse_collision_event(self.env.game_event)
        theta, _, _ = self._get_direction_and_distance(agent, object.loc, yaw)
        return abs(theta) < self.orientation_threshold and object.id in collisions

    def _successful_goal(self, reward):
        self._record_success()
        self._record_event("correct_goal")
        reward += XWorld3DTask.correct_reward
        self._bind("S -> correct")
        self.sentence = self._generate()
        return reward

    def _failed_goal(self, reward):
        self._record_failure()
        self._record_event("wrong_goal")
        reward += XWorld3DTask.wrong_reward
        self._bind("S -> wrong")
        self.sentence = self._generate()
        return reward

    def _time_reward(self):
        reward = XWorld3DTask.time_penalty
        self.steps_in_cur_task += 1
        h, w = self.env.get_dims()
        if self.steps_in_cur_task >= h * w * XWorld3DTask.navigation_max_steps_factor:
            self._record_failure()
            self._bind("S -> timeup")
            self.sentence = self._generate()
            self._record_event("time_up")
            return (reward, True)
        return (reward, False)

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
        Get the agent information; see XWorld3DEnv.get_agent()
        """
        return self.env.get_agent()

    def _set_entity_inst(self, e):
        self.env.set_entity_inst(e)

    def _move_entity(self, e, loc):
        self.env.move_entity(e, loc)

    def _delete_entity(self, e):
        self.env.delete_entity(e)

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
