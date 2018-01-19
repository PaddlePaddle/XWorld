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
from context_free_grammar import CFG
from py_gflags import get_flag
import itertools

class XWorld3DTask(object):
    ## some static class variables
    ## that shoule be shared by all derived classes
    time_penalty = -0.05
    correct_reward = 10.0
    wrong_reward = -10.0
    collision_penalty = 0.00

    failed_action_penalty = -0.1
#    max_steps = 500

    navigation_max_steps_factor = 10

    PI = 3.1415926
    PI_8 = PI / 8
    PI_4 = PI / 4
    PI_2 = PI / 2

    # how often we should record the environment usage for the curriculum learning
    record_env_usage_period = 200

    def __init__(self, env):
        ## define all the spatial relations
        self.directions = {
            ( self.PI_2-self.PI_8,  self.PI_2+self.PI_8)    : "right",
            (-self.PI_2-self.PI_8, -self.PI_2+self.PI_8)    : "left",
            (          -self.PI_8,            self.PI_8)    : "front",
            # two intervals of "back" cannot be merged, so we need to specify
            # them separately
            (   self.PI-self.PI_8,              self.PI)    : "back",
            (            -self.PI,   -self.PI+self.PI_8)    : "back",
            (           self.PI_8,  self.PI_4+self.PI_8)    : "front-right",
            ( self.PI_2+self.PI_8,    self.PI-self.PI_8)    : "back-right",
            (-self.PI_2+self.PI_8,           -self.PI_8)    : "front-left",
            (  -self.PI+self.PI_8, -self.PI_2-self.PI_8)    : "back-left"
        }
        self.distance_threshold = get_flag("x3_reaching_distance")
        self.orientation_threshold = self.PI_4
        self.env = env
        self.event = ""
        self.num_successes = 0
        self.num_failures = 0
        self.reset()
        self.cfg = CFG(*self._define_grammar())
        self.sentence = ""
        self.failure_recorded = False

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

    def _get_direction_and_distance(self, p1, yaw, p2):
        """
        Get the direciton of p2 wrt p1's orientation, and the distance from p1
        to l2.
        Return:
        theta:      relative angle from p2 to p1 wrt p1's orientation
        dist:       distance from p1 to p2
        direction:  name of the direction
        """
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        dist = pow(pow(dx, 2) + pow(dy, 2), 0.5)
        v1 = (cos(yaw), sin(yaw))
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
        return pow(pow(p2[0]-p1[0], 2) + pow(p2[1]-p1[1], 2), 0.5)

    def _record_success(self):
        self.num_successes += 1
        self._record_env_usage()

    def _record_failure(self):
        self.num_failures += 1
        self._record_env_usage()

    def _record_env_usage(self):
        ## wait until sufficient data
        if self.num_successes + self.num_failures == XWorld3DTask.record_env_usage_period:
            ## env usage is decided by the task success rate
            self.env.record_environment_usage(
                float(self.num_successes) / XWorld3DTask.record_env_usage_period)
            self.num_successes = 0
            self.num_failures = 0

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

    def reset_performance(self):
        self.num_successes = 0
        self.num_failures = 0

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

    def simple_navigation_reward(self):
        """
        A simple navigation reward stage. If the agent reaches the correct
        goal, it gets a positive reward. If it steps on an incorrect goal,
        it gets a negative reward. There will also be penalties if it has
        a failed action. The task returns to 'idle' stage when the correct
        goal is reached or the time is up for this task. The time-up setting
        is only for "one_channel" mode.
        """
        reward = XWorld3DTask.time_penalty

        agent, _, _ = self._get_agent()

        ## even though the collided object might be the goal, we still have a penalty
        ## hopefully the huge positive reward will motivate it
        collisions = self._parse_collision_event(self.env.game_event)
        if collisions:
            reward += XWorld3DTask.collision_penalty

        next_stage = "simple_navigation_reward"
        self.sentence = ""

        def reach_object(agent, yaw, object):
            theta, _, _ = self._get_direction_and_distance(agent, yaw, object.loc)
            return abs(theta) < self.orientation_threshold and object.id in collisions

        self.steps_in_cur_task += 1
        h, w = self.env.get_dims()
        if self.steps_in_cur_task >= h * w * XWorld3DTask.navigation_max_steps_factor:
            self.steps_in_cur_task = 0
            self._record_failure()
            self._bind("S -> timeup")
            self._record_event("time_up")
            next_stage = "idle"
            self.sentence = self._generate()
            self.failure_recorded = False
        else:
            objects_reach_test = [(g.id, reach_object(agent.loc, agent.yaw, g)) \
                                  for g in self._get_goals()]
            if (self.target.id, True) in objects_reach_test:
                self.steps_in_cur_task = 0
                self._record_success()
                self._record_event("correct_goal")
                reward += XWorld3DTask.correct_reward
                self._bind("S -> correct")
                self.sentence = self._generate()
                next_stage = "idle"
                self.failure_recorded = False
            elif (not self.failure_recorded) and [t for t in objects_reach_test if t[1]]: # other objects
                reward += XWorld3DTask.wrong_reward
                self._record_failure()
                if False:
                    self.failure_recorded = True
                else:
                    self._bind("S -> wrong")
                    self.sentence = self._generate()
                    self._record_event("wrong_goal")

        return [next_stage, reward, self.sentence]

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

    def _move_entity(self, e, loc):
        self.env.move_entity(e, loc)

    def __within_boundary(self, loc):
        """
        Determine if a location is out of boundary of the map
        """
        h, w = self.env.get_dims()
        x, y = loc
        return y >= 0 and y < h and x >= 0 and x < w

    def _get_surrounding_goals(self, refer_loc=None):
        """
        Given a reference location, return all goals within a circular neighborhood
        with a radius of self.distance_threshold
        """
        goals = self._get_goals()
        agent, _, _ = self._get_agent()
        if refer_loc is None:
            refer_loc = agent.loc
        ret = []
        for g in goals:
            _, dist, dir_name = self._get_direction_and_distance(refer_loc, agent.yaw, g.loc)
            if dist < self.distance_threshold:
                ret.append((g, dir_name))
        return ret

    def _get_surrounding_empty_grids(self, refer_loc=None):
        """
        Given a reference location, return all empty grids in its 3x3 neighborhood
        """
        # TODO
        raise NotImplementedError()

    def _get_between_pair_goals(self):
        """
        Return all pairs of goals that are separated horizontally by exactly
        one grid that is not a wall block
        """
        # TODO
        raise NotImplementedError()

    def _reachable(self, start, end):
        """
        Use BFS to determine that if location 'end' can be reached from location 'start'
        The obstacles are the wall blocks and goals on the current map.
        """
        blocks = [b.loc for b in self._get_blocks()]
        goals = [g.loc for g in self._get_goals()]
        obstacles = blocks + goals
        assert not start in obstacles, "start pos should not be in obstacles"
        if end in obstacles: # end could be occupied by a goal
            obstacles.remove(end)
        Y, X = self.env.get_dims()
        return (self.env.bfs(start, end, X, Y, obstacles) is not None)

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
