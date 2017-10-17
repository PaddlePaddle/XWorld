from context_free_grammar import CFG
from py_gflags import get_flag

class XWorldTask(object):
    ## some static class variables
    ## that shoule be shared by all derived classes
    time_penalty = -0.1
    correct_reward = 1.0
    wrong_reward = -1.0
    failed_action_penalty = -0.2

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
        self.num_successes = 0
        self.num_failures = 0
        self.reset(True)
        self.cfg = CFG(*self._define_grammar())

    ################ internal functions ####################
    def _define_grammar(self):
        """
        The derived class can override this function to define a grammar
        for the teacher. To do so, the function should return a grammar
        string and a start symbol.
        By default the grammar will be empty.
        """
        return "", ""

    def _get_all_directions(self):
        return self.directions.values()

    def _get_all_colors(self):
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

    def _record_success(self):
        self.num_successes += 1

    def _record_failure(self):
        self.num_failures += 1

    def _record_answer(self, answer):
        self.answer = answer

    def _record_target(self, target):
        self.target = target

    def _record_event(self, event):
        """
        Record an event at every time step if necessary
        Every event has a lifespan of only one time step
        """
        self.event = event

    ############# public APIs #############
    def reset(self, is_idle):
        self.steps_in_cur_task = 0
        self.target = (-1, -1)
        self.answer = ""
        if not is_idle:
            self._record_failure();

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
        return self.cfg.total_possible_sentences()

    def conversation_wrapup(self):
        """
        This dummpy stage simply adds an additional time step after the
        conversation is over, which enables the agent to learn language model
        from teacher's last sentence.
        """
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
            self._record_event("correct_reply")
        else:
            reward = XWorldTask.wrong_reward / 2
            self._record_failure()
            self._record_event("wrong_reply")
        return ["conversation_wrapup", reward, sentence]

    def simple_navigation_reward(self):
        reward = XWorldTask.time_penalty

        agent, _, action_successful = self._get_agent()
        if not action_successful:
            reward += XWorldTask.failed_action_penalty

        goal_locs = [g.loc for g in self._get_goals()]
        next_stage = "simple_navigation_reward"
        sentence = ""

        self.steps_in_cur_task += 1
        h, w = self.env.get_dims()
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
        return "|".join(["'" + s + "'" for s in strs])

    def _get_all_goal_names_as_rhs(self):
        return self._list_of_strs_to_rhs(self.env.get_all_possible_names("goal"))

    def _get_all_directions_as_rhs(self):
        return self._list_of_strs_to_rhs(self._get_all_directions())

    def _get_all_colors_as_rhs(self):
        return self._list_of_strs_to_rhs(self._get_all_colors())

    def _get_goals(self):
        return self.env.get_goals()

    def _get_blocks(self):
        return self.env.get_blocks()

    def _get_entities(self):
        return self.env.get_entities()

    def _color_defined(self, color):
        return color != "na"

    def _get_colored_goals(self):
        return [g for g in self._get_goals() if self._color_defined(g.color)]

    def _get_agent(self):
        return self.env.get_agent()

    def __within_boundary(self, loc):
        h, w = self.env.get_dims()
        x, y = loc
        return y >= 0 and y < h and x >= 0 and x < w

    def _get_surrounding_goals(self, refer_loc=None):
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
        if refer_loc is None:
            agent, _, _ = self._get_agent()
            refer_loc = agent.loc
        surrounding = [((refer_loc[0] + d[0], refer_loc[1] + d[1]), s) \
                       for d, s in self.directions.iteritems()]
        entities = [e.loc for e in self._get_entities()]
        empty_grids = [l for l in surrounding if not l[0] in entities]
        return empty_grids

    def _get_between_pair_goals(self):
        goals = self._get_goals()
        blocks = [b.loc for b in self._get_blocks()]
        return [(g1, g2) for g1 in goals for g2 in goals \
                 if (2, 0) == (g2.loc[0] - g1.loc[0], g2.loc[1] - g1.loc[1]) \
                and not (g1.loc[0] + 1, g1.loc[1]) in blocks]

    def _reachable(self, start, to):
        obstacles = set([b.loc for b in self._get_blocks()])
        visited = {start}
        ## TODO: use collections.deque if the map size is huge
        que = [start]
        while que:
            cur = que.pop(0)
            if cur == to:
                return True
            for d in self.directions.keys():
                next = (cur[0] + d[0], cur[1] + d[1])
                if self.__within_boundary(next) \
                   and (not next in obstacles) \
                   and (not next in visited):
                    visited.add(next)
                    que.append(next)
        return False

    def _bind(self, binding_str):
        self.cfg.bind(binding_str)

    def _generate(self):
        return self.cfg.generate()

    def _generate_all(self):
        return self.cfg.generate_all()

    def _set_production_rule(self, rule):
        self.cfg.set_production_rule(rule)
