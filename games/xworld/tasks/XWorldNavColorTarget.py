import random
from xworld_task import XWorldTask

class XWorldNavColorTarget(XWorldTask):
    def __init__(self, env):
        super(XWorldNavColorTarget, self).__init__(env)

    def idle(self):
        goals = self._get_colored_goals()
        agent, _, _ = self._get_agent()
        targets = [g for g in goals if self._reachable(agent.loc, g.loc)]
        if targets:
            sel_goal = random.choice(targets)
            self._record_target(sel_goal.loc);
            self._bind("S -> start")
            self._bind("O -> '" + sel_goal.name + "'")
            self._bind("C -> '%s'" % sel_goal.color)
            return ["simple_navigation_reward", 0.0, self._generate()]
        else:
            return ["idle", 0, ""]

    def get_stage_names(self):
        """
        return all the stage names; does not have to be in order
        """
        return ["idle", "simple_navigation_reward"]

    def _define_grammar(self):
        all_colors = self._get_all_colors_as_rhs()
        all_goal_names = self._get_all_goal_names_as_rhs()
        grammar_str = """
        S --> start | finish | timeup
        start -> I1 | I2 | I3 | I4 | I5 | I6 | I7
        finish -> 'Well' 'done' '!'
        timeup -> 'Time' 'up' '.'
        I1 -> A G 'please' '.'
        I2 -> 'Please' A G '.'
        I3 -> A G '.'
        I4 -> G 'is' 'your' D '.'
        I5 -> G 'is' 'the' D '.'
        I6 -> Y A G '?'
        I7 -> G '.'
        A -> 'go' 'to' | 'navigate' 'to' | 'reach' | 'move' 'to'
        Y -> 'Could' 'you' 'please' | 'Can' 'you' | 'Will' 'you'
        D -> 'destination' | 'target' | 'goal'
        G -> C O
        C --> %s
        O --> %s
        """ % (all_colors, all_goal_names)
        return grammar_str, "S"
