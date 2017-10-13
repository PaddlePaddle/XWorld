import random
from xworld_task import XWorldTask

class XWorldNavNear(XWorldTask):
    def __init__(self, env):
        super(XWorldNavNear, self).__init__(env)

    def idle(self):
        goals = self._get_goals()
        agent, _, _ = self._get_agent()
        targets = []
        for g in goals:
            surrounding_empty_grids = self._get_surrounding_empty_grids(g.loc)
            reachable_grids = [l for l in surrounding_empty_grids if self._reachable(agent.loc, l[0])]
            if reachable_grids:
                targets.append((g, random.choice(reachable_grids)))

        if targets:
            goal, loc = random.choice(targets)
            self._record_target(loc[0])
            self._bind("S -> start")
            self._bind("O -> '" + goal.name + "'")
            self._bind("D -> '%s'" % loc[1])
            return ["simple_navigation_reward", 0.0, self._generate()]
        else:
            return ["idle", 0, ""]

    def get_stage_names(self):
        """
        return all the stage names; does not have to be in order
        """
        return ["idle", "simple_navigation_reward"]

    def _define_grammar(self):
        all_directions = self._get_all_directions_as_rhs()
        all_goal_names = self._get_all_goal_names_as_rhs()
        grammar_str = """
        S --> start | finish | timeup
        start -> I1 | I2 | I3 | I4 | I5 | I6 | I7
        finish -> 'Well' 'done' '!'
        timeup -> 'Time' 'up' '.'
        I1 -> A G 'please' '.'
        I2 -> 'Please' A G '.'
        I3 -> A G '.'
        I4 -> G 'is' 'your' dest '.'
        I5 -> G 'is' 'the' dest '.'
        I6 -> Y A G '?'
        I7 -> G '.'
        G -> D R O
        D --> %s
        R -> 'to' | 'of' | 'near' | 'by'
        A -> 'go' 'to' | 'navigate' 'to' | 'reach' | 'move' 'to'
        Y -> 'Could' 'you' 'please' | 'Can' 'you' | 'Will' 'you'
        dest -> 'destination' | 'target' | 'goal'
        O --> %s
        """ % (all_directions, all_goal_names)
        return grammar_str, "S"
