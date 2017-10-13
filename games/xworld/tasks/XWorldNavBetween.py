import random
from xworld_task import XWorldTask

class XWorldNavBetween(XWorldTask):
    def __init__(self, env):
        super(XWorldNavBetween, self).__init__(env)

    def idle(self):
        goal_pairs = self._get_between_pair_goals()
        agent, _, _ = self._get_agent()
        targets = [(g1, g2, (g1.loc[0] + 1, g1.loc[1])) for g1, g2 in goal_pairs \
                   if self._reachable(agent.loc, (g1.loc[0] + 1, g1.loc[1]))]
        if targets:
            g1, g2, middle = random.choice(targets)
            self._record_target(middle);
            self._bind("S -> start")
            if random.uniform(0, 1) < 0.5:
                g1, g2 = g2, g1
            self._bind("O -> '" + g1.name + "'")
            self._bind("T -> '%s'" % g2.name)
            return ["simple_navigation_reward", 0.0, self._generate()]
        else:
            return ["idle", 0, ""]

    def get_stage_names(self):
        """
        return all the stage names; does not have to be in order
        """
        return ["idle", "simple_navigation_reward"]

    def _define_grammar(self):
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
        G -> 'the' 'grid' 'between' O 'and' T
        O --> %s
        T --> %s
        A -> 'go' 'to' | 'navigate' 'to' | 'reach' | 'move' 'to'
        Y -> 'Could' 'you' 'please' | 'Can' 'you' | 'Will' 'you'
        D -> 'destination' | 'target' | 'goal'
        """ % (all_goal_names, all_goal_names)
        return grammar_str, "S"
