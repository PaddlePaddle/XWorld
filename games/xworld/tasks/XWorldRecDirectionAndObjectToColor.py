import random
from xworld_task import XWorldTask
from py_gflags import get_flag

class XWorldRecDirectionAndObjectToColor(XWorldTask):
    def __init__(self, env):
        super(XWorldRecDirectionAndObjectToColor, self).__init__(env)

    def idle(self):
        goals = self._get_colored_goals()
        goals = [(random.choice(self._get_surrounding_goals(g.loc)), g) \
                 for g in goals if self._get_surrounding_goals(g.loc)]
        if goals:
            goal, color_goal = random.choice(goals)
            goal, _ = goal
            direction = self._get_direction(goal.loc, color_goal.loc)
            self._bind("S -> question")
            self._bind("D -> '%s'" % direction)
            self._bind("O -> '%s'" % goal.name)
            sentence = self._generate()
            if get_flag("task_mode") == "arxiv_lang_acquisition":
                # supervised; forward compatible
                return ["idle", 0, sentence + " " + color_goal.color]
            else:
                self._record_answer(color_goal.color) # record answer for reward stage
                return ["simple_recognition_reward", 0, sentence] # RL
        else:
            return ["idle", 0, ""]

    def get_stage_names(self):
        return ["idle", "simple_recognition_reward", "conversation_wrapup"]

    def _define_grammar(self):
        all_goal_names = self._get_all_goal_names_as_rhs()
        all_directions = self._get_all_directions_as_rhs()
        grammar_str = """
        S --> question | answer
        question -> Q1 | Q2 | Q3 | Q4 | Q5 | Q6 | Q7
        Q1 -> G C '?'
        Q2 -> C 'of' G '?'
        Q3 -> 'Tell' 'the' C 'of' G '.'
        Q4 -> 'What' C 'does' 'the' G 'have' '?'
        Q5 -> 'What' 'is' 'the' C 'of' G '?'
        Q6 -> 'Identify' 'the' C 'of' G '.'
        Q7 -> 'Say' 'the' C 'of' G '.'
        G -> D R O
        C -> 'color' | 'property'
        D --> %s
        R -> 'to' | 'of' | 'near' | 'by'
        O --> %s
        """ % (all_directions, all_goal_names)
        return grammar_str, "S"
