import random
from xworld_task import XWorldTask
from py_gflags import get_flag

class XWorldRecDirectionToColor(XWorldTask):
    def __init__(self, env):
        super(XWorldRecDirectionToColor, self).__init__(env)

    def idle(self):
        goals = self._get_surrounding_goals()
        goals = [g for g in goals if self._color_defined(g[0].color)]
        if goals:
            goal, direction = random.choice(goals)
            self._bind("S -> question")
            self._bind("D -> '%s'" % direction)
            sentence = self._generate()
            if get_flag("task_mode") == "arxiv_lang_acquisition":
                return ["idle", 0, sentence + " " + goal.color]
            else:
                self._record_answer(goal.color)
                return ["simple_recognition_reward", 0, sentence]

        return ["idle", 0, ""]

    def get_stage_names(self):
        return ["idle", "simple_recognition_reward", "conversation_wrapup"]

    def _define_grammar(self):
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
        C -> 'color' | 'property'
        G -> 'object' 'in' D
        D --> %s
        """ % all_directions
        return grammar_str, "S"
