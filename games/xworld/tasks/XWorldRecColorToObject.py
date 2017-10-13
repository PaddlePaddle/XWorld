import random
from xworld_task import XWorldTask
from py_gflags import get_flag

class XWorldRecColorToObject(XWorldTask):
    def __init__(self, env):
        super(XWorldRecColorToObject, self).__init__(env)

    def idle(self):
        goals = self._get_colored_goals()
        if goals:
            goal = random.choice(goals)
            self._bind("S -> question")
            self._bind("C -> '%s'" % goal.color)
            sentence = self._generate()
            if get_flag("task_mode") == "arxiv_lang_acquisition":
                return ["idle", 0, sentence + " " + goal.name]
            else:
                self._record_answer(goal.name)
                return ["simple_recognition_reward", 0, sentence]

        return ["idle", 0, ""]

    def get_stage_names(self):
        return ["idle", "simple_recognition_reward", "conversation_wrapup"]

    def _define_grammar(self):
        all_colors = self._get_all_colors_as_rhs()
        grammar_str = """
        S --> question | answer
        question -> Q1 | Q2 | Q3 | Q4 | Q5 | Q6 | Q7 | Q8 | Q9 | Q10 | Q11
        Q1 -> C 'what' '?'
        Q2 -> 'What' O 'in' C '?'
        Q3 -> 'Name' 'of' 'the' O 'in' C '?'
        Q4 -> 'The' O 'in' C '?'
        Q5 -> 'What' 'is' 'in' C '?'
        Q6 -> 'What' 'is' 'the' O 'in' C '?'
        Q7 -> 'What' 'is' C '?'
        Q8 -> 'Say' 'the' O 'in' C '.'
        Q9 -> 'Identify' 'the' O 'in' C '.'
        Q10 -> 'Tell' 'the' 'name' 'of' 'the' O 'which' 'is' C '.'
        Q11 -> 'The' O 'in' C 'is' '?'
        O -> 'object' | 'thing' | 'block' | 'grid'
        C --> %s
        """ % all_colors
        return grammar_str, "S"
