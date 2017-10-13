import random
from xworld_task import XWorldTask
from py_gflags import get_flag

class XWorldRecObjectToDirection(XWorldTask):
    def __init__(self, env):
        super(XWorldRecObjectToDirection, self).__init__(env)

    def idle(self):
        goals = self._get_surrounding_goals()
        if goals:
            goal, direction = random.choice(goals)
            self._bind("S -> question")
            self._bind("G -> '%s'" % goal.name)
            sentence = self._generate()
            if get_flag("task_mode") == "arxiv_lang_acquisition":
                # supervised; forward compatible
                return ["idle", 0, sentence + " " + direction]
            else:
                self._record_answer(direction) # record answer for reward stage
                return ["simple_recognition_reward", 0, sentence] # RL
        else:
            return ["idle", 0, ""]

    def get_stage_names(self):
        return ["idle", "simple_recognition_reward", "conversation_wrapup"]

    def _define_grammar(self):
        all_goal_names = self._get_all_goal_names_as_rhs()
        grammar_str = """
        S --> question | answer
        question -> Q1 | Q2 | Q3 | Q4 | Q5 | Q6 | Q7 | Q8 | Q9 | Q10 | Q11 | Q12 | Q13
        Q1 -> G 'location' '?'
        Q2 -> G 'where' '?'
        Q3 -> 'Where' 'is' 'the' G '?'
        Q4 -> 'What' 'is' 'the' 'location' 'of' G '?'
        Q5 -> 'Where' 'is' G 'located' '?'
        Q6 -> 'Which' 'direction' 'is' 'the' G '?'
        Q7 -> 'Which' 'side' 'is' 'the' G 'on' 'you' '?'
        Q8 -> 'Please' 'locate' G '.'
        Q9 -> 'Find' G '.'
        Q10 -> 'The' 'location' 'of' 'the' G 'is' '.'
        Q11 -> 'Say' 'the' 'location' 'of' 'the' G '.'
        Q12 -> 'Identify' 'the' 'direction' 'of' 'the' G '.'
        Q13 -> 'Tell' 'the' 'location' 'of' 'the' G '.'
        G --> %s
        """ % all_goal_names
        return grammar_str, "S"
