import random
from xworld3d_task import XWorld3DTask

class XWorld3DRecObjectToColor(XWorld3DTask):
    def __init__(self, env):
        super(XWorld3DRecObjectToColor, self).__init__(env)

    def idle(self):
        goals = self._get_colored_goals()
        if goals:
            goal = random.choice(goals)
            self._bind("S -> question")
            self._bind("G -> '%s'" % goal.name)
            sentence = self._generate()
            if get_flag("task_mode") == "arxiv_lang_acquisition":
                # supervised; forward compatible
                return ["idle", 0, sentence + " " + goal.color]
            else:
                self._record_answer(goal.color) # record answer for reward stage
                return ["simple_recognition_reward", 0, sentence] # RL
        else:
            return ["idle", 0, ""]

    def get_stage_names(self):
        return ["idle", "simple_recognition_reward", "conversation_wrapup"]

    def _define_grammar(self):
        all_goal_names = self._get_all_goal_names_as_rhs()
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
        G --> %s
        """ % all_goal_names
        return grammar_str, "S"
