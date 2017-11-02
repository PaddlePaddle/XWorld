import random
from xworld_task import XWorldTask
from py_gflags import get_flag

class XWorldRecBetweenToColor(XWorldTask):
    def __init__(self, env):
        super(XWorldRecBetweenToColor, self).__init__(env)

    def idle(self):
        goal_pairs = self._get_between_pair_goals()
        color_goals = self._get_colored_goals()
        goal_rows = []
        for g in color_goals:
            for g1, g2 in goal_pairs:
                if g.loc == (g1.loc[0] + 1, g1.loc[1]):
                    goal_rows.append((g1.name, g2.name, g.color))

        if goal_rows:
            g1_name, g2_name, color = random.choice(goal_rows)
            self._bind("S -> question")
            if random.uniform(0, 1) < 0.5:
                g1_name, g2_name = g2_name, g1_name
            self._bind("O -> '%s'" % g1_name)
            self._bind("T -> '%s'" % g2_name)
            sentence = self._generate()
            if get_flag("task_mode") == "arxiv_lang_acquisition":
                return ["idle", 0, sentence + " " + color]
            else:
                self._record_answer(color)
                return ["simple_recognition_reward", 0, sentence]

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
        G -> 'the' 'object' 'between' O 'and' T
        O --> %s
        T --> %s
        """ % (all_goal_names, all_goal_names)
        return grammar_str, "S"
