import random
from xworld_task import XWorldTask
from py_gflags import get_flag

class XWorldRecBetweenToObject(XWorldTask):
    def __init__(self, env):
        super(XWorldRecBetweenToObject, self).__init__(env)

    def idle(self):
        if random.uniform(0, 1) < 0.9:  ## tell goal
            rec_goal = True
        else:
            rec_goal = False

        goal_pairs = self._get_between_pair_goals()
        triplets = []
        for g1, g2 in goal_pairs:
            flag = False
            for g in self._get_goals():
                if g.loc == (g1.loc[0] + 1, g1.loc[1]):
                    flag = True
                    if rec_goal:
                        triplets.append((g1.name, g2.name, g.name))
            if not rec_goal and not flag:
                triplets.append((g1.name, g2.name, "nothing"))

        if triplets:
            g1_name, g2_name, name = random.choice(triplets)
            self._bind("S -> question")
            if random.uniform(0, 1) < 0.5:
                g1_name, g2_name = g2_name, g1_name
            self._bind("Z -> '%s'" % g1_name)
            self._bind("T -> '%s'" % g2_name)
            sentence = self._generate()
            if get_flag("task_mode") == "arxiv_lang_acquisition":
                return ["idle", 0, sentence + " " + name]
            else:
                self._record_answer(name)
                return ["simple_recognition_reward", 0, sentence]

        return ["idle", 0, ""]

    def get_stage_names(self):
        return ["idle", "simple_recognition_reward", "conversation_wrapup"]

    def _define_grammar(self):
        all_goal_names = self._get_all_goal_names_as_rhs()
        grammar_str = """
        S --> question | answer
        question -> Q1 | Q2 | Q3 | Q4 | Q5 | Q6 | Q7 | Q8 | Q9 | Q10 | Q11
        Q1 -> G 'what' '?'
        Q2 -> 'What' O G '?'
        Q3 -> 'Name' 'of' 'the' O G '?'
        Q4 -> 'The' O G '?'
        Q5 -> 'What' 'is' G '?'
        Q6 -> 'What' 'is' 'the' O G '?'
        Q7 -> 'What' 'is' G '?'
        Q8 -> 'Say' 'the' O G '.'
        Q9 -> 'Identify' 'the' O G '.'
        Q10 -> 'Tell' 'the' 'name' 'of' 'the' O 'which' 'is' G '.'
        Q11 -> 'The' O G 'is' '?'
        O -> 'object' | 'thing' | 'block' | 'grid'
        G -> 'between' Z 'and' T
        Z --> %s
        T --> %s
        """ % (all_goal_names, all_goal_names)
        return grammar_str, "S"
