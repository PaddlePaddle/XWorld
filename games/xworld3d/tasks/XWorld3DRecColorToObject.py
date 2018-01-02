import random
from xworld3d_task import XWorld3DTask

class XWorld3DRecColorToObject(XWorld3DTask):
    def __init__(self, env):
        super(XWorld3DRecColorToObject, self).__init__(env)

    def idle(self):
        goals = self._get_colored_goals()
        colors = [g.color for g in goals]
        ## filter out the goals that have unique colors on the map
        uni_color_goals = [g for g in goals if colors.count(g.color) == 1]
        if uni_color_goals:
            goal = random.choice(uni_color_goals)
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
