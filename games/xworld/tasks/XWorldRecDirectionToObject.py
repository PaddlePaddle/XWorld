import random
from xworld_task import XWorldTask
from py_gflags import get_flag

class XWorldRecDirectionToObject(XWorldTask):
    def __init__(self, env):
        super(XWorldRecDirectionToObject, self).__init__(env)

    def idle(self):
        found = False
        if random.uniform(0, 1) < 0.9:  ## tell goal
            goals = self._get_surrounding_goals()
            if goals:
                grid, direction = random.choice(goals)
                grid = grid.name
                found = True
        else: ## tell empty grid
            empty_grids = self._get_surrounding_empty_grids()
            if empty_grids:
                _, direction = random.choice(empty_grids)
                grid = "nothing"
                found = True

        if found:
            self._bind("S -> question")
            self._bind("D -> '%s'" % direction)
            sentence = self._generate()
            if get_flag("task_mode") == "arxiv_lang_acquisition":
                return ["idle", 0, sentence + " " + grid]
            else:
                self._record_answer(grid)
                return ["simple_recognition_reward", 0, sentence]

        return ["idle", 0, ""]

    def get_stage_names(self):
        return ["idle", "simple_recognition_reward", "conversation_wrapup"]

    def _define_grammar(self):
        all_directions = self._get_all_directions_as_rhs()
        grammar_str = """
        S --> question | answer
        question -> Q1 | Q2 | Q3 | Q4 | Q5 | Q6 | Q7 | Q8 | Q9 | Q10 | Q11
        Q1 -> D 'what' '?'
        Q2 -> 'What' O 'in' D '?'
        Q3 -> 'Name' 'of' 'the' O 'in' D '?'
        Q4 -> 'The' O 'in' D '?'
        Q5 -> 'What' 'is' 'in' D '?'
        Q6 -> 'What' 'is' 'the' O 'in' D '?'
        Q7 -> 'What' 'is' D '?'
        Q8 -> 'Say' 'the' O 'in' D '.'
        Q9 -> 'Identify' 'the' O 'in' D '.'
        Q10 -> 'Tell' 'the' 'name' 'of' 'the' O 'which' 'is' D '.'
        Q11 -> 'The' O 'in' D 'is' '?'
        O -> 'object' | 'thing' | 'block' | 'grid'
        D --> %s
        """ % all_directions
        return grammar_str, "S"
