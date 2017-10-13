import random
from xworld_task import XWorldTask
from py_gflags import get_flag

class XWorldRecDirectionAndObjectToObject(XWorldTask):
    def __init__(self, env):
        super(XWorldRecDirectionAndObjectToObject, self).__init__(env)

    def idle(self):
        goals = self._get_goals()
        found = False
        if random.uniform(0, 1) < 0.9:  ## tell goal
            goals = [(g, random.choice(self._get_surrounding_goals(g.loc))) \
                for g in goals if self._get_surrounding_goals(g.loc)]
            if goals:
                goal, goal2 = random.choice(goals)
                grid = goal2[0].name
                direction = goal2[1]
                found = True
        else: ## tell empty grid
            empty_grids = [(g, random.choice(self._get_surrounding_empty_grids(g.loc))) \
                for g in goals if self._get_surrounding_empty_grids(g.loc)]
            if empty_grids:
                goal, goal2 = random.choice(empty_grids)
                grid = "nothing"
                direction = goal2[1]
                found = True

        if found:
            self._bind("S -> question")
            self._bind("D -> '%s'" % direction)
            self._bind("Z -> '%s'" % goal.name)
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
        all_goal_names = self._get_all_goal_names_as_rhs()
        grammar_str = """
        S --> question | answer
        question -> Q1 | Q2 | Q3 | Q4 | Q5 | Q6 | Q7 | Q8 | Q9 | Q10 | Q11
        Q1 -> G 'what' '?'
        Q2 -> 'What' O 'in' G '?'
        Q3 -> 'Name' 'of' 'the' O 'in' G '?'
        Q4 -> 'The' O 'in' G '?'
        Q5 -> 'What' 'is' 'in' G '?'
        Q6 -> 'What' 'is' 'the' O 'in' G '?'
        Q7 -> 'What' 'is' G '?'
        Q8 -> 'Say' 'the' O 'in' G '.'
        Q9 -> 'Identify' 'the' O 'in' G '.'
        Q10 -> 'Tell' 'the' 'name' 'of' 'the' O 'which' 'is' G '.'
        Q11 -> 'The' O 'in' G 'is' '?'
        O -> 'object' | 'thing' | 'block' | 'grid'
        G -> D R Z
        D --> %s
        R -> 'to' | 'of' | 'near' | 'by'
        Z --> %s
        """ % (all_directions, all_goal_names)
        return grammar_str, "S"
