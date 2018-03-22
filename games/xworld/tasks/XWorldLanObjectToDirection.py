import random
from xworld_task import XWorldTask

class XWorldLanObjectToDirection(XWorldTask):
    def __init__(self, env):
        super(XWorldLanObjectToDirection, self).__init__(env)

    def idle(self):
        """
        Start a task
        """
        agent, _, _ = self._get_agent()
        goals = self._get_goals()

        assert len(goals) > 0, "there is no goal on the map!"
        sel_goal = random.choice(goals)
        direction = self._get_direction(agent.loc, sel_goal.loc)

        ## first generate all candidate answers
        self._bind("S -> answer")
        self._bind("G -> '%s'" % sel_goal.name)
        self._bind("D -> '%s'" % direction)
        self.answers = self._generate_all()

        ## then generate the question
        self._bind("S -> question")
        self._bind("G -> '%s'" % sel_goal.name)
        return ["reward", 0.0, self._generate()]

    def reward(self):
        """
        Giving reward to the agent
        """
        _, agent_sent, _ = self._get_agent()
        self._set_production_rule(
            "R -> " + " ".join(["'" + w + "'" for w in random.choice(self.answers).split()]))
        self._bind("S -> reply")
        if agent_sent in self.answers:
            self._bind("Y -> 'Yes'")
            reward = 1.0
            self._record_success()
            self._record_event("correct_reply", next=True)
        else:
            self._bind("Y -> 'No'")
            reward = -1.0
            self._record_failure()
            self._record_event("wrong_reply", next=True)
        return ["conversation_wrapup", reward, self._generate()]

    def get_stage_names(self):
        """
        return all the stage names; does not have to be in order
        """
        return ["idle", "reward", "conversation_wrapup"]

    def _define_grammar(self):
        all_goal_names = self._get_all_goal_names_as_rhs()
        all_directions = self._get_all_directions_as_rhs()
        grammar_str = """
        S -> question | answer | reply
        question -> 'Where' 'is' G '?'
        answer -> A1 | A2
        reply -> R | Y R
        A1 -> 'On' 'the' D 'is' G
        A2 -> G 'is' 'on' 'the' D
        D --> %s
        G --> %s
        Y --> 'Yes' | 'No'
        """ % (all_directions, all_goal_names)
        return grammar_str, "S"
