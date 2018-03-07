import random
from xworld_task import XWorldTask

class XWorldDialog(XWorldTask):
    def __init__(self, env):
        super(XWorldDialog, self).__init__(env)
        self.max_steps = 7 # maximum number of steps, should be related to number of sel classes
        self.speak_correct_reward = 1;
        self.speak_incorrect_reward = -1;
        self.question_ask_reward = 0.1;
        self.nothing_said_reward = -1;
        self.reset_dialog_setting()

    def reset_dialog_setting(self):
        self.question_ratio = 0.5 # the chance of asking a question or making a statement
        self.teacher_sent_prev_ = [] # stores teacher's sentences in a session in order
        self.behavior_flags = []

    def idle(self):
        """
        Start a task
        """
        agent, _, _ = self._get_agent()
        goals = self._get_goals()

        assert len(goals) > 0, "there is no goal on the map!"
        sel_goal = random.choice(goals)

        ## first generate all candidate answers
        self._bind("S -> statement")
        #self._bind("G -> '%s'" % sel_goal.name)
        self._set_production_rule("G -> " + " ".join(["'" + sel_goal.name + "'"]))
        self.answers = self._generate_all()

        ## then generate the question
        self._bind("S -> question")
        self.questions = self._generate_all()

        sent = self.sentence_selection_with_ratio()
        self._set_production_rule("R -> " + " ".join(["'" + sent + "'"]))
        teacher_sent = self._generate_and_save([sent])

        return ["reward", 0.0, teacher_sent]

    def reward(self):
        """
        Giving reward to the agent
        """
        # get agent's sentence (response to previous sentence from teacher)
        _, agent_sent, _ = self._get_agent()
        # get teacher's sentence
        prev_sent = self._get_last_sent()
        # if the previous stage is a qa stage
        qa_stage_prev = (prev_sent == "" or prev_sent in self.questions)
        is_question_asked = agent_sent in self.questions
        is_reply_correct = agent_sent in self.answers
        is_nothing_said = agent_sent == ""
        extend_step = (is_nothing_said or is_question_asked) and \
                       qa_stage_prev
        # in this case, move to the next object for interaction
        if not extend_step:
            self.env.within_session_reinstantiation()

        goals = self._get_goals()
        sel_goal = random.choice(goals)
        # update answers
        #self._bind("G -> '%s'" % sel_goal.name)
        self._set_production_rule("G -> " + " ".join(["'" + sel_goal.name + "'"]))
        self.answers = self._generate_all()

        self.steps_in_cur_task += 1
        # decide reward and next stage
        if self.steps_in_cur_task + 1 < self.max_steps:
            if self.steps_in_cur_task > self.max_steps / 2:
                self.question_ratio = 1
            if qa_stage_prev:
                if is_question_asked:
                    # reward feedback
                    if not is_nothing_said:
                        reward = self.question_ask_reward
                    else:
                        reward = self.nothing_said_reward
                        self.behavior_flags += [False]
                    # sentence feedback (answer/statement)
                    self._bind("S -> statement")
                    #self._bind("G -> '%s'" % sel_goal.name)
                    self._set_production_rule("G -> " + " ".join(["'" + sel_goal.name + "'"]))
                    teacher_sent = self._generate_and_save()
                elif is_reply_correct:
                    # switch to more regorous criteria
                    self.behavior_flags += [True]
                    if all(self.behavior_flags):
                        self._record_success()
                    else:
                        self._record_failure()
                    self._record_event("correct_reply", next=True)
                    reward = self.speak_correct_reward
                    teacher_sent = ""
                    self.reset_dialog_setting()
                    return ["conversation_wrapup", reward, teacher_sent]
                else:
                    self.behavior_flags += [False]
                    reward = speak_incorrect_reward
                    sent = self.sentence_selection_with_ratio()
                    self._set_production_rule("R -> " + " ".join(["'" + sent + "'"]))
                    teacher_sent = self._generate_and_save([sent])
            else:
                # reward feedback for different cases
                if is_reply_correct: # repeat statement
                    reward = 0
                elif is_nothing_said:
                    reward = self.nothing_said_reward
                elif is_question_asked:
                    reward = self.speak_incorrect_reward
                else:
                    self.behavior_flags += [False]
                    reward = self.speak_incorrect_reward
                # sentence feedback
                sent = self.sentence_selection_with_ratio()
                self._set_production_rule("R -> " + " ".join(["'" + sent + "'"]))
                teacher_sent = self._generate_and_save([sent])

            return ["reward", reward, teacher_sent]
        else:
            if qa_stage_prev and is_reply_correct:
                self.behavior_flags += [True]
                self._record_event("correct_reply", next=True)
                reward= self.speak_correct_reward
            else:
                self.behavior_flags += [False]
                reward = self.speak_incorrect_reward
                self._record_event("wrong_reply", next=True)

            if all(self.behavior_flags):
                self._record_success()
            else:
                self._record_failure()
            teacher_sent = ""
            self.reset_dialog_setting()
            return ["conversation_wrapup", reward, teacher_sent]

    def sentence_selection_with_ratio(self):
        if random.randint(0, 1) > self.question_ratio: # proceed with statement
            return random.choice(self.answers)
        else:
            return random.choice(self.questions)

    def get_stage_names(self):
        """
        return all the stage names; does not have to be in order
        """
        return ["idle", "reward", "conversation_wrapup"]

    def _define_grammar(self):
        #all_directions = self._get_all_directions_as_rhs()
        #all_goal_names = self._get_all_goal_names_as_rhs()
        grammar_str = """
        S --> question | statement
        question -> E | Q
        statement-> A1 | A2 | A3 | A4 | A5 | A6 | A7 | A8
        E -> ''
        Q -> Q1 | Q2 | Q3
        Q1 -> 'what'
        Q2 -> 'what' M
        Q3 -> 'tell' 'what' N
        M ->  'is' 'it' | 'is' 'this' | 'is' 'there' | 'do' 'you' 'see' | 'can' 'you' 'see' | 'do' 'you' 'observe' | 'can' 'you' 'observe'
        N -> 'it' 'is' | 'this' 'is' | 'there' 'is' | 'you' 'see' | 'you' 'can' 'see' | 'you' 'observe' | 'you' 'can' 'observe'
        A1 -> G
        A2 -> 'it' 'is' G
        A3 -> 'this' 'is' G
        A4 -> 'there' 'is' G
        A5 -> 'i' 'see' G
        A6 -> 'i' 'observe' G
        A7 -> 'i' 'can' 'see' G
        A8 -> 'i' 'can' 'observe' G
        G  -> 'dummy'
        """
        return grammar_str, "S"

    def _generate_and_save(self, teacher_sent = []):
        """
        generate (if teacher_sent is empty)  and save the teacher's sentence
        to teacher's previous sentence pool
        """
        if not teacher_sent:
            teacher_sent = [self._generate()]
        self.teacher_sent_prev_ = self.teacher_sent_prev_ + teacher_sent
        return teacher_sent[0]

    def _get_last_sent(self):
        """
        get the sentence from teacher in the last time step
        """
        assert self.teacher_sent_prev_, "make sure the previous sentence set is non-empty"
        sent = self.teacher_sent_prev_[-1]
        return sent
