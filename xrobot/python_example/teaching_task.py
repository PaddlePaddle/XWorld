import cv2
import numpy as np
import random
from math import cos
from math import acos
from math import sin
from math import asin
from math import copysign
from math import sqrt
from context_free_grammar import CFG

class XWorld3DTask(object):

	time_penalty = -0.01
	correct_reward = 1.0
	wrong_reward = -1.0
	collision_penalty = 0.0

	failed_action_penalty = -0.1
	navigation_max_steps = 10000

	performance_window_size = 200

	PI = 3.1415926
	PI_2 = PI / 2    # 90
	PI_4 = PI / 4    # 45
	PI_8 = PI / 8    # 22.5
	PI_12 = PI / 12  # 15

	def __init__(self, env):

		self.directions = {
			( self.PI_2-self.PI_8,  self.PI_2+self.PI_8)    : "right",
			(-self.PI_2-self.PI_8, -self.PI_2+self.PI_8)    : "left",
			(          -self.PI_8,            self.PI_8)    : "front",
			# two intervals of "behind" cannot be merged, so we need to specify
			# them separately
			(   self.PI-self.PI_8,              self.PI)    : "behind",
			(            -self.PI,   -self.PI+self.PI_8)    : "behind",
			(           self.PI_8,  self.PI_4+self.PI_8)    : "front-right",
			( self.PI_2+self.PI_8,    self.PI-self.PI_8)    : "behind-right",
			(-self.PI_2+self.PI_8,           -self.PI_8)    : "front-left",
			(  -self.PI+self.PI_8, -self.PI_2-self.PI_8)    : "behind-left"
		}
		self.orientation_threshold = self.PI_4
		self.distance_threshold = 1.0

		self.env = env
		self.event = ""
		self.cfg = CFG(*self._define_grammar())
		self.sentence = ""
		self.steps_in_cur_task = 0
		self.success_seq = []
		self.num_successes = 0
		self.num_failures = 0
		self.success_steps = 0


	def reset(self):
		self.steps_in_cur_task = 0
		self.answer = ""
		self.env.Clear()

	def _get_direction_and_distance(self, p1, p2, p1_yaw=None):
		dx = p2[0] - p1[0]
		dy = p2[2] - p1[2]
		dist = sqrt(dx ** 2 + dy ** 2)
		if p1_yaw is None:
			return dist
		if dist == 0:
			return 0, 0, ""
		else:
			v1 = (cos(p1_yaw), sin(p1_yaw))
			v2 = (dx / dist, dy / dist)
			# theta is the angle from p2 to p1 wrt p1's orientation
			cos_theta = max(-1, min(1, v1[0]*v2[0] + v1[1]*v2[1]))
			sin_theta = max(-1, min(1, v1[1]*v2[0] - v1[0]*v2[1]))
			theta = acos(cos_theta) * copysign(1, asin(sin_theta))
			direction = ""
			for r in self.directions.keys():
				if (theta >= r[0] and theta < r[1]):
					direction = self.directions[r]
			return theta, dist, direction

	def __record_result(self, res):
		self.success_seq.append(res)
		if len(self.success_seq) > XWorld3DTask.performance_window_size:
			self.success_seq.pop(0)
		self._record_env_usage()

	def _record_success(self):
		self.__record_result(1)
		self.num_successes += 1
		self.success_steps += self.steps_in_cur_task

	def _record_failure(self):
		self.__record_result(0)
		self.num_failures += 1

	def _record_env_usage(self):
		print ""

	def _time_reward(self):
		reward = XWorld3DTask.time_penalty
		self.steps_in_cur_task += 1
		if self.steps_in_cur_task >= self.navigation_max_steps:
			self._record_failure()
			self._bind("S -> timeup")
			self.sentence = self._generate()
			self._record_event("time_up")
			return (reward, True)
		return (reward, False)

	def _failed_goal(self, reward):
		self._record_failure()
		self._record_event("wrong_goal")
		reward += XWorld3DTask.wrong_reward
		self._bind("S -> wrong")
		self.sentence = self._generate()
		return reward

	def _successful_goal(self, reward):
		self._record_success()
		self._record_event("correct_goal")
		reward += XWorld3DTask.correct_reward
		self._bind("S -> correct")
		self.sentence = self._generate()
		return reward

	# TODO
	def _reach_object(self, agent, yaw, object):
		theta, dist, _ = self._get_direction_and_distance(agent, object, yaw)
		return (abs(theta) < self.orientation_threshold) and abs(dist) < self.distance_threshold

	def _record_answer(self, answer):
		self.answer = answer

	def _record_event(self, event, next=False):
		if not next:
			self.event = event
		else:
			self.prev_event = event

	def _define_grammar(self):
		return "", ""

	def _bind(self, binding_str):
		self.cfg.bind(binding_str)
		return "",""

	def _generate(self):
		return self.cfg.generate()

	def _generate_all(self):
		return self.cfg.generate_all()

	def _set_production_rule(self, rule):
		self.cfg.set_production_rule(rule)

	def _list_of_strs_to_rhs(self, strs):
		return "|".join(["'" + s + "'" for s in strs])

	def _get_all_goal_names_as_rhs(self):
		return self._list_of_strs_to_rhs(self._get_goals())

	def _get_all_goal_names_as_rhs(self, strs):
		return self._list_of_strs_to_rhs(strs)

	def obtain_performance(self):
		return (self.num_successes, self.num_failures, self.success_steps)

	def _get_goals(self):
		return self.env.GetGoals()

	def print_grammar(self):
		self.cfg.show()

	def total_possible_sentences(self):
		return self.cfg.total_possible_sentences()

	def conversation_wrapup(self):
		self._record_event(self.prev_event)
		self.prev_event = None
		return ["idle", 0, ""]

	def terminal(self):
		return ["terminal", 0, ""]

	def get_stages(self):
		return "idle"


class Task(object):
	def __init__(self, task_name, task):
		self.task_name = task_name
		self.stages = task.get_stages()
		self.currt_stage = "idle"
		self.currt_reward = 0.0
		self.currt_sentence = ""
		self.task = task;
		self.env = None

	def run_stage(self):
		
		res = self.stages[self.current_stage]()

		self.current_stage = res[0]
		self.currt_reward = res[1]
		self.currt_sentence = res[2]

	def register_stage(self, stage_name, func):
		self.stages[stage_name] = func

	def reset(self):
		self.current_stage = "idle"

	def get_current_stage(self):
		return self.current_stage

	def get_task_name(self):
		return self.task_name

	def is_idle(self):
		return self.current_stage == "idle"

class TaskGroup(object):
	def __init__(self, group_name):
		self.group_name = group_name
		self.busy_task = None
		self.task_weights = []
		self.task_list = []
		self.reward = 0.0

	def add_task(self, task_name, task_stage, weight = 0):
		task = Task(task_name, task_stage)
		
		self.task_list.append(task)

		if len(self.task_weights) == 0:
			self.task_weights.append(weight)
		else:
			self.task_weights.append(self.task_weights[len(self.task_weights) - 1] + weight)

	def is_idle(self):
		if self.busy_task == None:
			return True
		else:
			if self.busy_task.is_idle():
				self.busy_task = None
				return True
			else:
				return False

	def reset(self):
		self.busy_task = None

	def run_stage(self):
		if self.busy_task == None or self.busy_task.is_idle():
			rand = random.randint(0, len(self.task_list) - 1)
			self.busy_task = self.task_list[rand]
			self.busy_task.reset()
		if self.busy_task != None:
			self.busy_task.run_stage()

	def get_group_name(self):
		return self.group_name

	def get_sentence(self):
		if self.busy_task != None:
			return self.busy_task.currt_sentence