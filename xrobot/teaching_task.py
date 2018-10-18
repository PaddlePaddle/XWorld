import random

class TaskInterface(object):
	def get_stages():
		return "idle"

class Task(object):
	def __init__(self, task_name, task):
		self.task_name = task_name
		self.stages = task.get_stages()
		self.currt_stage = "idle"
		self.task = task;
		self.env = None

	def run_stage(self):
		self.current_stage = self.stages[self.current_stage]()

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

	def add_task(self, task_name, task_stage, weight = 0):
		task = Task(task_name, task_stage)
		
		self.task_list.append(task)

		# if len(self.task_list) == 0:
		# 	self.task_list.append(weight)
		# else:
		# 	self.task_list.append(self.task_weights[len(self.task_weights) - 1] + weight)

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