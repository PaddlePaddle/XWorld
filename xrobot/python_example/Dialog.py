import cv2
import numpy as np
import time
import random
from libxrobot import *
from teaching_task import *

box_red    = "../data/box/red.urdf";
box_green  = "../data/box/green.urdf";
box_blue   = "../data/box/blue.urdf";
box_yellow = "../data/box/yellow.urdf";
box_cyan   = "../data/box/cyan.urdf";
box_purple = "../data/box/purple.urdf";
box_orange = "../data/box/orange.urdf";
box_white  = "../data/box/white.urdf";

models = [box_red, box_green, box_blue, box_yellow, box_purple, box_orange, box_white] 
goals  = ["redbox", "greenbox", "bluebox", "yellowbox", "purplebox", "orangebox", "whitebox"]

class XWorld3DDialog(XWorld3DTask):
	def __init__(self, env):
		super(XWorld3DDialog, self).__init__(env)

		self.goal = None
		self.obj = None
		self.goal_model = list()
		for i in xrange(len(goals)):
			self.goal_model.append((goals[i], models[i]))

	def get_stages(self):
		stages = dict()
		stages["idle"] = self.start
		stages["teach_object"] = self.teach_object
		stages["teach_action"] = self.teach_action
		stages["task"] = self.task
		stages["terminal"] = self.start
		return stages

	def start(self):
		self.count = 0

		self.reset()
		self.env.EnableInventory(4)
		self.env.CreateArena(3, 3)
		
		for goal in goals:
			self.env.MakeObjectPickable(goal)

		random.shuffle(self.goal_model)
		self.select_goal_model = self.goal_model[0:4]
		self.select_goal_model += self.goal_model[0:4]
		random.shuffle(self.select_goal_model)

		self.agent = self.env.SpawnAnObject("husky/husky.urdf", [1,0,1], [-1,0,0,1.57], \
			1.0, "Agent", True, False)
		self.env.AttachCameraTo(self.agent, [0,1.7,0.0])
		self.env.Initialize()
		self.env.HighlightCenter(False)
		self.env.DisplayInventory(False)
		self.env.UpdateAttachCamera(25)
		self.env.HoldActions(True)

		return ["teach_object", 0.0, ""]

	def teach_object(self):

		if self.obj != None:
			self.env.RemoveAnObject(self.obj)
			self.obj = None

		if self.count < len(self.select_goal_model):

			sel_goal  = self.select_goal_model[self.count][0]
			sel_model = self.select_goal_model[self.count][1]

			x = random.uniform(-0.08, 0.08)
			y = random.uniform(-0.08, 0.08)
			r = random.uniform(0, 1.57)

			self.obj = self.env.SpawnAnObject(sel_model, [4 + x,0.15,1 + y], [0,1,0,r], \
				1.0, sel_goal, True, False)
			self._bind("S -> object")
			self._bind("G -> '" + sel_goal + "'")
			self.sentence = self._generate()
			self.count += 1

			return ["teach_object", 0.0, self.sentence]
		else:
			self.sentence = ""
			self.env.HoldActions(False)
			return ["teach_action", 0.0, self.sentence]

	def teach_action(self):
		
		next_stage = "teach_action"
		sel_goal  = self.select_goal_model[1][0]
		sel_model = self.select_goal_model[1][1]

		if self.obj == None:

			self.env.UpdateAttachCamera(0)
			self._bind("S -> action")
			self.sentence = self._generate()

			x = random.uniform(3, 8)
			y = random.uniform(3, 8)
			r = random.uniform(0, 1.57)

			self.obj = self.env.SpawnAnObject(sel_model, [x,0.15,y], [0,1,0,r], \
				1.0, sel_goal, True, False)


		e = set(self.env.QueryLastEvent()) & set(['Grasp', sel_goal])
		if e == set(['Grasp', sel_goal]):
			next_stage = "task"

			conf = {"single" : [self.select_goal_model[0][1], -1, 2, \
								self.select_goal_model[1][1], -1, 2, \
								self.select_goal_model[2][1], -1, 2]}

			self.env.LoadModels(models, goals)
			self.env.SpawnModelsConf(conf)
			self.env.SpawnModels()
			self.env.ClearInventory()

			self.goal = self.select_goal_model[0][0]
			self.task_id = random.randint(0, 1)

			if self.task_id == 0:
				self._bind("S -> task0")
			else:
				self._bind("S -> task1")

			self._bind("G -> '" + self.goal + "'")
			self.sentence = self._generate()
			print self.sentence

		return [next_stage, 0.0, self.sentence]

	def task(self):
		reward, time_out = self._time_reward()
		next_stage = "task"

		if not time_out:
			if self.task_id == 0:
				if self.env.QueryObjectWithLabelAtCameraCenter(self.goal):
					self._bind("S -> reward")
					self.sentence = self._generate()
				else:
					self._bind("S -> task0")
					self._bind("G -> '" + self.goal + "'")
					self.sentence = self._generate()

				e = set(self.env.QueryLastEvent()) & set(['Grasp', self.goal])
				if e == set(['Grasp', self.goal]):
					reward = self._successful_goal(reward)
					next_stage = "terminal"
			else:
				if self.env.QueryObjectWithLabelAtCameraCenter(self.goal):
					self._successful_goal(reward)
					next_stage = "terminal"

		return [next_stage, reward, self.sentence]

	def _define_grammar(self):
		all_goal_names = self._get_all_goal_names_as_rhs(goals)
		grammar_str = """
		S --> object | action | task0 | task1 | timeup | correct | wrong | reward
		object -> I0 
		action -> I1
		task0 -> I2
		task1 -> I3
		reward -> 'Now' 'pick' 'up' '.'
		correct -> 'Well' 'done' '!'
		wrong -> 'Wrong' '!'
		timeup -> 'Time' 'up' '.'
		I0 -> 'this' 'is' G '.' | G
		I1 -> 'pick' 'up' '.'
		I2 -> 'find' G 'and' 'pick' 'up' '.'
		I3 -> 'find' G '.' | 'navigate' 'to' G '.'
		G --> %s
		""" % all_goal_names
		return grammar_str, "S"

class XWorld3DEnv(object):
	def __init__(self):
		self.env = Playground(640, \
							  480, \
							  HEADLESS, \
							  RENDER_QUALITY_NORMAL, \
							  1)

		self.task_group = TaskGroup("TaskGroup")
		self.task_group.add_task("Dialog", XWorld3DDialog(self.env))
		self.first = True

	def reset(self):
		self.env.Clear()

	def step(self, action):

		if self.first != True:
			self.env.UpdateSimulationWithAction(action)

		self.task_group.run_stage()
		self.first = False
		sentence = self.task_group.get_sentence()
		self.env.UpdateRenderer()

		image_str = self.env.GetCameraRGBDRaw()
		image_rgbd = np.fromstring(image_str, np.uint8).reshape( 480, 640, 4 )
		image_rgbd = cv2.flip(image_rgbd, 0)

		image_rgbd_resize = cv2.resize(image_rgbd, None, fx=0.8, fy=0.8)
		image_rgb = np.array(image_rgbd_resize[:,:,:3])
		image_d   = np.array(image_rgbd_resize[:,:,3:4])

		cv2.putText(image_rgb, sentence, (30,30), \
			cv2.FONT_HERSHEY_PLAIN, 1.25, (15,255,15), 1, cv2.LINE_AA);
		cv2.imshow("RGB", image_rgb)

	def render(self):
		self.env.UpdateRenderer()

	def game_over(self):
		return False

def main():

	env = XWorld3DEnv()
	
	while (not env.game_over()):

		action = NO_ACTION

		# action inputs from keyboard
		key = cv2.waitKey(0)
		if key == 119:   # W
			action = 0
		elif key == 97:  # A
			action = 2
		elif key == 115: # S
			action = 1
		elif key == 100: # D
			action = 3
		elif key == 49:  # kp1 Pick
			action = 8
		elif key == 50:  # kp2 Drop
			action = 9
		elif key == 48:  # kp9 Up
			action = 4
		elif key == 57:  # kp0 Down
			action = 5
		elif key == 27:  # ESC
			break

		# update
		env.step(action)

if __name__ == '__main__':
    main()