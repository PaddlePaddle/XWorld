import cv2
import numpy as np
import time
from libxrobot import *
from teaching_task import *

box_cardboard = "../data/cardboard_box/box.urdf";
goals = ["box"]

class XRobot3DInteractions(XWorld3DTask):
	def __init__(self, env):
		super(XRobot3DInteractions, self).__init__(env)
		self.prev_find = None

	def get_stages(self):
		stages = dict()
		stages["idle"] = self.start
		stages["stack1"] = self.stack1
		stages["stack2"] = self.stack2
		stages["terminal"] = self.start
		return stages

	def start(self):
		self.reset()
		self.env.EnableInventory(1)
		self.env.CreateArena(3, 3)
		self.env.MakeObjectPickable("box")

		for i in xrange(3):
			x = random.uniform(-0.25, 0.25);
			y = random.uniform(-0.25, 0.25);
			r = random.uniform(0.0, 2.0);
			self.env.SpawnAnObject(box_cardboard, [x + 7, 0.5, y + i * 2 + 0.5],\
				[0,1,0,r], 1.0, "box", False)

		self.agent = self.env.SpawnAnObject("husky/husky.urdf", [2,0,2], \
			[-1,0,0,1.57], 1.0, "Agent", True)
		self.env.AttachCameraTo(self.agent, [0,2,0.0])
		self.env.Initialize()
		self.env.HighlightCenter(True)

		self._bind("S -> task")
		self.sentence = self._generate()

		return ["stack1", 0.0, self.sentence]

	def stack1(self):

		reward, time_out = self._time_reward()
		next_stage = "stack1"

		goal_list = self.env.QueryObjectByLabel("box")

		med, high = 0, 0
		for goal in goal_list:
			if goal.GetPosition()[1] > 0.9:
				med += 1
				high += 1
			if goal.GetPosition()[1] > 1.0:
				high += 1

		if med == 1:
			next_stage = "stack2"
			self._bind("S -> stack2")
			self.sentence = self._generate()

		return [next_stage, reward, self.sentence]

	def stack2(self):

		reward, time_out = self._time_reward()
		next_stage = "stack2"

		goal_list = self.env.QueryObjectByLabel("box")

		med, high = 0, 0
		for goal in goal_list:
			if goal.GetPosition()[1] > 0.9:
				med += 1
				high += 1
			if goal.GetPosition()[1] > 1.0:
				high += 1

		if med == 2 and high == 4:
			reward = self._successful_goal(reward)
			next_stage = "terminal"
		elif med == 0 and high == 0:
			next_stage = "stack1"
			self._bind("S -> task")
			self.sentence = self._generate()

		return [next_stage, reward, self.sentence]

	def _define_grammar(self):
		all_goal_names = self._get_all_goal_names_as_rhs(goals)
		grammar_str = """
		S --> task | stack2 | correct | wrong
		task -> I0 | I1
		stack2 -> I2
		correct -> 'Well' 'done' '!'
		wrong -> 'Wrong' '!'
		I0 -> 'find' 'three' G 'and' 'stack' 'them' '.'
		I1 -> 'find' 'all' G 'and' 'stack' 'them' '.'
		I2 -> 'only' 'one' 'left' '!' | 'nice' 'try' '!' | 'one' 'left' '!'
		G --> %s
		""" % all_goal_names
		return grammar_str, "S"


class XWorld3DEnv(object):
	def __init__(self):
		self.env = Playground(640, 480, HEADLESS, NORMAL, GPU0)
		self.task_group = TaskGroup("TaskGroup")
		self.task_group.add_task("NavTargetStack", XRobot3DInteractions(self.env))
		self.first = True

	def reset(self):
		self.env.Clear()

	def step(self, action):

		if self.first != True:
			self.env.UpdateSimulationWithAction(action)

		self.first = False
		self.task_group.run_stage()
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