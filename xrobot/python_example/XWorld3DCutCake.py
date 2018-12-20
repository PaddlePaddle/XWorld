import cv2
import numpy as np
import time
from libxrobot import *
from teaching_task import *

goals = ["cake"]

class XRobot3DInteractions(XWorld3DTask):
	def __init__(self, env):
		super(XRobot3DInteractions, self).__init__(env)
		self.prev_find = None
		self.slp = False

	def get_stages(self):
		stages = dict()
		stages["idle"] = self.start
		stages["findknife"] = self.findknife
		stages["cutcake"] = self.cutcake
		stages["terminal"] = self.cutcake
		return stages

	def start(self):
		self.reset()
		self.env.EnableInventory(4)
		self.env.CreateArena(3, 3)
		self.env.MakeObjectPickable("cakeknife")
		self.env.LoadXWorldScene("../data/scene_cake.xworld")

		self.agent = self.env.SpawnAnObject("husky/husky.urdf", [4,0,2], \
			[-1,0,0,1.57], 1.0, "Agent", True)
		self.env.AttachCameraTo(self.agent, [0,1.8,0.0])
		self.env.Initialize()
		self.env.HighlightCenter(True)
		self.env.DisplayInventory(True)

		self._bind("S -> task")
		self.sentence = self._generate()

		return ["findknife", 0.0, self.sentence]

	def findknife(self):

		next_stage = "findknife"
		reward, time_out = self._time_reward()

		e = set(self.env.QueryLastEvent()) & set(['Grasp', 'cakeknife'])
		if e == set(['Grasp', 'cakeknife']):
			next_stage = "cutcake"
			self._bind("S -> findknife")
			self.sentence = self._generate()

		return [next_stage, reward, self.sentence]

	def cutcake(self):

		next_stage = "cutcake"
		reward, time_out = self._time_reward()

		if not self.slp:
			e = set(self.env.QueryLastEvent()) & set(['Action', 1, 'S'])
			if e == set(['Action', 1, 'S']):
				# reward = self._successful_goal(reward)
				self._bind("S -> correct")
				self.sentence = self._generate()
				next_stage = "terminal"
				self.slp = True

			if e == set(['PutDown', 'cakeknife']):
				next_stage = "findknife"
				self._bind("S -> task")
				self.sentence = self._generate()

		return [next_stage, reward, self.sentence]


	def _define_grammar(self):
		all_goal_names = self._get_all_goal_names_as_rhs(goals)
		grammar_str = """
		S --> task | findknife | correct | wrong
		task -> I0
		findknife -> I1 | I2
		correct -> 'Well' 'done' '!'
		wrong -> 'Wrong' '!'
		I0 -> 'find' 'knife' 'and' 'cut' 'cake' '.'
		I1 -> 'nice' 'try' '!'
		I2 -> 'cut' 'cake' '.' | 'find' 'cake' '.'
		G --> %s
		""" % all_goal_names
		return grammar_str, "S"


class XWorld3DEnv(object):
	def __init__(self):
		self.env = Playground(640, \
							  480, \
							  HEADLESS, \
							  QUALITY_NORMAL, \
							  0)

		self.task_group = TaskGroup("TaskGroup")
		self.task_group.add_task("NavTargetInteraction", XRobot3DInteractions(self.env))
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
		elif key == 52:  # kp4 Attach
			action = 6
		elif key == 53:  # kp5 Detach
			action = 7
		elif key == 48:  # kp9 Up
			action = 4
		elif key == 57:  # kp0 Down
			action = 5
		elif key == 54:  # kp6 Open
			action = 12
		elif key == 51:  # kp3 Close
			action = 13
		elif key == 55:  # kp7 Enable Interact
			action = 11
		elif key == 56:  # kp8 Disable Interact
			action = 13
		elif key == 27:  # ESC
			break


		# update
		env.step(action)

if __name__ == '__main__':
    main()