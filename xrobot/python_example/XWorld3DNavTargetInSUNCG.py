from libxrobot import *
from teaching_task import *
import cv2
import numpy as np
import random
import os
import os.path

suncg_dir    = "../data/suncg";
suncg_meta   = suncg_dir + "/ModelCategoryMapping.csv";
suncg_house0 = suncg_dir + "/house/7c16efebdfe46f3f14fa81abe500589c/house.json";

goals        = ["door", "trash_can", "refrigerator", "chair", "fruit_bowl", "dishwasher", "microwave", "coffee_machine", "kettle"]

class XWorld3DNavTargetInSUNCG(XWorld3DTask):
	def __init__(self, env):
		super(XWorld3DNavTargetInSUNCG, self).__init__(env)

	def idle(self):
		self.reset()
		self.env.CreateSceneFromSUNCG()
		self.env.LoadSUNCG(suncg_house0, suncg_meta, suncg_dir)

		self.agent = self.env.SpawnAnObject("husky/husky.urdf", \
			[-6,0.1,-1], [-1,0,0,1.57], 0.6, "Agent", True)
		self.env.AttachCameraTo(self.agent, [0.0,1.3,0.0])
		self.env.Initialize()

		self.target = goals[random.randint(0, len(goals) - 1)]

		self._bind("S -> start")
		self._bind("G -> '" + self.target + "'")
		self.sentence = self._generate()

		return ["navigation", 0.0, self.sentence]

	def navigation(self):
		
		reward, time_out = self._time_reward()
		next_stage = "navigation"

		if not time_out:
			if self.env.QueryObjectWithLabelAtCameraCenter(self.target):
				reward = self._successful_goal(reward)
				next_stage = "terminal"
		else:
			next_stage = "terminal"

		return [next_stage, reward, self.sentence]

	def get_stages(self):
		stages = dict()
		stages["idle"] = self.idle
		stages["navigation"] = self.navigation
		stages["terminal"] = self.idle
		return stages

	def _define_grammar(self):
		all_goal_names = self._get_all_goal_names_as_rhs(goals)
		grammar_str = """
		S --> start | timeup | correct | wrong
		start -> I0 | I1 | I2 | I3 | I4 | I5 | I6
		correct -> 'Well' 'done' '!'
		wrong -> 'Wrong' '!'
		timeup -> 'Time' 'up' '.'
		I0 -> G
		I1 -> A G 'please' '.'
		I2 -> 'Please' A G '.'
		I3 -> A G '.'
		I4 -> G 'is' 'your' D '.'
		I5 -> G 'is' 'the' D '.'
		I6 -> Y A G '?'
		A -> 'go' 'to' | 'navigate' 'to' | 'reach' | 'move' 'to' | 'collect'
		Y -> 'Could' 'you' 'please' | 'Can' 'you' | 'Will' 'you'
		D -> 'destination' | 'target' | 'goal' | 'end'
		G --> %s
		""" % all_goal_names
		return grammar_str, "S"

class XWorld3DEnv(object):
	def __init__(self):
		self.env = Playground(640, 480, HEADLESS, NORMAL_NO_SHADOW, GPU0)
		self.task_group = TaskGroup("TaskGroup")
		self.task_group.add_task("NavTargetSUNCG", XWorld3DNavTargetInSUNCG(self.env))
		self.first = True
		self.env.SetLighting({ "exposure" : 1.2 })

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