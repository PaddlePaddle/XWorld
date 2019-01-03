import cv2
import numpy as np
import time
from libxrobot import *
from teaching_task import *
from XBridge import *

goals = ["box"]

class XRobot3DInteractions(XWorld3DTask):
	def __init__(self, env):
		super(XRobot3DInteractions, self).__init__(env)

	def get_stages(self):
		stages = dict()
		stages["idle"] = self.start
		stages["task"] = self.task
		return stages

	def start(self):
		self.reset()
		self.env.EnableInventory(4)
		self.env.CreateEmptyScene()
		self.env.LoadXWorldScene("../../data/scene.xworld")
		self.agent = self.env.SpawnAnObject("husky/husky.urdf", [0,0,0], \
			[-1,0,0,1.57], 1.0, "Agent", True)
		self.env.AttachCameraTo(self.agent, [0,1.6,0.0])
		self.env.Initialize()
		self.env.HighlightCenter(True)
		self.env.DisplayInventory(False)

		self._bind("S -> task")
		self.sentence = self._generate()

		return ["task", 0.0, self.sentence]

	def task(self):
		return ["task", 0.0, self.sentence]


	def _define_grammar(self):
		all_goal_names = self._get_all_goal_names_as_rhs(goals)
		grammar_str = """
		S --> task | correct | wrong 
		task -> I0
		correct -> 'Well' 'done' '!'
		wrong -> 'Wrong' '!'
		I0 -> 'Demo'
		G --> %s
		""" % all_goal_names
		return grammar_str, "S"

class XWorld3DEnv(object):
	def __init__(self):
		self.env = Playground(640, 480, HEADLESS, NORMAL, GPU0)
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
	srv = XWorldServer('',1236, env)
	srv.start()
	
	while (not env.game_over()):

		action = NO_ACTION

		# action inputs from keyboard
		key = cv2.waitKey(1)
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

	srv.stop()

if __name__ == '__main__':
    main()