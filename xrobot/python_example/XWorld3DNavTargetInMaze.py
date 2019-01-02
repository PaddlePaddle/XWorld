from libxrobot import *
from teaching_task import *
import cv2
import numpy as np
import random

door1      = "../data/door_green/door.json";
door2      = "../data/door_red/door.json";
door3      = "../data/door_yellow/door.json";
door4      = "../data/door_blue/door.json";
door5      = "../data/door_purple/door.json";

key2       = "../data/key_red/key.urdf";
key3       = "../data/key_yellow/key.urdf";
key4       = "../data/key_blue/key.urdf";
key5       = "../data/key_purple/key.urdf";

door       = "../data/door/door.urdf";
wall       = "../data/wall0/floor.urdf";
floor_0    = "../data/floor0/floor.urdf";
floor_1    = "../data/floor1/floor.urdf";
crate1     = "../data/crate_1/crate.urdf";
crate03    = "../data/crate_0.3/crate.urdf";
cat1       = "../data/cat_1/cat.urdf";
gift       = "../data/gift/gift.json";
piano      = "../data/piano/piano.json";
box_cardboard = "../data/cardboard_box/box.urdf";

floor_test  = "../data/floor/floor.urdf";
wall_test   = "../data/wall/floor.urdf";

class XWorld3DNavTargetInMaze(XWorld3DTask):
	def __init__(self, env):
		super(XWorld3DNavTargetInMaze, self).__init__(env)

	def idle(self):
		self.reset()
		self.env.EnableInventory(4)
		self.env.CreateRandomGenerateScene()
		self.env.SetLighting({ 
			"ssr" : True, \
			"exposure" : 0.0
		})

		doors  = [door2, door3, door4, door5]
		keys   = [key2, key3, key4, key5]
		d_tags = ["red", "yellow", "blue", "purple"]
		models = [box_cardboard, piano, gift, cat1]
		m_tags = ["box", "piano", "gift", "cat"]

		self.env.MakeObjectPickable("box")
		self.env.LoadBasicObjects(doors, keys, d_tags, door1, wall_test, [floor_test])
		start = self.env.LoadSceneConfigure(6, 6, 4, 3)
		lastgroup = self.env.GetRoomGroups()[-1]

		conf   = {"single" : [cat1, lastgroup, 3, box_cardboard, -1, 3,  gift, -1, 5], \
				  "stack"  : [box_cardboard, box_cardboard, -1, 1, 3]}

		self.env.LoadModels(models, m_tags)
		self.env.SpawnModelsConf(conf)
		self.env.SpawnModels()

		self.agent = self.env.SpawnAnObject("husky/husky.urdf", \
			start, [-1,0,0,1.57], 0.6, "Agent", True)
		self.env.AttachCameraTo(self.agent, [0.3,1.3,0.0])
		self.env.Initialize()
		self.env.HighlightCenter(True)
		self.env.DisplayInventory(True)

		self._bind("S -> start")
		self._bind("G -> '" + "cat" + "'")
		self.sentence = self._generate()

		return ["navigation", 0.0, self.sentence]

	def navigation(self):
		
		reward, time_out = self._time_reward()
		next_stage = "navigation"

		if not time_out:
			if self.env.QueryObjectWithLabelAtCameraCenter("cat"):
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
		all_goal_names = self._get_all_goal_names_as_rhs(["cat"])
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
		A -> 'find' | 'navigate' 'to' | 'reach' | 'move' 'to' | 'collect'
		Y -> 'Could' 'you' 'please' | 'Can' 'you' | 'Will' 'you'
		D -> 'destination' | 'target' | 'goal' | 'end'
		G --> %s
		""" % all_goal_names
		return grammar_str, "S"

class XWorld3DEnv(object):
	def __init__(self):
		self.env = Playground(640, 480, HEADLESS, NORMAL, GPU0)
		self.task_group = TaskGroup("TaskGroup")
		self.task_group.add_task("NavTargetMaze", XWorld3DNavTargetInMaze(self.env))
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
		elif key == 54: # kp6 Open
			action = 12
		elif key == 51: # kp3 Close
			action = 13
		elif key == 55: # kp7 Enable Interact
			action = 11
		elif key == 56: # kp8 Disable Interact
			action = 13
		elif key == 27:  # ESC
			break

		# update
		env.step(action)

if __name__ == '__main__':
	main()