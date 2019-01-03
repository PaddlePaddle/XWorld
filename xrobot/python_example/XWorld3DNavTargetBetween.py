from libxrobot import *
from teaching_task import *
import cv2
import numpy as np
import random
import math

meta       = "../data/ModelCategoryMapping.csv";
gift       = "../data/gift/gift.json";
crate1     = "../data/crate_1/crate.urdf";
cat1       = "../data/cat_1/cat.urdf";

models     = [crate1, gift, cat1]
m_tags     = ["crate1", "gift", "cat"]
conf0      = {"pair" : [crate1, gift, -1, 1]}
conf1      = {"single" : [cat1, -1, 8]}

class XWorld3DNavTargetBetween(XWorld3DTask):
	def __init__(self, env):
		super(XWorld3DNavTargetBetween, self).__init__(env)

	def idle(self):
		self.reset()
		self.env.CreateArena()
		self.env.EnableInventory(4)
		self.env.LoadTag(meta);

		self.agent = self.env.SpawnAnObject("husky/husky.urdf", \
			[3,0,3], [-1,0,0,1.57], 1.0, "Agent", True)
		self.env.AttachCameraTo(self.agent, [0.3,1.6,0.0])

		self.env.LoadModels(models, m_tags)
		self.env.SpawnModelsConf(conf0)
		self.env.ResolvePath()
		self.env.SpawnModelsConf(conf1)
		self.env.SpawnModels()
		self.env.Initialize()

		self._bind("S -> start")
		self._bind("G1 -> '" + "crate" + "'")
		self._bind("G2 -> '" + "gift" + "'")
		self.sentence = self._generate()

		return ["navigation", 0.0, self.sentence]

	def navigation(self):
		
		reward, time_out = self._time_reward()
		next_stage = "navigation"

		goal0 = self.env.QueryObjectByLabel("crate1")[0]
		goal1 = self.env.QueryObjectByLabel("gift")[0]
		goal0_center = goal0.GetPosition()
		goal1_center = goal1.GetPosition()
		agent_center = self.agent.GetPosition()
		goal01_center = tuple(map(lambda x, y: (x + y) / 2, goal0_center, goal1_center))
		agent_goal_vec = tuple(map(lambda x, y: x - y, agent_center, goal01_center))
		agent_goal_dst = math.sqrt(sum(map(lambda x: x ** 2 , agent_goal_vec)))

		if not time_out:
			if agent_goal_dst < 1.5:
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
		all_goal_names = self._get_all_goal_names_as_rhs(["crate", "gift"])
		grammar_str = """
		S --> start | timeup | correct | wrong
		start -> I0 | I1 | I2 | I3 | I4
		correct -> 'Well' 'done' '!'
		wrong -> 'Wrong' '!'
		timeup -> 'Time' 'up' '.'
		I0 -> A L B '.'
		I1 -> A L B 'please' '.'
		I2 -> 'Please' A L B '.'
		I3 -> L B 'is' 'your' D '.'
		I4 -> Y A L B '?'
		A -> 'go' 'to' | 'navigate' 'to' | 'reach' | 'move' 'to'
		B -> 'between' G1 'and' G2
		L -> 'the' 'location' | 'the' 'grid' | 'the' 'place'
		Y -> 'Could' 'you' 'please' | 'Can' 'you' | 'Will' 'you'
		D -> 'destination' | 'target' | 'goal' | 'end'
		G1 --> %s
		G2 --> %s
		""" % (all_goal_names, all_goal_names)
		return grammar_str, "S"

class XWorld3DEnv(object):
	def __init__(self):
		self.env = Playground(640, 480, HEADLESS, NORMAL, GPU0)
		self.task_group = TaskGroup("TaskGroup")
		self.task_group.add_task("NavTargetBetween", XWorld3DNavTargetBetween(self.env))
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