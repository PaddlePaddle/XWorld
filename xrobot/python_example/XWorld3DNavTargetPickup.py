from libxrobot import *
from teaching_task import *
import cv2
import numpy as np
import random

meta       = "/home/ziyuli/XWorld/xrobot/data/ModelCategoryMapping.csv";
gift       = "/home/ziyuli/XWorld/xrobot/data/gift/gift.json";
crate1     = "./crate_1/crate.urdf";
crate03    = "./crate_0.3/crate.urdf";

models     = [crate1, crate03]
m_tags     = ["large crate", "small crate"]
conf0       = {"single" : [crate03, -1, 5]}
conf1       = {"single" : [crate1, -1, 5]}

class XWorld3DNavTargetPickup(XWorld3DTask):
	def __init__(self, env):
		super(XWorld3DNavTargetPickup, self).__init__(env)

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
		self.env.SpawnModelsConf(conf1)
		self.env.SpawnModels()

		self.env.Initialize()

		self._bind("S -> start")
		self._bind("G0 -> '" + "small' 'crate" + "'")
		self._bind("G1 -> '" + "large' 'crate" + "'")
		self.sentence = self._generate()

		return ["pickup", 0.0, self.sentence]

	def pickup(self):

		reward, time_out = self._time_reward()
		next_stage = "pickup"

		if not time_out:
			desired_event = set(["Grasp", "small crate"])
			if set(self.env.QueryLastEvent()) & desired_event == desired_event:
				next_stage = "dropdown"


		return [next_stage, reward, self.sentence]

	def dropdown(self):
		
		reward, time_out = self._time_reward()
		next_stage = "dropdown"

		print self.env.QueryLastEvent()

		if not time_out:
			if self.env.QueryObjectWithLabelAtCameraCenter("small crate"):
				small_crate = self.env.QueryObjectAtCameraCenter()
				if small_crate.GetPosition()[1] > 0.5:
					reward = self._successful_goal(reward)
					next_stage = "terminal"
				else:
					reward = self._failed_goal(reward)
					next_stage = "terminal"
			else:
				e = set(self.env.QueryLastEvent()) & set(["PutDown", "Nothing"])
				if e == set(["PutDown", "Nothing"]):
					next_stage = "dropdown"
					[next_stage, reward, self.sentence]
				elif e:
					next_stage = "pickup"
		else:
			next_stage = "terminal"

		return [next_stage, reward, self.sentence]

	def get_stages(self):
		stages = dict()
		stages["idle"] = self.idle
		stages["pickup"] = self.pickup
		stages["dropdown"] = self.dropdown
		stages["terminal"] = self.idle
		return stages

	def _define_grammar(self):
		all_goal_names = self._get_all_goal_names_as_rhs(["small' 'crate", "large' 'crate"])
		grammar_str = """
		S --> start | timeup | correct | wrong
		start -> I1 | I2 | I3 | I6 
		correct -> 'Well' 'done' '!'
		wrong -> 'Wrong' '!'
		timeup -> 'Time' 'up' '.'
		I1 -> A0 G0 'and' A1 G1 'please' '.'
		I2 -> 'Please' A0 G0 'and' A1 G1 '.'
		I3 -> A0 G0 'and' A1 G1 '.'
		I6 -> Y A0 G0 'and' A1 G1 '?'
		A1 -> 'put' 'onto' | 'drop' 'onto'
		A0 -> 'pick' 'up' | 'grasp' | 'take' | 'collect'
		Y  -> 'Could' 'you' 'please' | 'Can' 'you' | 'Will' 'you'
		G0 --> %s
		G1 --> %s
		""" % (all_goal_names, all_goal_names)
		return grammar_str, "S"

class XWorld3DEnv(object):
	def __init__(self):
		self.env = Playground(640, \
							  480, \
							  HEADLESS, \
							  QUALITY_NORMAL, \
							  0)

		self.task_group = TaskGroup("TaskGroup")
		self.task_group.add_task("NavTargetPickup", XWorld3DNavTargetPickup(self.env))
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