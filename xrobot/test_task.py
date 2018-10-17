import libxrobot
from teaching_task import *
import cv2
import numpy as np

class XRobot3DNavTarget(TaskInterface):
	def __init__(self, env):
		self.env = env
		self.agent = None

	def get_stages(self):
		stages = dict()
		stages["idle"] = self.start
		stages["navigation"] = self.navigation
		return stages

	def start(self):
		self.env.EnableInventory(10)
		self.env.Clear()
		self.env.CreateAnTestScene()
		self.env.SetLighting()

		self.env.SpawnAnObject("./crate_1/crate.urdf", [5,0,0], [1,0,0,0], 1.0, "Crate", False)
		self.agent = self.env.SpawnAnObject("husky/husky.urdf", [2,0,2], [-1,0,0,1.57], 1.0, "Agent", True)
		self.env.AttachCameraTo(self.agent, [0.3,1.3,0.0])
		self.env.Initialize()

		return "navigation"

	def navigation(self):
		# Query Object At Forward
		if self.env.QueryObjectWithLabelAtForward("Crate"):
			return "idle"

		# Fetch Agent Status
		position = self.agent.GetPosition()
		orientation = self.agent.GetOrientation()

		# Fetch Raw Images
		image_str = self.env.GetCameraRGBDRaw()
		image_rgbd = np.fromstring(image_str, np.uint8).reshape( 480, 640, 4 )
		image_rgbd = cv2.cvtColor(image_rgbd, cv2.COLOR_BGRA2RGBA)
		image_rgbd = cv2.flip(image_rgbd, 0)

		image_rgb = image_rgbd[:,:,:3]
		image_depth = image_rgbd[:,:,3]
		cv2.imshow("RGB", image_rgb)
		cv2.imshow("Depth", image_depth)

		# key = cv2.waitKey(1)

		# if key == 27: # ESC
		# 	return "idle"
		# elif key == 119: # W
		# 	self.env.MoveForward(10)
		# elif key == 97:  # A
		# 	self.env.TurnLeft(10)
		# elif key == 115: # S
		# 	self.env.MoveBackward(10)
		# elif key == 100: # D
		# 	self.env.TurnRight(10)

		return "navigation"


# scene = libxrobot.Playground(640, 480, libxrobot.HEADLESS)
# task_group = TaskGroup("TaskGroup")
# task_nav   = XRobot3DNavTarget(scene)
# task_group.add_task("Navigation", task_nav)

# while True:
# 	task_group.run_stage()
