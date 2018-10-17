import libxrobot
from teaching_task import *
import cv2
import numpy as np
import random

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

		x = random.randint(1, 8)
		y = random.randint(1, 8)

		self.env.SpawnAnObject("./crate_1/crate.urdf", [x,0,y], [1,0,0,0], 1.0, "Crate", False)
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
		return "navigation"

