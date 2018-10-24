from teaching_task import *
import cv2
import numpy as np
import random

class XRobot3DNavAgentTarget(TaskInterface):
	def __init__(self, env):
		self.env = env
		self.agent = None
		self.nav_agent = None

	def get_stages(self):
		stages = dict()
		stages["idle"] = self.start
		stages["navigation"] = self.navigation
		return stages

	def start(self):
		self.env.SetLighting({ "ssr": True })

		self.env.Clear()
		self.env.CreateAnTestScene()
		self.env.EnableInventory(10)
		self.env.EnableNavigation([-2,-1,-2], [10,5,10], False)

		self.env.SpawnAnObject("./crate_1/crate.urdf", [5,0,5], [1,0,0,0], 1.0, "Crate", True)
		self.env.SpawnAnObject("./crate_1/crate.urdf", [4,0,4], [1,0,0,0], 1.0, "Crate", True)
		self.env.AssignAgentRadius(0.2)
		self.env.BakeNavigationMesh()

		self.nav_agent = self.env.SpawnNavigationAgent("./crate_0.3/crate.urdf", "NavAgent", [2,0,0], [1,0,0,0])
		self.env.AssignNavigationAgentTarget(self.nav_agent, [8, 0, 8])

		self.agent = self.env.SpawnAnObject("husky/husky.urdf", [2,0,2], [-1,0,0,1.57], 1.0, "Agent", True)
		self.env.AttachCameraTo(self.agent, [0.3,1.3,0.0])
		self.env.Initialize()

		return "navigation"

	def navigation(self):
		# # Query Object At Forward
		# if self.env.QueryObjectWithLabelAtForward("NavAgent"):
		# 	return "idle"

		# Fetch Agent Status
		position = self.agent.GetPosition()
		orientation = self.agent.GetOrientation()

		# Fetch Raw Images
		image_str = self.env.GetCameraRGBDRaw()
		image_rgbd = np.fromstring(image_str, np.uint8).reshape( 480, 640, 4 )
		image_rgbd = cv2.cvtColor(image_rgbd, cv2.COLOR_BGRA2RGBA)
		image_rgbd = cv2.flip(image_rgbd, 0)

		image_rgb = np.array(image_rgbd[:,:,:3])
		image_depth = np.array(image_rgbd[:,:,3])

		frames = "frames: " + str(self.env.GetStatus()["frames"])
		framerate = " | framerate: " + str(self.env.GetStatus()["framerate"])

		if self.env.GetStatus()["frames"] % 800 == 0:
			x = random.randint(1, 8)
			y = random.randint(1, 8)
			self.env.BakeNavigationMesh()
			self.env.AssignNavigationAgentTarget(self.nav_agent, [x, 0, y])


		cv2.putText(image_rgb, frames + framerate, (30,30), \
    		cv2.FONT_HERSHEY_PLAIN, 1, (200,250,250), 1);

		cv2.imshow("RGB", image_rgb)
		return "navigation"

