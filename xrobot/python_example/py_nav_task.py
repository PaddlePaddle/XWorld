from teaching_task import *
import cv2
import numpy as np
import random

class XRobot3DNavTarget(XWorld3DTask):
	def __init__(self, env):
		self.env = env
		self.agent = None

	def get_stages(self):
		stages = { 
			"idle" : self.start,
			"navigation" : self.navigation
		}
		return stages

	def start(self):
		self.env.EnableInventory(10)
		self.env.Clear()
		self.env.CreateAnTestScene()

		self.env.MakeObjectPickable("key_crate")

		x = random.randint(4, 7)
		y = random.randint(4, 7)

		self.env.SpawnAnObject("/home/ziyuli/XWorld/xrobot/data/door1/door.json", [x,1,y], [1,0,0,0], 1.0, "Door", False)

		x = random.randint(2, 4)
		y = random.randint(2, 4)

		self.env.SpawnAnObject("crate_0.3/crate.urdf", [x,0,y], [1,0,0,0], 1.0, "key_crate", False)

		self.agent = self.env.SpawnAnObject("husky/husky.urdf", [2,0,2], [-1,0,0,1.57], 1.0, "Agent", True)
		self.env.AttachCameraTo(self.agent, [0.3,1.3,0.0])
		self.env.Initialize()

		return "navigation"

	def navigation(self):
		# Query Object At Forward
		# if self.env.QueryObjectWithLabelAtForward("Crate"):
		# 	return "idle"

		# Fetch Agent Status
		position = self.agent.GetPosition()
		orientation = self.agent.GetOrientation()

		# Fetch Raw Images
		image_width  = self.env.GetWidth()
		image_height = self.env.GetHeight()

		image_str = self.env.GetCameraRGBDRaw()
		image_rgbd = np.fromstring(image_str, np.uint8).reshape( image_height, image_width, 4 )
		image_rgbd = cv2.cvtColor(image_rgbd, cv2.COLOR_BGRA2RGBA)
		image_rgbd = cv2.flip(image_rgbd, 0)

		image_rgbd_resize = cv2.resize(image_rgbd, None, fx=0.8, fy=0.8)
		image_rgb = np.array(image_rgbd_resize[:,:,:3])

		frames = "frames: " + str(self.env.GetStatus()["frames"])
		framerate = " | framerate: " + str(self.env.GetStatus()["framerate"])

		cv2.putText(image_rgb, frames + framerate, (30,30), \
    		cv2.FONT_HERSHEY_PLAIN, 1, (200,250,250), 1);

		cv2.imshow("RGB", image_rgb)
		return "navigation"

