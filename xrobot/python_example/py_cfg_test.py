from teaching_task import *
import cv2
import numpy as np
import random

gift       = "/home/ziyuli/XWorld/xrobot/data/gift/gift.json";

class XRobot3DTest(XWorld3DTask):
	def __init__(self, env):
		super(XWorld3DTask, self).__init__(env)
		self.agent = None

	def get_stages(self):
		stages = dict()
		stages["idle"] = self.start
		stages["navigation"] = self.navigation
		return stages

	def start(self):
		self.env.Clear()
		self.env.CreateAnTestScene()
		self.env.EnableInventory(4)

		self.env.SpawnAnObject(gift, [6,0,6], [1,0,0,0], 1.0, "gift", True)

		self.agent = self.env.SpawnAnObject("husky/husky.urdf", [2,0,2], [-1,0,0,1.57], 1.0, "Agent", True)
		self.env.AttachCameraTo(self.agent, [0.3,1.6,0.0])
		self.env.Initialize()

		return "navigation"

	def navigation(self):

		# Fetch Raw Images
		image_str = self.env.GetCameraRGBDRaw()
		image_rgbd = np.fromstring(image_str, np.uint8).reshape( 360, 640, 4 )
		image_rgbd = cv2.cvtColor(image_rgbd, cv2.COLOR_BGRA2RGBA)
		image_rgbd = cv2.flip(image_rgbd, 0)

		image_rgbd_resize = cv2.resize(image_rgbd, None, fx=0.75, fy=0.75)
		image_rgb = np.array(image_rgbd_resize[:,:,:3])

		frames = "frames: " + str(self.env.GetStatus()["frames"])
		framerate = " | framerate: " + str(self.env.GetStatus()["framerate"])

		cv2.putText(image_rgb, frames + framerate, (30,30), \
    		cv2.FONT_HERSHEY_PLAIN, 1, (200,250,250), 1);

		cv2.imshow("RGB", image_rgb)
		return "navigation"

