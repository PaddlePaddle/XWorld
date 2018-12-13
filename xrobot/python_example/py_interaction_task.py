from teaching_task import *
import cv2
import numpy as np
import random

oven       = "../data/oven/oven.json";
drawer     = "../data/drawer/drawer.json";
drawer2    = "../data/drawer2/drawer.json";
gift       = "../data/gift/gift.json";
laptop     = "../data/laptop/laptop.json";
cake       = "../data/cake/cake.json";
trashcan   = "../data/trashcan/trashcan.json";
screen     = "../data/screen/screen.json";
piano      = "../data/piano/piano.json";
apple      = "../data/apple/apple.urdf";

class XRobot3DInteractions(XWorld3DTask):
	def __init__(self, env):
		self.env = env
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
		self.env.MakeObjectPickable("apple")

		self.env.SpawnAnObject(oven, [4,0,2], [1,0,0,0], 1.0, "oven", True)
		self.env.SpawnAnObject(drawer, [6,0,2], [1,0,0,0], 1.0, "drawer", True)
		self.env.SpawnAnObject(gift, [8,0,2], [1,0,0,0], 1.0, "gift", True)
		self.env.SpawnAnObject(laptop, [10,0,2], [1,0,0,0], 1.0, "laptop", True)
		self.env.SpawnAnObject(cake, [4,0,6], [1,0,0,0], 1.0, "cake", True)
		self.env.SpawnAnObject(trashcan, [6,0,6], [1,0,0,0], 1.0, "trashcan", True)
		self.env.SpawnAnObject(screen, [8,0,6], [1,0,0,0], 1.0, "screen", True)
		self.env.SpawnAnObject(piano, [10,0,6], [1,0,0,0], 1.0, "piano", True)
		self.env.SpawnAnObject(drawer2, [4,0,10], [1,0,0,0], 1.0, "drawer2", True)

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

