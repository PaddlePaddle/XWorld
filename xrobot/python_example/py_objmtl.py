from libxrobot import *
from teaching_task import *
import cv2
import numpy as np
import random
import os
import os.path

suncg_dir   = os.path.abspath(os.path.join(os.getcwd(), os.pardir)) + "/data/suncg";
suncg_meta  = suncg_dir + "/metadata/ModelCategoryMapping.csv";
suncg_house = suncg_dir + "/house/7c16efebdfe46f3f14fa81abe500589c/house.json";

class XRobot3DSUNCG(XWorld3DTask):
	def __init__(self, env):
		self.env = env
		self.agent = None

	def get_stages(self):
		stages = dict()
		stages["idle"] = self.start
		stages["navigation"] = self.navigation
		return stages

	def start(self):
		self.env.SetLighting({ "ssr": True })

		# self.env.EnableInventory(1)
		self.env.Clear()
		self.env.CreateSceneFromSUNCG()
		self.env.LoadSUNCG(suncg_house, suncg_meta, suncg_dir, \
			REMOVE_DOOR | REMOVE_STAIR)

		self.agent = self.env.SpawnAnObject("husky/husky.urdf", \
			[-6,0.1,-1], [-1,0,0,1.57], 0.6, "Agent", True)
		self.env.AttachCameraTo(self.agent, [0.3,1.3,0.0])
		self.env.Initialize()

		return "navigation"

	def navigation(self):
		# Query Object At Forward
		if self.env.QueryObjectWithLabelAtForward("car"):
			return "idle"

		# Fetch Raw Images
		image_width  = self.env.GetWidth()
		image_height = self.env.GetHeight()

		image_str = self.env.GetCameraRGBDRaw()
		image_rgbd = np.fromstring(image_str, np.uint8).reshape( image_height, image_width, 4 )
		image_rgbd = cv2.cvtColor(image_rgbd, cv2.COLOR_BGRA2RGBA)
		image_rgbd = cv2.flip(image_rgbd, 0)

		image_rgb = np.array(image_rgbd[:,:,:3])
		image_depth = np.array(image_rgbd[:,:,3])


		# frames = "frames: " + str(self.env.GetStatus()["frames"])
		framerate = " | framerate: " + str(self.env.GetStatus()["framerate"])

		# cv2.putText(image_rgb, frames + framerate, (30,30), \
  #   		cv2.FONT_HERSHEY_PLAIN, 1, (200,250,250), 1);

		cv2.imshow("RGB", image_rgb)
		return "navigation"

