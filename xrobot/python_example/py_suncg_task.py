from libxrobot import *
from teaching_task import *
import cv2
import numpy as np
import random
import os
import os.path

# suncg_dir   = os.path.abspath(os.path.join(os.getcwd(), os.pardir)) + "/data/suncg";
suncg_dir    = "/home/ziyuli/Desktop/suncg";
suncg_meta   = suncg_dir + "/metadata/ModelCategoryMapping.csv";
suncg_house0 = suncg_dir + "/house/7c16efebdfe46f3f14fa81abe500589c/house.json";
suncg_house1 = suncg_dir + "/house/31d7d38dc069a43158a7f9e32a120134/house.json";
suncg_house2 = suncg_dir + "/house/84f673e2e09782667407e49ccbf1cfbb/house.json";
suncg_house3 = suncg_dir + "/house/165a17414742577e4a0773da0be3cb42/house.json";

suncg_houses = [suncg_house0, suncg_house1, suncg_house2, suncg_house3]

class XRobot3DSUNCG(XWorld3DTask):
	def __init__(self, env):
		self.env = env
		self.agent = None
		self.count = 1

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
		self.env.LoadSUNCG(suncg_houses[self.count % 4], suncg_meta, suncg_dir, \
			REMOVE_DOOR | REMOVE_STAIR)

		self.agent = self.env.SpawnAnObject("husky/husky.urdf", \
			[-6,0.1,-1], [-1,0,0,1.57], 0.6, "Agent", True)
		self.env.AttachCameraTo(self.agent, [0.3,1.3,0.0])
		self.env.Initialize()

		return "navigation"

	def navigation(self):
		
		if self.count < 100010:
			self.count += 1
			return "idle"

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

		image_rgbd_resize = cv2.resize(image_rgbd, None, fx=0.25, fy=0.25)
		image_rgb = np.array(image_rgbd_resize[:,:,:3])

		# image_rgb = np.array(image_rgbd[:,:,:3])
		# image_depth = np.array(image_rgbd[:,:,3])


		# frames = "frames: " + str(self.env.GetStatus()["frames"])
		framerate = " | framerate: " + str(self.env.GetStatus()["framerate"])

		# cv2.putText(image_rgb, frames + framerate, (30,30), \
  #   		cv2.FONT_HERSHEY_PLAIN, 1, (200,250,250), 1);

		cv2.imshow("RGB", image_rgb)
		return "navigation"

