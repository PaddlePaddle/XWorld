from libxrobot import *
from teaching_task import *
import cv2
import numpy as np
import random
import os
import os.path

'''
Task: Navigate to chair in SUNCG scene

This task requires agent to navigate the target object.
'''

suncg_dir    = "../data/suncg";
suncg_meta   = suncg_dir + "/ModelCategoryMapping.csv";
suncg_house  = suncg_dir + "/house/7c16efebdfe46f3f14fa81abe500589c/house.json";

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
		self.env.Clear()
		self.env.CreateSceneFromSUNCG()
		self.env.LoadSUNCG(suncg_house, suncg_meta, suncg_dir)

		self.agent = self.env.SpawnAnObject("husky/husky.urdf", \
			[-6,0.1,-1], [-1,0,0,1.57], 0.6, "Agent", True)
		self.env.AttachCameraTo(self.agent, [0.3,1.3,0.0])
		self.env.Initialize()

		return "navigation"

	def navigation(self):
		
		# Query Object At Forward
		if self.env.QueryObjectWithLabelAtCameraCenter("chair"):
			return "idle"

		# Fetch Raw Images
		image_width  = self.env.GetWidth()
		image_height = self.env.GetHeight()

		image_str = self.env.GetCameraRGBDRaw()
		image_rgbd = np.fromstring(image_str, np.uint8).reshape( image_height, image_width, 4 )
		image_rgbd = cv2.cvtColor(image_rgbd, cv2.COLOR_BGRA2RGBA)
		image_rgbd = cv2.flip(image_rgbd, 0)
		image_rgb = np.array(image_rgbd[:,:,:3])
		cv2.imshow("RGB", image_rgb)
		return "navigation"

