from libxrobot import *
from teaching_task import *
import cv2
import numpy as np
import random

suncg_dir   = "/home/ziyuli/Desktop/suncg";
suncg_meta  = suncg_dir + "/metadata/ModelCategoryMapping.csv";
suncg_house = suncg_dir + "/house/ffb90e69c64c4442a73204e8161ddaf2/house.json";

class XRobot3DSUNCG(TaskInterface):
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

		self.env.EnableInventory(1)
		self.env.Clear()
		self.env.CreateSceneFromSUNCG()
		self.env.LoadSUNCG(suncg_house, suncg_meta, suncg_dir, REMOVE_DOOR | REMOVE_STAIR)

		self.agent = self.env.SpawnAnObject("husky/husky.urdf", \
			[-6,0,-1], [-1,0,0,1.57], 0.6, "Agent", True)
		self.env.AttachCameraTo(self.agent, [0.3,1.3,0.0])
		self.env.Initialize()

		return "navigation"

	def navigation(self):
		# Query Object At Forward
		if self.env.QueryObjectWithLabelAtForward("car"):
			return "idle"

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

		cv2.putText(image_rgb, frames + framerate, (30,30), \
    		cv2.FONT_HERSHEY_PLAIN, 1, (200,250,250), 1);

		cv2.imshow("RGB", image_rgb)
		return "navigation"

