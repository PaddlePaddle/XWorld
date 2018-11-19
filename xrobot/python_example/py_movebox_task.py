from teaching_task import *
import cv2
import numpy as np
import random

door1      = "/home/ziyuli/XWorld/xrobot/data/door_green/door.json";
door2      = "/home/ziyuli/XWorld/xrobot/data/door_red/door.json";
door3      = "/home/ziyuli/XWorld/xrobot/data/door_yellow/door.json";
door4      = "/home/ziyuli/XWorld/xrobot/data/door_blue/door.json";
door5      = "/home/ziyuli/XWorld/xrobot/data/door_purple/door.json";

key2       = "./key_red/key.urdf";
key3       = "./key_yellow/key.urdf";
key4       = "./key_blue/key.urdf";
key5       = "./key_purple/key.urdf";

door       = "./door/door.urdf";
wall       = "./wall0/floor.urdf";
floor_0    = "./floor0/floor.urdf";
floor_1    = "./floor1/floor.urdf";
floor_2    = "./floor2/floor.urdf";
crate1     = "./crate_1/crate.urdf";
crate03    = "./crate_0.3/crate.urdf";
cat1       = "./cat_1/cat.urdf";

floor_test  = "./floor/floor.urdf";
wall_test   = "./wall/floor.urdf";

class XRobot3DMoveBox(XWorld3DTask):
	def __init__(self, env):
		self.env = env
		self.agent = None

	def get_stages(self):
		stages = dict()
		stages["idle"] = self.start
		stages["navigation"] = self.navigation
		return stages

	def start(self):
		self.env.EnableInventory(4)
		self.env.Clear()
		self.env.CreateRandomGenerateScene()
		self.env.SetLighting({ 
			"ssr" : True, \
			"exposure" : 0.0, \
			"linear_voxelize" : True, \
			"indirect_strength" : 0.25 \
		})

		doors  = [door2, door3, door4, door5]
		keys   = [key2, key3, key4, key5]
		d_tags = ["red", "yellow", "blue", "purple"]
		models = [crate1, crate03, cat1]
		m_tags = ["large crate", "small crate", "cat"]

		self.env.MakeObjectPickable("small crate")
		self.env.LoadBasicObjects(doors, keys, d_tags, door1, wall_test, [floor_test])
		start = self.env.LoadSceneConfigure(6, 6, 3, 2)
		
		last_visit_room = self.env.GetRoomVisitSequence()[-1]
		last_visit_grp  = self.env.GetRoomGroups()[last_visit_room]
		
		print last_visit_grp

		conf   = {"stack"  : [crate03, crate1, last_visit_grp, 1, 3]}

		self.env.LoadModels(models, m_tags)
		self.env.SpawnModels(conf)

		self.agent = self.env.SpawnAnObject("husky/husky.urdf", \
			start, [-1,0,0,1.57], 0.6, "Agent", True)
		self.env.AttachCameraTo(self.agent, [0.3,1.3,0.0])
		self.env.Initialize()

		return "navigation"

	def navigation(self):

		objects = self.env.QueryObjectByLabel("small crate")
		for i in xrange(len(objects)):
			res = self.env.LocateObjectInGrid(objects[i])
			if len(res) > 0 and res[0] == 0:
				return "idle"

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

		image_rgbd_resize = cv2.resize(image_rgbd, None, fx=0.75, fy=0.75)
		image_rgb = np.array(image_rgbd_resize[:,:,:3])


		frames = "frames: " + str(self.env.GetStatus()["frames"])
		framerate = " | framerate: " + str(self.env.GetStatus()["framerate"])

		cv2.putText(image_rgb, frames + framerate, (30,30), \
    		cv2.FONT_HERSHEY_PLAIN, 1, (200,250,250), 1);

		cv2.imshow("RGB", image_rgb)
		# cv2.imshow("Depth", image_depth)
		return "navigation"

