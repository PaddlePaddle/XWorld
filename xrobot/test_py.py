import libxrobot
import cv2
import numpy as np

scene = libxrobot.Playground(640, 480, libxrobot.HEADLESS)
scene.EnableInventory(10)
scene.Clear()
scene.CreateAnTestScene()
scene.SetLighting()

scene.SpawnAnObject("./crate_0.3/crate.urdf", [5,0,0], [1,0,0,0], 1.0, "SmallCrate", False)
agent = scene.SpawnAnObject("husky/husky.urdf", [2,0,2], [-1,0,0,1.57], 1.0, "Agent", True)
scene.AttachCameraTo(agent, [0.3,1.3,0.0])
scene.Initialize()

while True:

	scene.Update()

	# Fetch Agent Status
	position = agent.GetPosition()
	orientation = agent.GetOrientation()

	# Fetch Raw Images
	image_str = scene.GetCameraRGBDRaw()
	image_rgbd = np.fromstring(image_str, np.uint8).reshape( 480, 640, 4 )
	image_rgbd = cv2.cvtColor(image_rgbd, cv2.COLOR_BGRA2RGBA)
	image_rgbd = cv2.flip(image_rgbd, 0)

	image_rgb = image_rgbd[:,:,:3]
	image_depth = image_rgbd[:,:,3]

	cv2.imshow("RGB", image_rgb)
	cv2.imshow("Depth", image_depth)

	key = cv2.waitKey(1)

	if key == 27: # ESC
		break
	elif key == 119: # W
		scene.MoveForward(10)
	elif key == 97:  # A
		scene.TurnLeft(10)
	elif key == 115: # S
		scene.MoveBackward(10)
	elif key == 100: # D
		scene.TurnRight(10)
	elif key == 48: # 9
		scene.Grasp()
	elif key == 57: # 0
		scene.PutDown()
	elif key == 54: # kp6
		scene.LookUp()
	elif key == 51: # kp3
		scene.LookDown()