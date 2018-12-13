from libxrobot import *
import multiprocessing as mp
import argparse
import numpy as np
import random
import time
import os
import os.path

'''
The benchmark requires SUNCG OBJ+MTL instead of JSON.
Please use the OBJ+MTL conversion tool which is provided by SUNCGtoolbox

SUNCGtoolbox
https://github.com/shurans/SUNCGtoolbox

Please check the link below for the conversion tool.
https://github.com/shurans/SUNCGtoolbox#convert-to-objmtl
'''

suncg_dir   = "../data/suncg";
suncg_meta  = suncg_dir + "/metadata/ModelCategoryMapping.csv";
suncg_house = suncg_dir + "/house/00065ecbdd7300d35ef4328ffe871505/house.obj";

def worker(device, id):
	env = Playground(args.w, args.h, 1, args.quality, device)

	env.CreateEmptyScene()

	house = env.SpawnAnObject(suncg_house, \
		[-40,0,-40], [1,0,0,0], 1.0, "House", True)

	agent = env.SpawnAnObject("husky/husky.urdf", \
		[2,0,2], [-1,0,0,1.57], 1.0, "Agent", True)

	env.AttachCameraTo(agent, [0.3,1.3,0.0])
	env.Initialize()
	env.Update()

	start = time.time()

	for i in xrange(args.num_iter):
		env.UpdateRenderer()
		if args.simulation:
			env.UpdateSimulationWithAction(NO_ACTION)
		image_str = env.GetCameraRGBDRaw()
		time.sleep(0.1)

	end = time.time()

	print "gpu: " + str(device) + \
		", id: " + str(id) + \
		", fps: " + str(int(args.num_iter / (end - start)))

if __name__ == '__main__':

	parser = argparse.ArgumentParser()
	parser.add_argument('--num-proc', type=int, default=1)
	parser.add_argument('--num-gpu', type=int, default=1)
	parser.add_argument('--w', type=int, default=120)
	parser.add_argument('--h', type=int, default=90)
	parser.add_argument('--num-iter', type=int, default=2000)
	parser.add_argument('--quality', type=int, default=0)
	parser.add_argument('--simulation', type=int, default=1)
	args = parser.parse_args()

	procs = []
	for i in xrange(args.num_proc):
		device = i % args.num_gpu
		procs.append(mp.Process(target=worker, args=(device, i)))

	print "\n"

	start = time.time()

	for p in procs:
		p.start()

	for p in procs:
		p.join()

	end = time.time()

	print str(end - start)