from libxrobot import *
import multiprocessing as mp
import argparse
import numpy as np
import random
import time
import os
import os.path

suncg_dir    = "../../data/suncg";
suncg_meta   = suncg_dir + "/ModelCategoryMapping.csv";
suncg_house0 = suncg_dir + "/house/7c16efebdfe46f3f14fa81abe500589c/house.json";

def worker(device, id):
	env = Playground(args.w, args.h, 1, args.quality, device)

	env.CreateSceneFromSUNCG()
	env.LoadSUNCG(suncg_house0, suncg_meta, suncg_dir)

	agent = env.SpawnAnObject("husky/husky.urdf", \
		[-6,0.1,-1], [-1,0,0,1.57], 1.0, "Agent", True)

	env.AttachCameraTo(agent, [0.0,1.3,0.0])
	env.Initialize()
	env.Update()

	start = time.time()

	for i in xrange(args.num_iter):
		env.UpdateRenderer()
		if args.simulation:
			env.UpdateSimulationWithAction(NO_ACTION)
		image_str = env.GetCameraRGBDRaw()

	end = time.time()

	print "\ngpu: " + str(device) + \
		", id: " + str(id) + \
		", fps: " + str(int(args.num_iter / (end - start)))

if __name__ == '__main__':

	parser = argparse.ArgumentParser()
	parser.add_argument('--num-proc', type=int, default=1)
	parser.add_argument('--num-gpu', type=int, default=1)
	parser.add_argument('--w', type=int, default=120)
	parser.add_argument('--h', type=int, default=90)
	parser.add_argument('--num-iter', type=int, default=5000)
	parser.add_argument('--quality', type=int, default=1)
	parser.add_argument('--simulation', type=int, default=1)
	args = parser.parse_args()

	procs = []
	for i in xrange(args.num_proc):
		device = i % args.num_gpu
		procs.append(mp.Process(target=worker, args=(device, i)))

	for p in procs:
		p.start()

	for p in procs:
		p.join()