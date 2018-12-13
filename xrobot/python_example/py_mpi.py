from mpi4py import MPI
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

comm = MPI.COMM_WORLD
rank = comm.Get_rank()
size = comm.Get_size()

if __name__ == '__main__':

	parser = argparse.ArgumentParser()
	parser.add_argument('--w', type=int, default=120)
	parser.add_argument('--h', type=int, default=90)
	parser.add_argument('--quality', type=int, default=0)
	args = parser.parse_args()

	from libxrobot import *

	device = rank % 2;

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

	for i in xrange(1000):

		if (rank % 2 == 1):                                                              
			comm.Barrier()  

		# print "rank %d rendering %d begin" % (rank, i)
		env.UpdateRenderer()
		env.UpdateSimulationWithAction(NO_ACTION)
		env.GetCameraRGBDRaw()
		# print "rank %d rendering %d end" % (rank, i)

		comm.Barrier()

		# print "rank %d computing %d begin" % (rank, i)
		time.sleep(0.1)
		# print "rank %d computing %d end" % (rank, i)

		comm.Barrier()

		if (rank % 2 == 0):                                                              
			comm.Barrier() 

	end = time.time()

	print "gpu: " + str(device) + \
	", rank: " + str(rank) + \
	", fps: " + str(end - start)