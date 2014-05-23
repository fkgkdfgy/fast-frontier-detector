#!/usr/bin/env python

#import numpy as np
from numpy import *
import numpy.numarray as na
import matplotlib.pyplot as plt

# s = "/home/matan/workspace/Gmapping_Original/trunk/bin/results_3_01_12/ubremen-cartesium-demo2.gfs.log/executions/partial_ffd_executions.txt"
data = genfromtxt("partial_ffd_executions.txt")

# get data
times = data[:,4:]
rows,cols = times.shape
means = times.mean(axis=0)
stds = times.std(axis=0)

# plot mean graph
xlocations = na.array(range(len(means)))+0.5
plt.bar(xlocations, means, color='Cyan', yerr=stds)
plt.xlabel("particles")
plt.ylabel("average run-time (microseconds)")
plt.grid(b=True,which='major')

import  matplotlib.pyplot as p
p.xlim(0,len(means)+1)


import os
mapName = os.getcwd().split('/')
mapName = mapName[len(mapName)-1]

#plt.show()
p.savefig("plot_particle_times_" + mapName + ".png")
p.savefig("plot_particle_times_" + mapName + ".eps")
