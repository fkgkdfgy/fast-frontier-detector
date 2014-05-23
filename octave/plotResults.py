#!/usr/bin/env python

#import numpy as np
import os
from numpy import *
import numpy.numarray as na
import matplotlib.pyplot as plt

# scan all maps
maps = filter(os.path.isdir, os.listdir('.'))
xlabels = []
means = zeros(0)
stds = zeros(0)

if '.svn' in maps:
	maps.remove('.svn')

first = True

for currMap in maps:
	xlabels.append(currMap)
	dataFFD = genfromtxt(currMap + "/executions/partial_ffd_executions.txt")
	dataWFD = genfromtxt(currMap + "/executions/exploration_execution_rafael.txt")
	dataWolfram = genfromtxt(currMap + "/executions/exploration_execution_wolfram.txt")
	dataFFD = dataFFD[:,1]
	
	# check IF NEEDED CLIPPING!!!!!1
	# WFD works on a world smaller by 8X8 so factor it to 64
	dataWFD = 64*dataWFD
	print type(dataFFD)
	print dataFFD.shape
	print currMap
	print dataFFD.mean(), dataWFD.mean(), dataWolfram.mean()
	
	currMeans = concatenate(([],[dataFFD.mean(), dataWFD.mean(), dataWolfram.mean()]),0)
	currStds = concatenate(([],[dataFFD.std(), dataWFD.std(), dataWolfram.std()])) 
	
	if first:
		means = currMeans
		stds = currStds
		first = False
	else:	
		means = vstack((means, currMeans))
		stds = vstack((stds, currStds))
	#means = concatenate((means, currMeans),0)
		
	
	#stds = concatenate((stds, currStds),0)
	


ind = arange(len(means))
width = 0.25

fig = plt.figure()
ax = fig.add_subplot(111)

#plt.yscale('log')
offset = 0
rectList = []
colors = ['#fafad2', 'Cyan', 'Red']
for i in range(len(means[0])):
	rectList.append(ax.bar(ind+width*i , means[:,i], width, color=colors[i], yerr=stds[:,i], log=True))
	#offset += width

#for i in range(cols):
#	currBars = means[:,i]


# add some labels
ax.set_ylabel('logscale time (microseconds)')
ax.set_xticks(ind+width*1.5)
ax.set_xlabel('environments')
ax.set_xticklabels( ('(A)', '(B)', '(C)', '(D)', '(E)', '(F)'), fontsize=13 )
ax.grid(b=True,which='major')


def getFirst(x):
	return x[0]

ax.legend(map(getFirst, rectList), ('FFD', 'WFD', 'SOTA'))
plt.ylim(10**3,10 ** 9)
#plt.show()
plt.savefig('plot_Results.png')
plt.savefig('plot_Results.eps')

if 1==2:
	# plot mean graph
	xlocations = na.array(range(len(means)))+0.5
	plt.bar(xlocations, means, color='Cyan')
	plt.xlabel("particles")
	plt.ylabel("average run-time (microseconds)")
	plt.grid(b=True,which='major')

	plt.hold(True)

	# plot error bars
	plt.errorbar(range(1,len(means)+1),means,stds)

	import  matplotlib.pyplot as p
	p.xlim(0,len(means)+1)

	#plt.show()
	p.savefig("matan.png")

