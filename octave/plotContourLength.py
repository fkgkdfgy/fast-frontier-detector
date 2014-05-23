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
allLengths = array([])
beans = 100
listLengths = []

if '.svn' in maps:
	maps.remove('.svn')
maps.remove('simulation.clf')
for currMap in maps:
	contours = genfromtxt(currMap + "/contours/contour_lengths.txt")
	lengths = contours[:,0]
	times = contours[:,1]	

	maxSize = max(len(allLengths), len(lengths));
	listLengths.append(lengths)		
	'''
	if len(allLengths) < maxSize:
		
		s = allLengths.shape;
		r = s[0]
		c = 1
		
		if len(s) == 1:
			allLengths = lengths
		else:
			c = s[1]
			allLengths = array([allLengths, zeros( (maxSize - r, c))])
			allLengths = array([allLengths, lengths])
	else:
		print maxSize, len(lengths)
		print allLengths
		tempLengths = array([lengths, zeros( (maxSize - len(lengths),1) )])
		allLengths  = array([allLengths, tempLengths])
	'''		
	# sort rows
	sortedContours = contours[contours[:,0].argsort(),]
	lengths = sortedContours[:,0]
	times = sortedContours[:,1]

	# plot scatter graph of contour lengths
	fig = plt.figure()
	ax = fig.add_subplot(111)
	ax.scatter(lengths, times)
	plt.xlim(0,200000)
	plt.ylim(0,150000)

	# calculate correlation
	import scipy.stats
	rho,p = scipy.stats.pearsonr(lengths, times)

	# add some labels
	ax.set_ylabel('run-time (microseconds)')
	ax.set_xlabel('contour lengths')
	ax.set_title('Pearson = ' + str(rho))
	ax.grid(b=True,which='major')

	#plt.show()
	plt.savefig(currMap + '/executions/plot_scatter.png')

	# plot hist graph of contour lengths
	fig = plt.figure()
	ax = fig.add_subplot(111)
	ax.hist(lengths, beans)
	plt.xlim(0,200000)
	plt.ylim(0,10)

	# add some labels
	ax.set_ylabel('count')
	ax.set_xlabel('contour lengths')
	ax.grid(b=True,which='major')
	ax.set_title('Histogram beans = ' + str(beans))

	#plt.show()
	plt.savefig(currMap + '/executions/plot_hist.png')


# plot all contour histogram


fig = plt.figure()
ax = fig.add_subplot(111)
beans = 10

# build histogram matrix
maxSize = max(map(len, listLengths))

data = zeros((maxSize, maxSize))
print maxSize
print data.shape
#for item in listLengths:
for i in range(len(listLengths)):
	currLen = len(listLengths[i])
	data[:currLen,i] = listLengths[i]
	#ax.hist(item, beans)
	#plt.xlim(0,200000)

# add some labels
data = data.transpose()
ax.hist(data, bins=beans, label=maps)
ax.set_ylabel('count')
ax.set_xlabel('contour lengths')
ax.grid(b=True,which='major')
plt.xlim((30000,35000))
plt.ylim((0,5))
ax.set_title('All Contours Histogram beans = ' + str(beans))
ax.legend()
plt.savefig('plot_all_contours_hist.png')
