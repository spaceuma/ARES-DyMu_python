# -*- coding: utf-8 -*-

#======================LOCAL PATH REPAIRING SCRIPT=============================
#   Case Example of combining Global Path Planning and Local Path Repairing
#          Author: J. Ricardo Sanchez Ibanez (ricardosan@uma.es)
#==============================================================================

import numpy as np
import matplotlib.pyplot as plt
import lib.slope_cost as sc
import lib.dymu as dymu
from time import time

# The DEM (elevationMap) and soilMap are read
elevationMap = np.loadtxt(open("maps/DECOS_elevationMap.csv", "rb"),\
                          delimiter=" ", skiprows=0)
soilMap = np.loadtxt(open("maps/DECOS_soilMap.csv", "rb"),\
                          delimiter=" ", skiprows=0)
globalRes = 1.0

# Getting CostMap
slopeThreshold = 30.0
costMap, obstacleMap, slopeMap = sc.getCostMap(elevationMap, soilMap,\
                                               globalRes, slopeThreshold)

# We set the goal
goal = [39,99]
start = [110,56]

# Total Cost Maps are computed
init = time()
Tmap = dymu.computeTmap(costMap, obstacleMap, goal, start)
print('Elapsed time of computing Total Cost Map: ' + str(time()-init))

# The Global Path is extracted
path = dymu.getPathGDM(Tmap,np.asarray(start),np.asarray(goal),0.4)



fig1, (ax1, ax2) = plt.subplots(1, 2, tight_layout=True)
#costMap1[np.isinf(costMap1)] = 0
pos1 = ax1.contourf(obstacleMap, cmap = 'Reds')
ax1.plot(goal[0],goal[1],'or')
ax1.plot(start[0],start[1],'or')
#ax1.plot(path[:,0],path[:,1],'r')
ax1.set_aspect('equal')
ax2.contourf(Tmap, 100, cmap = 'viridis', alpha = .5)
ax2.contour(Tmap, 100, cmap = 'viridis')
ax2.plot(goal[0],goal[1],'or')
ax2.plot(start[0],start[1],'or')
ax2.set_aspect('equal')
ax2.plot(path[:,0],path[:,1],'r')
plt.show()
