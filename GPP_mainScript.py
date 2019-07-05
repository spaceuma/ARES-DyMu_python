# -*- coding: utf-8 -*-

#======================GLOBAL PATH PLANNING SCRIPT=============================
#     Case Example where Global Path Planning is used on two cost maps
#          Author: J. Ricardo Sanchez Ibanez (ricardosan@uma.es)
#==============================================================================

import numpy as np
import matplotlib.pyplot as plt
import lib.exoter as exoter
import lib.dymu as dymu
from time import time

# The DEM (elevationMap) and soilMap are read
elevationMap = np.loadtxt(open("maps/DECOS_elevationMap.csv", "rb"),\
                          delimiter=" ", skiprows=0)

soilMap = np.loadtxt(open("maps/DECOS_soilMap.csv", "rb"),\
                          delimiter=" ", skiprows=0)

# Getting CostMaps with and without Wheel-walking mode
costMap1, obstacleMap = exoter.getCostMap(elevationMap, soilMap)
costMap2, obstacleMap = exoter.getCostMapWOWW(elevationMap, soilMap)

# We set the goal
goal = [87,59]

# Total Cost Maps are computed
init = time()
Tmap1 = dymu.computeTmap(costMap1, obstacleMap, goal, [])
print('Elapsed time of computing Total Cost Map 1: ' + str(time()-init))
init = time()
Tmap2 = dymu.computeTmap(costMap2, obstacleMap, goal, [])
print('Elapsed time of computing Total Cost Map 2: ' + str(time()-init))


start = [(60,110), (39,99), (90,96), (57,56), (110,56)]


fig1, (ax1, ax2) = plt.subplots(1, 2, tight_layout=True)
#costMap1[np.isinf(costMap1)] = 0
pos1 = ax1.contourf(costMap1, cmap = 'Reds')
ax1.plot(goal[0],goal[1],'or')
ax1.set_aspect('equal')
ax2.contourf(costMap2, cmap = 'Reds')
ax2.plot(goal[0],goal[1],'or')
ax2.set_aspect('equal')
plt.show()

fig2, (ax3, ax4) = plt.subplots(1, 2, tight_layout=True)
ax3.contourf(Tmap1, 100, cmap = 'viridis', alpha = .5)
ax3.contour(Tmap1, 100, cmap = 'viridis')
ax3.plot(goal[0],goal[1],'or')
for i,s in enumerate(start):
    ax3.plot(start[i][0],start[i][1],'or')
    path = dymu.getPathGDM(Tmap1,np.asarray(start[i]),np.asarray(goal),0.5)
    ax3.plot(path[:,0],path[:,1],'r')
ax3.set_aspect('equal')
ax4.contourf(Tmap2, 100, cmap = 'viridis', alpha = .5)
ax4.contour(Tmap2, 100, cmap = 'viridis')
ax4.plot(goal[0],goal[1],'or')
for i,s in enumerate(start):
    ax4.plot(start[i][0],start[i][1],'or')
    path = dymu.getPathGDM(Tmap2,np.asarray(start[i]),np.asarray(goal),0.5)
    ax4.plot(path[:,0],path[:,1],'r')
ax4.set_aspect('equal')
plt.show()
