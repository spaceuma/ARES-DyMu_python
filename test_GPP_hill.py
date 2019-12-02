# -*- coding: utf-8 -*-

#======================GLOBAL PATH PLANNING TEST===============================
#                    Arriving to a Hill with obstacles
#          Author: J. Ricardo Sanchez Ibanez (ricardosan@uma.es)
#==============================================================================

import numpy as np
import matplotlib.pyplot as plt
import lib.dymu as dymu
from scipy import ndimage


costMap = np.ones((100,100))
proximityCostMap = np.zeros_like(costMap)
obstacleMap = np.zeros_like(costMap)
obstacleMap[0,:] = 1
obstacleMap[-1,:] = 1
obstacleMap[:,0] = 1
obstacleMap[:,-1] = 1



r = int(5)
r = r + 1 - r%2
y,x = np.ogrid[-r: r+1, -r: r+1]
convMatrix = x**2+y**2 <= r**2
convMatrix = convMatrix.astype(float)
obstacleMap[40,40] = 1
obstacleMap[50,35] = 1
#obstacleMap[50,35] = 1
obstacleMap = ndimage.morphology.binary_dilation(obstacleMap, structure = convMatrix).astype(obstacleMap.dtype)


proximityMap = ndimage.morphology.distance_transform_edt(1-obstacleMap)

# We create a conic hill
for j in range(30,71):
    for i in range(30,71):
        dist = np.linalg.norm([i-50,j-50])
        if dist <= 20.0:
            costMap[j,i] = 0.5# + 0.025*dist#2.1/(0.1*dist+0.05)
#        if dist <= 15.0:
#            costMap[j,i] = 10
   
#obstacleMap[50,50] = 0        
            
costMap[np.where(proximityMap == 0.0)] = 200


proximityCostMap[:] = costMap

proximityCostMap[np.where(proximityMap <= 3.0)] = 40.0/proximityMap[np.where(proximityMap <= 3.0)]

costMap = np.maximum(costMap, proximityCostMap)
            
fig, axes = plt.subplots(constrained_layout=True)
cc = axes.contourf(costMap, 100, cmap = 'nipy_spectral')
cbar = fig.colorbar(cc)
cbar.set_label('Anisotropy')
axes.set_aspect('equal')
plt.show()

start = [20,20]
goal = [50,50]

Tmap = dymu.computeTmap(costMap, obstacleMap, goal, [])
path = dymu.getPathGDM(Tmap,np.asarray(start),np.asarray(goal),0.5)
fig, axes = plt.subplots(constrained_layout=True)
cc = axes.contourf(Tmap, 100, cmap = 'nipy_spectral',alpha = .5)
axes.contour(Tmap, 100, cmap = 'nipy_spectral')    
axes.plot(path[:,0],path[:,1],'r')   