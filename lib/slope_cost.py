# -*- coding: utf-8 -*-

#======================COST MAP CREATION SCRIPT=============================
#                  Creation of costmap based on slope
#          Author: J. Ricardo Sanchez Ibanez (ricardosan@uma.es)
#==============================================================================

import numpy as np
from bisect import bisect_left
import cv2

rad2deg = 180/np.pi

slopecostLUT = np.zeros((2,5))
slopecostLUT[:][0] = [0, 5, 10, 15, 20]
slopecostLUT[:][1] = [10.0, 12.0, 15.0, 20.0, 40.0]


def getCost(slope):
    if slope >= slopecostLUT[0][-1]:
        return 50.0
    index = bisect_left(slopecostLUT[0][:], slope)
    decimalPart = (slope - slopecostLUT[0][index-1])/(slopecostLUT[0][index] - slopecostLUT[0][index-1])
    cost = decimalPart*(slopecostLUT[1][index]-slopecostLUT[1][index-1]) + slopecostLUT[1][index-1]
    return cost



def getCostMap(elevationMap, soilMap, res, slopeThreshold):
    
    dX,dY = np.gradient(elevationMap,*[res, res])
    slopeMap = rad2deg*np.arctan(np.sqrt(dX**2+dY**2))
    slopeMap[np.isnan(slopeMap)] = 0
    
    costMap = np.ones_like(elevationMap)
    obstacleMap = np.zeros_like(elevationMap)
    
    obstacleMap[np.where(slopeMap > slopeThreshold)] = 1
    obstacleMap[soilMap == 0] = 1
    
    for i in range(slopeMap.shape[1]):
        for j in range(slopeMap.shape[0]):
            costMap[j][i] = getCost(slopeMap[j][i])
    
    costMap[obstacleMap == 1] = 2*slopeThreshold
    
    costMap = cv2.blur(costMap,(3,3))
    
    return costMap, obstacleMap, slopeMap