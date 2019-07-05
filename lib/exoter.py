# -*- coding: utf-8 -*-

#======================COST MAP CREATION SCRIPT=============================
#        Creation of costmap based on ExoTeR simulation model data
#          Author: J. Ricardo Sanchez Ibanez (ricardosan@uma.es)
#==============================================================================

import numpy as np
import cv2

def getCostMap(elevationMap, soilMap):
    
    costMap = np.ones_like(elevationMap)
    obstacleMap = np.zeros_like(elevationMap)
    
    obstacleMap[soilMap == 0] = 1
    
    costMap[soilMap == 0] = 2*0.472
    
    costMap[soilMap == 1] = 0.088
    
    costMap[soilMap == 2] = 0.236
    
    costMap[soilMap == 3] = 0.472
    
    costMap = cv2.blur(costMap,(3,3))
    
    return costMap, obstacleMap

def getCostMapWOWW(elevationMap, soilMap):
    costMap = np.ones_like(elevationMap)*np.inf
    obstacleMap = np.zeros_like(elevationMap)
    
    obstacleMap[soilMap == 0] = 1
    
    costMap[soilMap == 0] = 2*2.148
    
    costMap[soilMap == 1] = 0.088
    
    costMap[soilMap == 2] = 1.074
    
    costMap[soilMap == 3] = 2.148
    
    costMap = cv2.blur(costMap,(3,3))
    
    return costMap, obstacleMap