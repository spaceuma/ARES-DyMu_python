# -*- coding: utf-8 -*-

#======================COST MAP CREATION SCRIPT=============================
#        Creation of costmap based on ExoTeR simulation model data
#          Author: J. Ricardo Sanchez Ibanez (ricardosan@uma.es)
#==============================================================================

import numpy as np

def getCostMap(elevationMap, soilMap):
    
    costMap = np.ones_like(elevationMap)*np.inf
    locMap = np.zeros_like(elevationMap)
    
    costMap[soilMap == 1] = 0.088
    locMap[soilMap == 1] = 1
    
    costMap[soilMap == 2] = 0.236
    locMap[soilMap == 2] = 2
    
    costMap[soilMap == 3] = 0.472
    locMap[soilMap == 3] = 1
    
    return costMap, locMap

def getCostMapWOWW(elevationMap, soilMap):
    costMap = np.ones_like(elevationMap)*np.inf
    locMap = np.zeros_like(elevationMap)
    
    costMap[soilMap == 1] = 0.088
    locMap[soilMap == 1] = 1
    
    costMap[soilMap == 2] = 1.074
    locMap[soilMap == 2] = 0
    
    costMap[soilMap == 3] = 2.148
    locMap[soilMap == 3] = 1
    
    return costMap, locMap