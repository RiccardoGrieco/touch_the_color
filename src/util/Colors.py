#!/usr/bin/env python

import numpy as np


class Colors:
    colorNames = []
    kinectValues = []

    colorNames.append('BLUE')
    #colorNames.append('MAGENTA')
    colorNames.append('LIGHT BLUE')
    colorNames.append('LIGHT GREEN')

    kinectValues.append((np.array([115,50,50]),np.array([125,255,255])))
    #kinectValues.append((np.array([145,50,50]),np.array([160,255,255])))
    kinectValues.append((np.array([70, 50, 50]), np.array([110, 255, 255])))
    kinectValues.append((np.array([38, 50, 50]), np.array([48, 255, 255])))

def getColor(colorName):
    colorID = Colors.colorNames.index(colorName)
    return Colors.kinectValues[colorID]