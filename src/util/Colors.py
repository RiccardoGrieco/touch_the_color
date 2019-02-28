#!/usr/bin/env python

import numpy as np


class Colors:
    colorNames = []
    kinectValues = []

    colorNames.append('RED')
    colorNames.append('BLUE')
    colorNames.append('GREEN')
    colorNames.append('YELLOW')
    colorNames.append('MAGENTA')

    kinectValues.append((np.array([0,50,50]),np.array([15,255,255])))
    kinectValues.append((np.array([105,50,50]),np.array([135,255,255])))
    kinectValues.append((np.array([45,50,50]),np.array([75,255,255])))
    kinectValues.append((np.array([15,50,50]),np.array([45,255,255])))
    kinectValues.append((np.array([135,50,50]),np.array([165,255,255])))