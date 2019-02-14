#!/usr/bin/env python

from math import sqrt

class Vector2D:
    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y
        self.calculateInfo()

    def calculateInfo(self):
        self.intensity = sqrt(self.x**2 + self.y**2)
        if self.intensity!=0:
            self.versorX = self.x/self.intensity
            self.versorY = self.y/self.intensity
        else:
            self.versorX = 0
            self.versorY = 0
    
    def __add__(self, other):
        ret = Vector2D()
        ret.x = self.x + other.x
        ret.y = self.y + other.y
        ret.calculateInfo()
        return ret

    def __sub__(self, other):
        ret = Vector2D()
        ret.x = self.x - other.x
        ret.y = self.y - other.y
        ret.calculateInfo()
        return ret