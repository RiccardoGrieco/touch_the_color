#!/usr/bin/env python

from math import sqrt

class Vector2D:
    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y
        self.infoCalculated = False

    def calculateInfo(self):
        self.intensity = sqrt(self.x**2 + self.y**2)
        if self.intensity!=0:
            self.versorX = self.x/self.intensity
            self.versorY = self.y/self.intensity
        else:
            self.versorX = 0
            self.versorY = 0
        self.infoCalculated = True
    
    def __add__(self, other):
        return Vector2D(self.x+other.x, self.y+other.y)

    def __sub__(self, other):
                return Vector2D(self.x-other.x, self.y-other.y)

    def __str__(self):
        formatParameters = (self.x, self.y, self.getIntensity(), self.getVersorX(), self.getVersorY())
        return "Vector:(%.5f,%.5f) | Intensity: %.5f | Versor:(%.5f,%.5f)" % formatParameters

    def getIntensity(self):
        if not(self.infoCalculated):
            self.calculateInfo()
        return self.intensity
    
    def getVersorX(self):
        if not(self.infoCalculated):
            self.calculateInfo()
        return self.versorX
    
    def getVersorY(self):
        if not(self.infoCalculated):
            self.calculateInfo()
        return self.versorY

    def multiplyMatrix(self, matrix):
        x = self.x*matrix[0][0] + self.y*matrix[0][1]
        y = self.x*matrix[1][0] + self.y*matrix[1][1]
        ret = Vector2D(x,y)
        return ret