#!/usr/bin/env python

import rospy
import time
import random
import math
import numpy as np

from sys import maxint
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class FollowCorridor:

    def __init__(self):
        rospy.init_node("followCorridorrr", anonymous=True)
        self.velPub = rospy.Publisher("/RosAria/cmd_vel", Twist, queue_size=1)
        self.sonarSub = rospy.Subscriber("/RosAria/sonar", PointCloud, self.readSonar)
        self.poseSub = rospy.Subscriber("/RosAria/pose", Odometry, self.updatePose)
        self.sonarReading = PointCloud()
        self.RAD = pi / 180
        self.releaser = False
        self.pose = Odometry()
        self.distance = np.array([10 for i in range(0, 9)])
        self.iter = 0
        self.readIter = maxint
        self.canRead = True
        self.fieldsPoints = list()
        self.releaserCorridorFound = False

    def readSonar(self, msg):
        self.sonarReading = msg
        i = 0

        if self.canRead:
            del self.fieldsPoints[:]

            for p in self.sonarReading.points:
                # if 0 < i <= 6:
                # print("sto leggendo i sensori da 1 a 6.")
                self.distance[i] = math.sqrt(p.x * p.x + p.y * p.y)

                if self.distance[i] <= 5:
                    # consideriamo le letture fino a 5mt
                    if self.distance[i] <= 1:
                        if self.distance[i] == 0:
                            self.distance[i] = 0.001
                        self.fieldsPoints.append([(6 - self.distance[i]) * (-p.x / self.distance[i]),
                                                (6 - self.distance[i]) * (-p.y / self.distance[i])])
                        if self.distance[i] < 0.5:
                            # attiva releaser di avoid
                            self.releaser = True
            i = i + 1


    def lookForCorridor(self):
        ## TODO

        # percieve walls
        # left wall, sonar[0, 1, 14, 15]
        # left interpolation (use: scipy.stats.linregress(x, y))
        # right wall, sonar[6, 7, 8, 9]
        # right interpolation (use: scipy.stats.linregress(x, y))
        # compare walls: are they parallel?
        # activate releaser
        # save actual pose, it serves!
        # save straight lines
        # find heading (versore vettore direzione)
        return
        
    def followCorridor(self, force):
        ## TODO... it sussumes wander.
        # @param force: array of 2, is the potential field vector

        # calculate rotation matrix respect to self.startPose
        # heading force = heading * rotationMatrix * defaultValue(field intensity: it is constant)
        #
        # wall repulsion =
        return force

    def avoid(self):
        # TODO: deve cambiare
        maxWalkableDistance = 5
        force = [0, 0]
        for p in self.fieldsPoints:
            force = [force[0] + p[0], force[1] + p[1]]
        return force

    def originRectProjection(self, inclination, intercept):
        x = -intercept/(1/inclination + inclination)
        y = intercept/(1+ inclination**2)
        return x, y 

    def calculateCorridorFields(self):
        # given y1=m1x+q1 and y2=m2x+q2

        # calculate left wall force versor
        x,y = self.originRectProjection(self.inclination1, self.intercept1)
        magnitude = math.sqrt(x*x, y*y)
        wallDirectionX1 = - (x/magnitude)
        wallDirectionY1 = - (y/magnitude)

        # calculate right wall force versor
        x,y = originRectProjection(self.inclination2, self.intercept2)
        magnitude = math.sqrt(x*x, y*y)
        wallDirectionX2 = - (x/magnitude)
        wallDirectionY2 = - (y/magnitude)

        # calculate follow corridor field versor
        
        # find 2 left and 2 right wall points used for wall rect calculation
        #
        ##########################################################

        followX1 = leftX2 - leftX1
        followY1 = leftY2 - leftY1
        magnitude = math.sqrt(followX1**2 + followY1**2)
        followX1 = followX1/magnitude
        followY1 = followY1/magnitude

        followX2 = rightX2 - rightX1
        followY2 = rightY2 - rightY1
        magnitude = math.sqrt(followX2**2 + followY2**2)
        followX2 = followX2/magnitude
        followY2 = followY2/magnitude

        followCorridorX = (followX1 + followX2)
        followCorridorY = (followY1 + followY2)
        magnitude = math.sqrt(followCorridorX**2 + followCorridorY**2)
        followCorridorX = followCorridorX/magnitude
        followCorridorY = followCorridorY/magnitude


if __name__ == "__main__":
    try:
		robot = FollowCorridor()
		#print(robot.sonarAngles)
		robot.start()
    except rospy.ROSInterruptException:
		print("ROSInterruptException")
    except KeyboardInterrupt:
		print("KeyboardInterrupt")