#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import time
from math import pi
from nav_msgs.msg import Odometry

class Wander:
    def __init__(self):
        rospy.init_node("wanderobot", anonymous=True)
        self.velPub = rospy.Publisher("/RosAria/cmd_vel", Twist, queue_size=1)
        self.poseSub = rospy.Subscriber("/RosAria/pose", Odometry, self.updatePose)
        self.fwdCmds = Twist()
        self.angCmds = Twist()
        self.pose = Odometry()
        self.loop = True
        self.fwdCmds.linear.x = 0.15

    def updatePose(self, msg):
        self.pose = msg

    def start(self):
        rate = rospy.Rate(100)
       	rate_start = rospy.Rate(1)
        rate_start.sleep() # attesa dell'inizializzazione dei nodi di ros
        self.wander()
        rate.sleep()

    def wander(self):
        self.velPub.publish(self.fwdCmds)
        time.sleep(2)
        self.velPub.publish(Twist())
        self.rotate(-0.15, 0.6)
        self.velPub.publish(self.fwdCmds)
        time.sleep(2)
        self.velPub.publish(Twist())
        self.rotate(0.15, 0.6)
        self.fwdCmds.linear.x = -0.15
        self.velPub.publish(self.fwdCmds)
        time.sleep(2)
        self.velPub.publish(Twist())

        

    def rotate(self, velocity, degree):
        startA = self.pose.pose.pose.orientation.z
        residuo = 0
        self.angCmds.angular.z = velocity
        while self.loop:
            if residuo >= degree:
                break
            # t = residuo/self.angCmds.angular.z
            time.sleep(1)
            residuo = abs(self.pose.pose.pose.orientation.z - startA)
            self.velPub.publish(self.angCmds)
            print(residuo)
        self.velPub.publish(Twist())

if __name__ == "__main__":
    try:
        wander = Wander()
        wander.start()
    except rospy.ROSInterruptException:
        print "ROSInterruptException"
    except KeyboardInterrupt:
        print "KeyboardInterrupt"
        wander.loop = False

    