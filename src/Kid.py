#!/usr/bin/env python

import rospy

class Kid:
    def __init__(self):
		rospy.init_node("kid", anonymous=True)

    def look(self, msg):
        #PS
        #TODO look perception scheme
        # if time elapsed then ...

    def moveToColor(self): #TODO params
        #B
        #TODO move to color behaviour

    def wander(self): #TODO params
        #B
        #TODO wander behaviour

    def avoid(self): #TODO params
        #B
        #TODO avoid obstacle behaviour

    def move(self): #TODO params
        #MS
        #TODO move motor schema

    def integrate(self): #TODO params
        #B
        #TODO manage objects position after movement

    def feelForce(self): #TODO params
        #PS
        #TODO calculate overall external force on robot
    
    def readSensors(self, msg):
        #PS
        #TODO read ultrasonic sensor information
    
    def listen(self, msg):
        #PS
        #TODO listen for messages

    def determineEndGame(self): #TODO params
        #B
        #TODO determine end game behaviour

    def communicateColorTouched(self): #TODO params
        #MS
        #TODO communicate that the color has been touched 