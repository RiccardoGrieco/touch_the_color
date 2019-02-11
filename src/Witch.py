#!/usr/bin/env python

import rospy

class Witch:
    def __init__(self):
        rospy.init_node("witch", anonymous=True)
        
    def listen(self, msg):
        #PS
        #TODO listen behaviour

    def determineEndGame(self):
        #B
        #TODO determine end game behaviour

    def communicateLoser(self):
        #MS
        #TODO broadcast 

    def config(self,*args): #TODO *args
        #TODO init function (if needed)

    def mainLoop(self):
        #TODO mainLoop