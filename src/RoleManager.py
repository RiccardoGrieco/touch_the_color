#!/usr/bin/env python

import rospy

class RoleManager:
    def __init__(self):
		rospy.init_node("role_manager", anonymous=True)
        #playersDictionary (name->ip)
        #myName
        #TODO first witch
        #TODO topic /role
        #TODO create communication node
        
    def endGame(self, msg):
        #TODO switch or re-initialize Role
        #TODO config communication node


    class Communication:
        #lets role nodes communicate with each other 

        def read(self, msg):
            #TODO

        def send(self, msg):
            #TODO

        def config(self, ...): #TODO params
            #TODO configure sockets

    