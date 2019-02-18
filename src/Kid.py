#!/usr/bin/env python

import rospy


class Kid:
    def __init__(self):
        rospy.init_node("Kid", anonymous=True)

        self.gameStarted = False
        self.colorToTouch = ""

        # TODO gestire queste bestiole qui
        self.RMSpeakerPub = rospy.Publisher("node_to_rolemanager", String, queue_size=10)
        self.RMListenerSub = rospy.Subscriber("rolemanager_to_node", String, self.ownRoleManagerListener)

        self.start()

    def ownRoleManagerListener(self, msg):
        # manage messages FROM RoleManager reading from its subscriber

        # 0:color (color to touch)
        # 1 (go!)

        if msg[0] == "0":
            self.colorToTouch = msg[2:]

        elif msg[0] == "1":
            i = 0
            self.gameStarted = True

    def ownRoleManagerSpeaker(self, typeOfMess):
        # manage messages TO RoleManager writing on its publisher

        # 0 (color touched)

        if typeOfMess == 0:
            self.RMSpeakerPub.publish("0")      # color touched

    def start(self):
        rate_start = rospy.Rate(1)
        rate_start.sleep()

        while not(rospy.is_shutdown()):
            # print("hey")
            i = 1

#    def look(self, msg):
        #PS
        #TODO look perception scheme
        # if time elapsed then ...

#    def moveToColor(self): #TODO params
        #B
        #TODO move to color behaviour

#    def wander(self): #TODO params
        #B
        #TODO wander behaviour

#    def avoid(self): #TODO params
        #B
        #TODO avoid obstacle behaviour

#    def move(self): #TODO params
        #MS
        #TODO move motor schema

#    def integrate(self): #TODO params
        #B
        #TODO manage objects position after movement

#    def feelForce(self): #TODO params
        #PS
        #TODO calculate overall external force on robot
    
#    def readSensors(self, msg):
        #PS
        #TODO read ultrasonic sensor information
    
#    def listen(self, msg):
        #PS
        #TODO listen for messages

#    def determineEndGame(self): #TODO params
        #B
        #TODO determine end game behaviour

#   def communicateColorTouched(self): #TODO params
        #MS
        #TODO communicate that the color has been touched


if __name__ == "__main__":
    rb = Kid()
