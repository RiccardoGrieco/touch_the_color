#!/usr/bin/env python

import rospy

class Witch:
    def __init__(self):
        rospy.init_node("Witch", anonymous=True)

        self.noWinners = 0  # when it's noTotKids-1 than the game ends
        self.noTotKids = 0

        # TODO gestire queste bestiole qui
        self.RMSpeakerPub = rospy.Publisher("node_to_rolemanager", String, queue_size=10)
        self.RMListenerSub = rospy.Subscriber("rolemanager_to_node", String, self.ownRoleManagerListener)

        self.start()

    def ownRoleManagerListener(self, msg):
        # manage messages FROM RoleManager reading from its subscriber

        # 0 (another Kid touched the color)
        # 1 (number of total players)

        if msg[0] == "0":
            self.noWinners += 1
        elif msg[0] == "1":
            self.noTotKids = int(msg[2:])

    def start(self):
        rate_start = rospy.Rate(1)
        rate_start.sleep()

        while not(rospy.is_shutdown()):
            # print("hey")
            i = 1

    def listen(self, msg):
        #PS
        #TODO listen behaviour
        i = 0

    def determineEndGame(self):
        #B
        #TODO determine end game behaviour
        i = 0

    def communicateLoser(self):
        #MS
        #TODO broadcast
        i = 0

    def config(self,*args):
        #TODO *args
        #TODO init function (if needed)
        i = 0

    def mainLoop(self):
        #TODO mainLoop
        i = 0


if __name__ == "__main__":
    rb = Witch()

