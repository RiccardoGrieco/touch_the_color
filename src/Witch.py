#!/usr/bin/env python

import rospy
from random import randint
from util.Colors import Colors
from std_msgs.msg import String


class Witch:

    def __init__(self):
        rospy.init_node("Witch", anonymous=True)

        self.noWinners = 0  # when it's noTotKids-1 than the game ends
        self.noTotKids = 0

        # TODO gestire queste bestiole qui
        self.RMSpeakerPub = rospy.Publisher("node_to_rolemanager", String, queue_size=10)
        self.RMListenerSub = rospy.Subscriber("rolemanager_to_node", String, self.ownRoleManagerListener)

        color = self.chooseColor()
        self.ownRoleManagerSpeaker(0, color)

        rospy.spin()

    def chooseColor(self):
        colorIndex = randint(0, len(Colors.colorNames) - 1)
        return Colors.colorNames[colorIndex]

    def ownRoleManagerSpeaker(self, typeOfMsg, color):
        """

        :param typeOfMsg:
        :param color:
        :return:
        """

        if typeOfMsg == 0:
            self.RMSpeakerPub.publish("0:" + color)     # tell to RM the color I decided

        elif typeOfMsg == 1:
            self.RMSpeakerPub.publish("1")              # tell to RM that the game ends

    def ownRoleManagerListener(self, msg):
        # manage messages FROM RoleManager reading from its subscriber

        # 0 (another Kid touched the color)
        # 1 (number of total players)

        if msg[0] == "0":
            self.noWinners += 1
            if self.determineEndGame():
                self.ownRoleManagerSpeaker(1, "")

        elif msg[0] == "1":
            self.noTotKids = int(msg[2:])

    def listen(self, msg):
        #PS
        #TODO listen behaviour
        i = 0

    def determineEndGame(self):
        """
        Test if the game ends.
        :return: True if the game ends; False otherwise.
        """

        if self.noWinners == self.noTotKids - 1:    # end-game test
            return True
        else:
            return False

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

