#!/usr/bin/env python

import rospy
import time
from random import randint
from util.Colors import Colors
from std_msgs.msg import String


class Witch:

    def __init__(self):
        rospy.init_node("Witch", anonymous=True)

        self.noWinners = 0  # when it's noTotKids-1 than the game ends
        self.noTotKids = 0

        self.RMSpeakerPub = rospy.Publisher("node_to_rolemanager", String, queue_size=10)
        self.RMListenerSub = rospy.Subscriber("rolemanager_to_node", String, self.ownRoleManagerListener)

    def start(self):
        color = self.chooseColor()
        self.ownRoleManagerSpeaker(0, color)

        rospy.spin()

    def chooseColor(self):
        # colorIndex = randint(0, len(Colors.colorNames) - 1)
        colorIndex = Colors.colorNames.index("LIGHT GREEN")     # TODO: temporaneo, da togliere
        return Colors.colorNames[colorIndex]

    def ownRoleManagerSpeaker(self, typeOfMsg, color):
        """

        :param typeOfMsg:
        :param color:
        :return:
        """

        if typeOfMsg == 0:
            time.sleep(3)
            print("scrivo topic: " + str(typeOfMsg) + ": " + color)
            self.RMSpeakerPub.publish("0:" + color)     # tell to RM the color I decided

        elif typeOfMsg == 1:
            print("scrivo topic: fine gioco")
            self.RMSpeakerPub.publish("1")              # tell to RM that the game ends

    def ownRoleManagerListener(self, msg):
        # manage messages FROM RoleManager reading from its subscriber

        # 0 (another Kid touched the color)
        # 1 (number of total players)

        msg = msg.data

        if msg[0] == "0":
            self.noWinners += 1
            if self.determineEndGame():
                self.ownRoleManagerSpeaker(1, "")

        elif msg[0] == "1":
            print("leggo topic: " + "noTotKids = " + msg)
            self.noTotKids = int(msg[2:])

    def determineEndGame(self):
        """
        Test if the game ends.
        :return: True if the game ends; False otherwise.
        """

        print("noWinners: " + str(self.noWinners) + ", noTotKids: " + str(self.noTotKids))

        # TODO scegliere come determinare la fine del gioco
        # if self.noWinners == self.noTotKids - 1:    # end-game test
        if self.noWinners == self.noTotKids:
            print("ho determinato la fine del gioco")
            return True
        else:
            print("colore toccato ma la partita non e' finita")
            return False


if __name__ == "__main__":
    try:
        robot = Witch()

        robot.start()
    except rospy.ROSInterruptException:
        print("ROSInterruptException")
    except KeyboardInterrupt:
        print("KeyboardInterrupt")

