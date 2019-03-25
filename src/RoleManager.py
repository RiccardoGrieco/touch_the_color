#!/usr/bin/env python


import rospy
import socket
import threading
import roslaunch
import roslib
import select
import time
from std_msgs.msg import String
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
import sys


class RoleManager:

    PORT = 65535
    RECEIVE_SOCKET_TIMEOUT = 3
    CONNECTION_ATTEMPT_TIMEOUT = 5

    def __init__(self):
        """
        The rosparam ipList MUST be setted as list of strings ("['127.0.0.1', '127.0.1.1']".
        In ipList there's also my ip address.
        """

        roslib.load_manifest('sound_play')      # for audio
        rospy.init_node("role_manager", anonymous=True)
        self.myIPAddress = rospy.get_param("/myIPAddress")
        self.ipList = rospy.get_param("/ipList")                        # list of ip (me included)
        self.nodeListenerSub = rospy.Subscriber("node_to_rolemanager", String, self.ownNodeListener)
        self.nodeSpeakerPub = rospy.Publisher("rolemanager_to_node", String, queue_size=10)

        self.role = False                       # role = True for Witch, role = False for Kid
        self.witchIPAddress = min(self.ipList)  # the first witch

        self.host = None
        self.port = self.PORT
        self.sock = None
        self.launch = None
        self.myThread = []
        self.conn = []      # list of socket objects
        self.address = []   # list of other players' ip addresses
        self.topicHandlers = []
        self.winnersIPAddress = []
        self.stopThreads = False
        self.noConnected = 0
        self.listenSock = None
        self.listenSockSetted = False
        self.socketKidToWitch = None

        # handlers to methods that manages sockets sends
        self.WITCH_COLOR_HANDLERS = [self.tellColorBySocket, self.tellEndGameBySocket]
        self.KID_COLOR_HANDLERS = [self.tellColorTouchedBySocket]

        # handlers to methods that manages the received socket messages
        self.handlers = [self.manageColorMsg,
                         self.manageColorTouchedMsg,
                         self.manageEndGameMsg]

        self.soundclient = SoundClient()

        self.config()

        rospy.spin()        # in loop until is killed

    def tellColorBySocket(self, color):
        """
        Only Witch call this method.
        Send to Kids' RM the chosen color.
        :param color: the chosen color
        """

        msg = "0:" + color + "|"
        noPartecipants = len(self.ipList)-1
        while self.noConnected < noPartecipants:
            time.sleep(0.1)

        self.soundclient.say("color")
        self.soundclient.say("color")
        self.soundclient.say(color)

        for c in self.conn:
            c.send(msg)

    def tellEndGameBySocket(self):
        """
        Only Witch call this method.
        Send to Kid's RM the end of the game and loser's IP address.
        """

        last = self.winnersIPAddress[-1]
        self.winnersIPAddress.append(self.myIPAddress)
        # rescue ip in ipList but not in winnersIPAddress
        nextWitchIP = [ip for ip in self.ipList if ip not in self.winnersIPAddress]
        # if len(nextWitchIP) < 1:
        nextWitchIP.append(last)

        msg = "2:" + str(nextWitchIP[0]) + "|"

        for c in self.conn:
            c.send(msg)

        self.soundclient.say("Game over")

        self.manageEndGameMsg([nextWitchIP[0]])

    def tellColorTouchedBySocket(self):
        """
        Only Kid call this method.
        Send to Witch's RM that the color has been touched.
        """
        msg = "1:" + self.myIPAddress + "|"

        self.socketKidToWitch.sendall(msg)

        self.soundclient.say("I have touched the color")

    def createAndStartConnection(self):
        """
        Create and start socket connection.
        :return False if connection doesn't succeed, True otherwise.
        """

        self.host = self.witchIPAddress
        self.stopThreads = False

        if self.myIPAddress == self.witchIPAddress:

            if not self.listenSockSetted:
                self.listenSock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.listenSock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                self.listenSock.bind((self.host, self.port))
                self.listenSockSetted = True

            self.listenSock.listen(len(self.ipList) - 1)
            for i in range(1, len(self.ipList)):
                conn, address = self.listenSock.accept()
                conn.setblocking(0)
                self.conn.append(conn)
                self.address.append(address)
                thread = threading.Thread(target=self.manageSocket, args=[conn])
                self.myThread.append(thread)
                thread.start()
                self.noConnected = self.noConnected+1

                print("Connected to client ", address, ".")
            print("Connected to ALL clients")
        else:

            try:
                self.socketKidToWitch = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.socketKidToWitch.connect((self.host, self.port))
            except Exception as e:
                print(str(e))
                return False

            self.socketKidToWitch.setblocking(0)
            socketThread = threading.Thread(target=self.manageSocket, args=[self.socketKidToWitch])

            self.myThread.append(socketThread)
            socketThread.start()

            print("Connected to server.")

        return True

    def manageSocket(self, threadSocket):
        """
        Manage socket messages.
        :param threadSocket: the socket to listen.
        """

        size = 1024
        while not self.stopThreads:
            try:
                ready = select.select([threadSocket], [], [], self.RECEIVE_SOCKET_TIMEOUT)

                if ready[0]:
                    data = threadSocket.recv(size)[:-1]

                    if data:

                        for msg in data.split("|"):
                            params = msg.split(":")                    # param[0]=type of msg; param[1]=msg
                            self.handlers[int(params[0])](params[1:])   # call the method that msg refers to
                    else:
                        print('Disconnected.')
                        break
            except Exception as e:
                print(str(e))
                exit(1)
                break
        threadSocket.shutdown(socket.SHUT_RDWR)
        threadSocket.close()

    def manageColorMsg(self, args):
        """
        Only Kid call this method.
        Write on topic the color to touch.
        :param args: args[0]=color name (uppercase).
        """

        color = args[0]
        self.ownNodeSpeaker(0, color)

    def manageColorTouchedMsg(self, args):
        """
        Only Witch call this method.
        :param args: args[0]=Kid's - who touched the color - IP address.
        """

        ipWinner = args[0]
        self.winnersIPAddress.append(ipWinner)  # add this winner to winners list
        print("Color touched by ", ipWinner)
        self.ownNodeSpeaker(0, "")      # write on topic that another Kid wins


    def manageEndGameMsg(self, args):
        """
        Based on whether I'm the loser or not, kill the actual node and launch the next one.
        :param args: args[0]=loser's name (IP address).
        """

        ipLoser = args[0]
        self.launch.shutdown()
        threading.Thread(target=self.resetParameters, args=[ipLoser]).start()   # another thread will call resetParams
        # self.resetParameters(ipLoser)


    def ownNodeSpeaker(self, typeOfMess, color):
        # manage messages TO Kid/Witch nodes writing on its publisher

        # Witch:
        # 0 (a "color touched" socket is arrived)
        # 1 (the number of total players)
        # Kid:
        # 0 (the socket which contains the color arrived)
        # 1 (the "go!" socket arrived)

        typeOfMess = int(typeOfMess)

        if self.role:   # I am a Witch
            if typeOfMess == 0:
                self.nodeSpeakerPub.publish("0")                            # another robot touched the color
            elif typeOfMess == 1:
                self.nodeSpeakerPub.publish("1:" + str(len(self.ipList) - 1))    # number of Kids

        else:           # I am a Kid
            if typeOfMess == 0:
                self.nodeSpeakerPub.publish("0:" + color)   # color received


    def ownNodeListener(self, msg):
        # manage messages FROM Kid/Witch nodes reading from its subscriber

        # Witch
        # 0:colore (il colore che la mia Witch ha scelto, da inviare a tutti i RoleManager dei Kid)
        # 1 (ho ricevuto tutti i messaggi  necessari alla fine del gioco) senza info
        # Kid
        # 0 (colore trovato) senza info

        msg = msg.data

        if self.role:   # I am a Witch
            if msg[0] == "0":
                color = msg[2:]

                self.topicHandlers[0](color)    # call tellColorBySocket(color)

            elif msg[0] == "1":
                # game over: n-1 messages received

                self.topicHandlers[1]()      # tellEndGameBySocket()

        else:           # I am a Kid
            if msg[0] == "0":
                self.topicHandlers[0]()     # tellColorTouchedBySocket
                # self.sock.sendall("1:" + self.myIPAddress + "|")  # send a "color touched msg" to the Witch

    def prepareLaunchNode(self, iAmWitch):
        """
        Choose and configure the Witch/Kid node that will be launched.
        :param iAmWitch: True if I am the witch of the next game, False otherwise
        :return: the launcher object refers to the Witch/Kid node to launch
        """

        path = roslib.packages.get_pkg_dir("touch_the_color")

        if iAmWitch:
            path += "/src/witchLauncher.launch"
        else:
            path += "/src/kidLauncher.launch"

        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch = roslaunch.parent.ROSLaunchParent(uuid, [path])
        return launch

    def startNode(self):
        """
        Start the node referred to self.launch.
        ... it seams there's no way to control the launcher status
        """

        self.launch.start()
        rospy.loginfo("started")

    def config(self):
        """
        Configure topic handlers, prepare and start the launch node and call createAndStartConnection.
        """

        connected = False

        if self.myIPAddress == self.witchIPAddress:
            self.role = True
            self.topicHandlers = self.WITCH_COLOR_HANDLERS
        else:
            self.role = False
            self.topicHandlers = self.KID_COLOR_HANDLERS

        # based on its role, launch Witch or Kid node
        self.launch = self.prepareLaunchNode(self.role)
        self.startNode()

        time.sleep(2)

        if self.role:                   # if I am a Witch
            self.ownNodeSpeaker(1, "")  # send to my Witch the number of players

        startConnectionAttempt = time.time()
        while not connected and time.time()-startConnectionAttempt<self.CONNECTION_ATTEMPT_TIMEOUT:
            connected = self.createAndStartConnection()
        
        if not connected:
            self.launch.shutdown()
            exit(1)

    def resetParameters(self, witchIp):
        """
        Reset all parameters for the next game. At the end, call config.
        """

        self.stopThreads = True
        for t in self.myThread:
            if t is not None:
                t.join()

        self.host = None
        self.launch = None
        self.role = None
        self.socketKidToWitch = None
        self.noConnected = 0
        self.topicHandlers = []
        del self.myThread[:]
        del self.conn[:]
        del self.address[:]
        del self.winnersIPAddress[:]

        self.witchIPAddress = witchIp

        self.config()


if __name__ == "__main__":
    try:
        rm = RoleManager()
    except rospy.ROSInterruptException:
        print("ROSInterruptException")
    except KeyboardInterrupt:
        print("KeyboardInterrupt")
