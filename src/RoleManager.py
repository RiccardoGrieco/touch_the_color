#!/usr/bin/env python


import rospy
import socket
import threading
import roslaunch
import roslib
import select
import time
from std_msgs.msg import String


class RoleManager:

    PORT = 65535
    RECEIVE_SOCKET_TIMEOUT = 3
    CONNECTION_ATTEMPT_TIMEOUT = 5

    def __init__(self):
        """
        The rosparam ipList MUST be setted as list of strings ("['127.0.0.1', '127.0.1.1']".
        In ipList there's also my ip address.
        """

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

        # handlers to methods that manages sockets sends
        self.WITCH_COLOR_HANDLERS = [self.tellColorBySocket, self.tellEndGameBySocket]
        self.KID_COLOR_HANDLERS = [self.tellColorTouchedBySocket]

        # handlers to methods that manages the received socket messages
        self.handlers = [self.manageColorMsg,
                         self.manageColorTouchedMsg,
                         self.manageEndGameMsg]

        print("len ipList = " + str(len(self.ipList)))

        self.config()

        rospy.spin()        # in loop until is killed

    def tellColorBySocket(self, color):
        """
        Only Witch call this method.
        Send to Kids' RM the chosen color.
        :param color: the chosen color
        """
        # TODO
        print("in tellColorBySocket")
        msg = "0:" + color + "|"
        noPartecipants = len(self.ipList)-1
        while self.noConnected < noPartecipants:
            time.sleep(0.1)
            # print("noConnected: " + str(self.noConnected))

        for c in self.conn:
            c.send(msg)
        print("esco da tellColorBySocket")

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

        self.manageEndGameMsg([nextWitchIP[0]])

    def tellColorTouchedBySocket(self):
        """
        Only Kid call this method.
        Send to Witch's RM that the color has been touched.
        """
        msg = "1:" + self.myIPAddress + "|"

        self.socketKidToWitch.sendall(msg)

    def createAndStartConnection(self):
        """
        Create and start socket connection.
        :param witchIPAddress: ip address of robot that will be the witch in the next game
        :return False if connection doesn't succeed, True otherwise.
        """

        self.host = self.witchIPAddress
        self.stopThreads = False

        """
        if self.sock is not None:
            self.sock.close()
        """

        if self.myIPAddress == self.witchIPAddress:

            # self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) I don't know what it do
            if not self.listenSockSetted:
                self.listenSock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.listenSock.bind((self.host, self.port))
                self.listenSockSetted = True

            self.listenSock.listen(len(self.ipList) - 1)
            for i in range(1, len(self.ipList)):
                conn, address = self.listenSock.accept()
                conn.setblocking(0)
                self.conn.append(conn)
                self.address.append(address)
                thread = threading.Thread(target=self.manageSocket, args=[conn])
                thread = threading.Thread(target=self.manageConnectionWithKid, args=(conn, address))
                self.myThread.append(thread)
                thread.start()
                self.noConnected = self.noConnected+1
                # TODO togliere
                print("Connected to client")
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
            # thread = threading.Thread(target=self.manageConnectionWithWitch, args=[self.witchIPAddress])
            self.myThread.append(socketThread)
            socketThread.start()

            # TODO togliere
            print("Connected to server")

        return True

    def manageSocket(self, threadSocket):
        """
        Manage socket messages.
        :param socket: the socket to listen.
        """

        size = 1024
        while not self.stopThreads:
            try:
                ready = select.select([threadSocket], [], [], self.RECEIVE_SOCKET_TIMEOUT)

                if ready[0]:
                    data = threadSocket.recv(size)[:-1]

                    if data:
                        # TODO togliere
                        print("MESSAGGIO SOCKET: " + str(data))

                        for msg in data.split("|"):
                            params = msg.split(":")                    # param[0]=type of msg; param[1]=msg
                            self.handlers[int(params[0])](params[1:])   # call the method that msg refers to
                    else:
                        print('Server/Client disconnected.')
                        break
            except Exception as e:
                print(str(e))
                break
        threadSocket.shutdown(socket.SHUT_RDWR)
        threadSocket.close()


    def manageConnectionWithWitch(self, witchIPAddress):
        """
        Only Kids call this method.

        :param witchIPAddress:
        :return: False is something in the connection go wrong.
        """

        size = 1024

        while not self.stopThreads:
            try:
                ready = select.select([self.sock], [], [], self.RECEIVE_SOCKET_TIMEOUT)

                if ready[0]:
                    data = self.sock.recv(size)[:-1]

                    if data:
                        # TODO togliere
                        print("MESSAGGIO SOCKET DA WITCH: " + str(data))

                        for msg in data.split("|"):
                            params = msg.split(":")                    # param[0]=type of msg; param[1]=msg
                            self.handlers[int(params[0])](params[1:])   # call the method that msg refers to
                    else:
                        print('Server disconnected')
                        break
            except Exception as e:
                print(str(e))
                break
        self.sock.shutdown(socket.SHUT_RDWR)
        self.sock.close()

    def manageConnectionWithKid(self, conn, address):
        """
        Only Witches call this method.

        :param conn: new socket object that could be used to send/receive messages on the connection
        :param address: the address bound to the socket
        :return: False is something in the connection go wrong.
        """

        size = 1024

        while not self.stopThreads:
            try:
                ready = select.select([conn], [], [], self.RECEIVE_SOCKET_TIMEOUT)

                if ready[0]:
                    data = conn.recv(size)[:-1]

                    if data:
                        # TODO togliere
                        print("MESSAGGIO SOCKET DA KID: " + str(data))

                        for msg in data.split("|"):
                            params = msg.split(":")                    # param[0]=type of msg; param[1]=msg

                            self.handlers[int(params[0])](params[1:])   # call the method that msg refers to
                    else:
                        print('Client disconnected')
                        break
            except Exception as e:
                print(str(e))
                break

        conn.shutdown(socket.SHUT_RDWR)
        conn.close()


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
        TODO: decidere se gestire il messaggio direttamente qui o riferire tutto al nodo Witch e poi vedere che fare
        :param args: args[0]=Kid's - who touched the color - IP address.
        """

        ipWinner = args[0]
        self.winnersIPAddress.append(ipWinner)  # add this winner to winners list
        print("gestisco messaggio di colore toccato da " + str(ipWinner))
        self.ownNodeSpeaker(0, "")      # write on topic that another Kid wins


    def manageEndGameMsg(self, args):
        """
        Based on whether I'm the loser or not, kill the actual node and launch the next one.
        :param args: args[0]=loser's name (IP address).
        """

        ipLoser = args[0]
        self.launch.shutdown()  # TODO posso verificare che termina?
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

        print("entro in ownNodeListener")

        msg = msg.data

        if self.role:   # I am a Witch
            if msg[0] == "0":
                color = msg[2:]
                print("leggo topic: " + msg)
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

        # TODO togliere
        print("IP LIST: ", self.ipList)
        print("MY IP:", self.myIPAddress)

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
            print("ESCO")
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

        """
        if self.sock is not None:
            try:
                self.sock.shutdown(socket.SHUT_RDWR)
                self.sock.close()
            except Exception:
                self.sock = None
        """

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

    def endGame(self, msg):
        # ?
        i = 0


if __name__ == "__main__":
    try:
        rm = RoleManager()
    except rospy.ROSInterruptException:
        print("ROSInterruptException")
    except KeyboardInterrupt:
        print("KeyboardInterrupt")
