#!/usr/bin/env python

import rospy
import math
import time
import message_filters
import cv2
import image_geometry
import numpy as np
import imutils


from util.Vector2D import Vector2D
from cv_bridge import CvBridge, CvBridgeError
from math import sqrt, pi, cos, sin
from random import random
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud, Image, CameraInfo
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class Kid:
    
    # distance constants in meters
    RELIABLE_DISTANCE = 5
    OBSTACLE_DISTANCE = 1
    SAFETY_DISTANCE = 0.5
    COLOR_TOUCHED_DISTANCE = 0.55
    MIN_ATTRACTION_DISTANCE = 4.5
    MIN_ATTRACTION_FORCE = 0.7

    # time and rate constants in seconds
    MAX_TIME_ELAPSED = 10000#TODO 5
    POSE_UPDATE_RATE = 0.25 
    RANDOM_FIELD_RATE = 10
    LOOK_UPDATE_RATE = 0.10

    # angular constants 
    DEG_TO_RAD = pi/180.0
    RAG_TO_DEG = 180.0/pi
    ANGULAR_THRESHOLD = 3.0 * DEG_TO_RAD
    NO_ROTATION_MATRIX = [[1.0,0.0],[0.0,1.0]]

    # attraction constants
    MAX_ATTRACTION_DISTANCE = 4.5
    MIN_ATTRACTION_FORCE = 0.7
    ATTRACTION_SLOPE = (1-MIN_ATTRACTION_FORCE)/(SAFETY_DISTANCE-MAX_ATTRACTION_DISTANCE)

    def __init__(self):
        rospy.init_node("Kid", anonymous=True)

        # attributes for obstacle avoidance
        self.distance = [10.0 for i in range(0,16)]
        self.obstacles = []

        # attributes for pose management and POI robot-centric localization
        self.axisRotationMatrix = self.NO_ROTATION_MATRIX
        self.startOrientation = 0.0
        self.translationVector = Vector2D()
        self.startPoint = Vector2D()
        self.teta = 0.0
        self.rotationMatrix = self.NO_ROTATION_MATRIX
        self.POI = Vector2D()

        # releasers
        self.targetFound = False
        self.avoidReleaser = False
        self.canRead = True
        self.POIFound = False

        # temporal releasers
        self.lastPOIFound = -self.MAX_TIME_ELAPSED
        self.lastPoseUpdateTime = -self.POSE_UPDATE_RATE
        self.lastLookUpdateTime = -self.LOOK_UPDATE_RATE
        self.lastRandomField = -self.RANDOM_FIELD_RATE

        self.inGame = False
        self.colorToTouch = ""
        self.colorTouched = False

        # ROS library needed for image conversion from ROS to OpenCV
        self.bridge = CvBridge()
        # Needed to interpretate images geometrically with camera parameters
        self.camera_model = image_geometry.PinholeCameraModel()

        # publishers and subscribers
        self.velPub = rospy.Publisher("/RosAria/cmd_vel", Twist, queue_size=1)
        self.sonarSub = rospy.Subscriber("/RosAria/sonar", PointCloud, self.readSonar)
        self.poseSub = rospy.Subscriber("/RosAria/pose", Odometry, self.updatePose)
        # TODO gestire queste bestiole qui
        self.RMSpeakerPub = rospy.Publisher("node_to_rolemanager", String, queue_size=10)
        self.RMListenerSub = rospy.Subscriber("rolemanager_to_node", String, self.ownRoleManagerListener)
        # Subscriber for computer vision module
        self.imageSub = message_filters.Subscriber("/camera/rgb/image_rect_color", Image)
        self.depthSub = message_filters.Subscriber("/camera/depth_registered/image", Image)
        self.cameraInfoSub = message_filters.Subscriber("/camera/depth_registered/camera_info", CameraInfo)
        # Topic time synchronizer
        ats = message_filters.ApproximateTimeSynchronizer([self.imageSub,self.depthSub,self.cameraInfoSub], queue_size = 10, slop = 0.1)
        ats.registerCallback(self.look)

        # Color target
        self.colorlower = None
        self.colorupper = None

    def ownRoleManagerListener(self, msg):
        # manage messages FROM RoleManager reading from its subscriber

        # 0:color (color to touch)
        # 1 (go!)

        msg = msg.data

        if msg[0] == "0":
            self.colorToTouch = msg[2:]

            #TODO remove
            # self.ownRoleManagerSpeaker(0)
            # time.sleep(2)

            self.colorlower, self.colorupper = Colors.getColor(self.colorToTouch)
            self.inGame = True


    def ownRoleManagerSpeaker(self, typeOfMess):
        # manage messages TO RoleManager writing on its publisher

        # 0 (color touched)

        if typeOfMess == 0:
            self.RMSpeakerPub.publish("0")      # color touched

    def start(self):
        rate_start = rospy.Rate(3.3)

        # wait until the game starts
        while not self.inGame and not rospy.is_shutdown():
            time.sleep(0.5)

        while self.inGame and not rospy.is_shutdown():
            self.canRead = False
            fieldVector = self.wander()
            if self.POIFound or time.time()-self.lastPOIFound<self.MAX_TIME_ELAPSED:
                fieldVector = self.moveToColor(fieldVector)
            if self.avoidReleaser:
                fieldVector = self.avoid(fieldVector)
                self.avoidReleaser = False
            self.canRead = True
            self.move(fieldVector)
            rate_start.sleep()

        self.move(Vector2D())

        if self.colorTouched:
            print("TROVATO!")
            self.ownRoleManagerSpeaker(0)

    def look(self, RGBimage, depthImage, cameraInfo):
        currentTime = time.time()
        if not self.inGame or currentTime-self.lastLookUpdateTime<self.LOOK_UPDATE_RATE:
            return
        else:
            self.lastLookUpdateTime = currentTime

        centroid = self.readImageRGB(RGBimage)
        if centroid is not None:
            ray = self.extractCameraInfo(cameraInfo, centroid)
            distance = self.extractDepth(depthImage, centroid)

            if distance is not None and not np.isnan(distance):
                candidatePOI = Vector2D(ray[0]*distance, ray[2]*distance)

                if not self.POIFound and (currentTime - self.lastPOIFound) >= self.MAX_TIME_ELAPSED:
                    self.POI = candidatePOI
                    self.POIFound = True
                else:
                    currentPOI = self.POI + self.translationVector
                    print("POI CORRENTE: ", currentPOI.getIntensity())
                    print("POI CANDIDATO: ", candidatePOI.getIntensity())

                    if candidatePOI.getIntensity() < currentPOI.getIntensity():
                        self.POI = candidatePOI
                        self.POIFound = True

                # self.POI.x = ray[0]*distance
                # self.POI.y = ray[2]*distance
                # self.POIFound = True
                #print("DISTANZA DA POI: ", distance)
                if distance <= self.COLOR_TOUCHED_DISTANCE:
                    self.inGame = False
                    self.colorTouched = True
            else:
                self.POIFound = False
        else:
            self.POIFound = False

        if self.POIFound:
            self.lastPOIFound = currentTime

    def moveToColor(self, force):
        #B
        # translate and rotate POI
        currentPOI = self.POI+self.translationVector
        currentPOI = currentPOI.multiplyMatrix(self.rotationMatrix)

        # calculate attraction vector
        intensity = self.attract(currentPOI.getIntensity())
        vector = Vector2D(currentPOI.getVersorX()*intensity, currentPOI.getVersorY()*intensity)
        
        return vector

    def wander(self): #TODO params
        # return Vector2D()
        #B
        currentTime = time.time()
        if currentTime-self.lastRandomField<self.RANDOM_FIELD_RATE:
            return Vector2D(0.0,1.0)
        else:
            intensity = 0.5 + random()*0.5
            angle = random()*pi
            if random()<0.5:
                angle = -angle
            self.lastRandomField = currentTime
            return Vector2D(cos(angle)*intensity, sin(angle)*intensity)

    def avoid(self, force):
        #B
        for p in self.obstacles:
            intensity = self.repel(p.getIntensity())
            vector = Vector2D(p.getVersorX()*intensity, p.getVersorY()*intensity)
            force -= vector
        return force

    def move(self, vector):
        #MS
        degree = math.atan2(vector.x, vector.y)
        angularVelocity = degree/2.0
        
        msg = Twist()
        
        linearVelocity = vector.y/4.0
        if linearVelocity<=0.0001 : #TODO abs(linearVelocity to admit backwards movement
            linearVelocity = 0.0

        if abs(degree)<=self.ANGULAR_THRESHOLD or pi+self.ANGULAR_THRESHOLD<=abs(degree)<=pi-self.ANGULAR_THRESHOLD:
            angularVelocity = 0.0

        msg.linear.x = linearVelocity
        msg.angular.z = angularVelocity
        
        self.velPub.publish(msg)

        #time.sleep(0.3)

#    def integrate(self): #TODO params
        #B
        #TODO manage objects position after movement

    def feelForce(self, points):
        #PS
        del self.obstacles[:]
        i = 0
        for p in points:
            if i>7:
                continue
            distance = sqrt((p.x)**2 + (p.y)**2)
            if distance<=self.RELIABLE_DISTANCE:
                if distance<=self.OBSTACLE_DISTANCE:
                    self.avoidReleaser = True
                    self.obstacles.append(Vector2D(p.y,p.x))
            i = i+1

    def readSonar(self, msg):
        #PS
        if self.canRead:
            points = msg.points
            self.feelForce(points)

    def updatePose(self, msg):
        #PS
        currentTime = time.time()
        if currentTime-self.lastPoseUpdateTime<self.POSE_UPDATE_RATE:
            return
        else:
            self.lastPoseUpdateTime = currentTime

        pose = msg.pose.pose
        msgQuaternion = pose.orientation
        msgPoint = pose.position
        point = Vector2D(msgPoint.y, msgPoint.x)
        orientation = euler_from_quaternion([msgQuaternion.x, msgQuaternion.y, msgQuaternion.z, msgQuaternion.w])[2]

        if self.POIFound:
            self.startPoint = point
            self.startOrientation = orientation
            self.axisRotationMatrix = [[cos(-orientation), -sin(-orientation)], [sin(-orientation), cos(-orientation)]]
            self.startPoint = point.multiplyMatrix(self.axisRotationMatrix)
            self.translationVector = Vector2D()
            self.rotationMatrix = self.NO_ROTATION_MATRIX
            self.teta = 0
            # self.lastPOIFound = currentTime
        elif currentTime - self.lastPOIFound < self.MAX_TIME_ELAPSED:
            self.teta = orientation - self.startOrientation
            self.rotationMatrix = [[cos(self.teta), -sin(self.teta)], [sin(self.teta), cos(self.teta)]]
            self.translationVector = self.startPoint - point.multiplyMatrix(self.axisRotationMatrix)

#    def listen(self, msg):
        #PS
        #TODO listen for messages

#    def determineEndGame(self): #TODO params
        #B
        #TODO determine end game behaviour

#   def communicateColorTouched(self): #TODO params
        #MS
        #TODO communicate that the color has been touched

    # repulsion function for potential fields
    def repel(self, d):
        factor = 14.0
        base = 2.0
        return  (base**(-factor * (d-self.SAFETY_DISTANCE)))

    # attraction function for potential fields
    def attract(self, d):
        return self.ATTRACTION_SLOPE*d+(1.0-self.ATTRACTION_SLOPE*self.SAFETY_DISTANCE)

    # ray extraction from image
    def extractCameraInfo(self, cameraInfo, point):
        self.camera_model.fromCameraInfo(cameraInfo)
        return self.camera_model.projectPixelTo3dRay(point)

    # depth extraction from depth image
    def extractDepth(self, image, coordinates):
        try:
            cvImage = self.bridge.imgmsg_to_cv2(image, "32FC1")
        except CvBridgeError as e:
            print(e)
            return None
        
        # Flip image
        cvImage = cv2.flip(cvImage,1)
        return cvImage[coordinates[1]][coordinates[0]]

    # centroid extraction from RGB image
    def readImageRGB(self, image):
        try:
            cvImage = self.bridge.imgmsg_to_cv2(image,"bgr8")
        except CvBridgeError as e:
            print(e)
            return None
        # Flip image
        cvImage = cv2.flip(cvImage,1)
        # Applying Gaussian blur to remove noise
        cvImage = cv2.GaussianBlur(cvImage,(11,11),0)

        # RGB to HSV conversion
        hsvImage = cv2.cvtColor(cvImage,cv2.COLOR_BGR2HSV)

        # Blob extraction
        mask = cv2.inRange(hsvImage, self.colorlower, self.colorupper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        contours = cv2.findContours(mask.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        contours = imutils.grab_contours(contours)
        
        # Widest blob extraction
        if len(contours) > 0:
            widestBlob = max(contours, key=cv2.contourArea)
            # Centroid extraction using image moments
            M = cv2.moments(widestBlob)
            centroid = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        else:
            centroid = None
        return centroid

    

if __name__ == "__main__":
    try:
        robot = Kid()

        robot.start()
    except rospy.ROSInterruptException:
        print("ROSInterruptException")
    except KeyboardInterrupt:
        print("KeyboardInterrupt")
