#!/usr/bin/env python
import rospy
import message_filters
import cv2
import numpy as np
import image_geometry
import imutils
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

CONST = 20

class Kinect:
    def __init__(self):
        rospy.init_node('Kinect', anonymous=True)

        # Subscribers
        self.imageSub = message_filters.Subscriber("/camera/rgb/image_rect_color", Image)
        self.depthSub = message_filters.Subscriber("/camera/depth_registered/image", Image)
        self.cameraInfoSub = message_filters.Subscriber("/camera/depth_registered/camera_info", CameraInfo)
        # Topic time synchronizer
        ats = message_filters.ApproximateTimeSynchronizer([self.imageSub,self.depthSub,self.cameraInfoSub], queue_size = 10, slop = 0.1)
        ats.registerCallback(self.imageCapture)

        # ROS library needed for image conversion from ROS to OpenCV
        self.bridge = CvBridge()
        # Needed to interpretate images geometrically with camera parameters
        self.camera_model = image_geometry.PinholeCameraModel()
        # Color init
        lightblue = np.uint8([[[204,204,0]]])
        hsv_lightblue = cv2.cvtColor(lightblue, cv2.COLOR_BGR2HSV)
        self.colorlower = np.array([hsv_lightblue[0][0][0]-CONST, 50, 50])
        self.colorupper = np.array([hsv_lightblue[0][0][0]+CONST, 255, 255])

    def imageCapture(self, RGBimage, depthImage, cameraInfo):
        # Main callback
        centroid = self.readImageRGB(RGBimage)
        if centroid is not None:
            ray = self.extractCameraInfo(cameraInfo, centroid)
            distance = self.extractDepth(depthImage, centroid)
            if not np.isnan(distance):
                print(ray[0]*distance, ray[2]*distance) # RETURN Vector2D

    def setColor(self, color):
        self.colorlower, self.colorupper = color #TODO: controllo su parametro

    def extractCameraInfo(self, cameraInfo, point):
        self.camera_model.fromCameraInfo(cameraInfo)
        return self.camera_model.projectPixelTo3dRay(point)
        
    def extractDepth(self, image, coordinates):
        try:
            cvImage = self.bridge.imgmsg_to_cv2(image, "32FC1")
        except CvBridgeError as e:
            print(e)
        
        # Flip image
        cvImage = cv2.flip(cvImage,1)
        return cvImage[coordinates[1]][coordinates[0]]

    def readImageRGB(self, image):
        try:
            cvImage = self.bridge.imgmsg_to_cv2(image,"bgr8")
        except CvBridgeError as e:
            print(e)
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

if __name__ == '__main__':
    ic = Kinect()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")