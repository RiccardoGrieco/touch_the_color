#!/usr/bin/env python

import rospy
import message_filters
import cv2
import numpy as np
import image_geometry
import imutils
import time
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

CONST = 20

class Kinect_video:
    def __init__(self):
        rospy.init_node('Kinect', anonymous=True)

        # Subscribers
        self.imageSub = rospy.Subscriber("/camera/rgb/image_rect_color", Image, self.readImageRGB)
        self.depthSub = rospy.Subscriber("/camera/depth_registered/image", Image, self.extractDepth)
        self.cameraInfoSub = rospy.Subscriber("/camera/depth_registered/camera_info", CameraInfo, self.extractCameraInfo)

        # ROS library needed for image conversion from ROS to OpenCV
        self.bridge = CvBridge()
        # Needed to interpretate images geometrically with camera parameters
        self.camera_model = image_geometry.PinholeCameraModel()
        # Color init
        lightblue = np.uint8([[[204, 204, 0]]])
        hsv_lightblue = cv2.cvtColor(lightblue, cv2.COLOR_BGR2HSV)
        self.colorlower = np.array([hsv_lightblue[0][0][0]-CONST, 50, 50])
        self.colorupper = np.array([hsv_lightblue[0][0][0]+CONST, 255, 255])
        print(hsv_lightblue[0][0][0])
        # Centroid init
        self.centroid = None
        self.radius = None

    def setColor(self, color):
        self.colorlower, self.colorupper = color #TODO: controllo su parametro

    def extractCameraInfo(self, cameraInfo):
        self.camera_model.fromCameraInfo(cameraInfo)
        #if self.centroid is not None:
        #    print(self.camera_model.projectPixelTo3dRay(self.centroid))
        
    def extractDepth(self, image):
        try:
            cvImage = self.bridge.imgmsg_to_cv2(image, "32FC1")
        except CvBridgeError as e:
            print(e)
        cvImage = cv2.resize(cvImage,(640,480))
        # Flip image
        cvImage = cv2.flip(cvImage,1)
        if self.centroid is not None:
            #print(cvImage[self.centroid[1]][self.centroid[0]],"(",self.centroid[1],",",self.centroid[0],")")
            cv2.circle(cvImage, self.centroid, int(self.radius), (0, 255, 255), 2)
            cv2.circle(cvImage, self.centroid, 5, (0, 0, 255), -1)

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
        # Applying mask on original image
        #cvImage = cv2.bitwise_and(cvImage, cvImage, mask = mask)
        # Widest blob extraction
        if len(contours) > 0:
            widestBlob = max(contours, key=cv2.contourArea)
            ((_, _), self.radius) = cv2.minEnclosingCircle(widestBlob)
            # print("RADIUS: ", self.radius)
            # Centroid extraction using image moments
            M = cv2.moments(widestBlob)
            if self.radius >= 20:
                self.centroid = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                print(hsvImage[self.centroid[1],self.centroid[0]])
                cv2.circle(cvImage, self.centroid, int(self.radius), (0, 255, 255), 2)
                cv2.circle(cvImage, self.centroid, 5, (0, 0, 255), -1)
        else:
            self.centroid = None
        # Show output
        cv2.imshow('RGB', cvImage)
        cv2.waitKey(3)

if __name__ == '__main__':
    ic = Kinect_video()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")