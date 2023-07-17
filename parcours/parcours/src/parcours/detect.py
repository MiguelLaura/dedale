#!/usr/bin/env python3

######################################################
# BUT :                                              #
#   détecter les couleurs rouge et bleue des flèches #
#   indique où tourner en fonction de la couleur     #
#   il doit le faire quand on est suffisament près   #
#   distance subjective                              #
# Utilise la caméra                                  #
# Correspond au TP05                                 #
######################################################


import cv2
from cv_bridge import CvBridge, CvBridgeError
#import numpy as np
import rospy
from datetime import datetime

# Type of input messages
from sensor_msgs.msg import Image


class DetectNode:
    def __init__(self):
        # Creates a node called and registers it to the ROS master
        rospy.init_node('detect')

        # CvBridge is used to convert ROS messages to matrices manipulable by OpenCV
        self.bridge = CvBridge()

        # Initialize the node parameters
        # / 2, * 2.5 pour respecter les conventions de cv2
        self.interval_lower_blue = (190 / 2, 70 * 2.5, 40 * 2.5)
        self.interval_upper_blue = (200 / 2, 110 * 2.5, 70 * 2.5)

        self.interval_lower_red = (5 / 2, 40 * 2.5, 40 * 2.5)
        self.interval_upper_red = (25 / 2, 75 * 2.5, 75 * 2.5)

        # Publisher to the output topics.
        self.pub = rospy.Publisher('~output', Image, queue_size=10)

        # Subscriber to the input topic. self.callback is called when a message is received
        self.subscriber = rospy.Subscriber('/camera/image_raw', Image, self.callback)


    def callback(self, msg):
        '''
        Function called when an image is received.
        msg: Image message received
        img_bgr: Width*Height*3 Numpy matrix storing the image
        '''
        # Convert ROS Image -> OpenCV
        try:
            img_bgr = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logwarn(e)
            return

        # Easier to detect colors in HSV
        img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)


        def draw_contours_and_centre(interval_lower, interval_upper):
            mask = cv2.inRange(img_hsv, interval_lower, interval_upper)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

            # Draw contour in green
            cv2.drawContours(img_bgr, contours, -1, (0,255,0), 3)
            
            # Find the largest area contour
            if contours != []:
                max_area = -1
                for i in range(len(contours)):
                    area = cv2.contourArea(contours[i])
                    if area > max_area:
                        max_area = area
            
                return max_area
        
        max_area_blue = draw_contours_and_centre(self.interval_lower_blue, self.interval_upper_blue)
        max_area_red = draw_contours_and_centre(self.interval_lower_red, self.interval_upper_red)
        

        # Object close enough
        if max_area_blue and max_area_blue > 450000:
            print(datetime.now().strftime("%H:%M:%S"), "TOURNEZ À DROITE")
        if max_area_red and max_area_red > 600000:
            print(datetime.now().strftime("%H:%M:%S"), "TOURNEZ À GAUCHE")
    
        # Convert OpenCV -> ROS Image and publish
        try:
            self.pub.publish(self.bridge.cv2_to_imgmsg(img_bgr, "bgr8"))
        except CvBridgeError as e:
            rospy.logwarn(e)


if __name__ == '__main__':
    # Start the node and wait until it receives a message or stopped by Ctrl+C
    node = DetectNode()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
