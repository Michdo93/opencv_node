#!/usr/bin/env python
import os
import sys
import time
import rospy
import roslib
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# OpenCV
import cv2

VERBOSE = False


class CV2Imread(object):

    def __init__(self):
        """Configure subscriber."""
        # Create a subscriber with appropriate topic, custom message and name of
        # callback function.
        self.pub = rospy.Publisher('opencv/image/get', Image, queue_size = 10)

        self.br = CvBridge()
        self.image = None

        self.name = "opencv.jpeg"

        self.enable = False

        if self.enable:
            self.start()
        else:
            self.stop()

    def start(self):
        self.enable = True
        self.pub = rospy.Publisher('opencv/image/get', Image, queue_size = 10)

        img = cv2.imread(self.name)

        try:
            self.image = self.br.cv2_to_imgmsg(img, 'bgr8')
        except CvBridgeError as e:
            print(e)

    def stop(self):
        """Turn off subscriber."""
        self.enable = False
        self.pub.unregister()
        
if __name__ == '__main__':
    # Initialize the node and name it.
    node_name = "CV2Imread"
    rospy.init_node(node_name, anonymous=False)
    
    image = CV2Imread()
    
    # Go to the main loop
    try:
        image.start()
    # Stop with Ctrl + C
    except KeyboardInterrupt:
        image.stop()
        pass
    
    # Allow ROS to go to all callbacks.
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
