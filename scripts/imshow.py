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


class CV2Imshow(object):

    def __init__(self):
        """Configure subscriber."""
        # Create a subscriber with appropriate topic, custom message and name of
        # callback function.
        self.sub = rospy.Subscriber('opencv/image/get', Image, self.callback, queue_size = 10)

        self.br = CvBridge()
        self.image = None
        self.data = None

        self.enable = False

        if self.enable:
            self.start()
        else:
            self.stop()

    def start(self):
        self.enable = True
        self.sub = rospy.Subscriber('opencv/image/get', Image, self.callback, queue_size = 10)

    def stop(self):
        """Turn off subscriber."""
        self.enable = False
        self.sub.unregister()

    def callback(self, data):
        """Handle subscriber data."""
        # Simply print out values in our custom message.
        self.data = data

        try:
            self.image = self.br.imgmsg_to_cv2(self.data, 'bgr8')
        except CvBridgeError as e:
            print(e)
        
        cv2.imshow('cv_img', self.image)
        cv2.waitKey(25)
        
if __name__ == '__main__':
    # Initialize the node and name it.
    node_name = "CV2Imshow"
    rospy.init_node(node_name, anonymous=False)
    
    image = CV2Imshow()
    
    # Go to the main loop
    try:
        image.start()
        # Wait for messages on topic, go to callback function when new messages arrive.
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
    # Stop with Ctrl + C
    except KeyboardInterrupt:
        image.stop()

        nodes = os.popen("rosnode list").readlines()
        for i in range(len(nodes)):
            nodes[i] = nodes[i].replace("\n","")

        for node in nodes:
            os.system("rosnode kill " + node_name)

        
        print("Node stopped")
    cv2.destroyAllWindows()
