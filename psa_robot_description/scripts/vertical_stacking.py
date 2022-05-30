#! /usr/bin/python3

# rospy for the subscriber
import rospy

# OpenCV2 for saving an image
import cv2
import numpy as np
from matplotlib.pyplot import gray
from geometry_msgs.msg import Twist

# ROS Image message
from sensor_msgs.msg import Image

# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError


def verticalStacking(image, y_limits):

    sampled_image = image[y_limits[0]:y_limits[1], :]

    cv2.imshow('test', sampled_image)

    stack = np.sum(sampled_image, axis=0) / 255

    return stack

    
