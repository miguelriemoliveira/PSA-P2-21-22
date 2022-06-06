#! /usr/bin/python3

# rospy for the subscriber
import numpy as np
import rospy

# OpenCV2 for saving an image
import cv2
from geometry_msgs.msg import Twist

# ROS Image message
from sensor_msgs.msg import Image

# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError

# from psa_robot_description.scripts.vertical_stacking import verticalStacking
from vertical_stacking import verticalStacking

# Global variables
# Instantiate CvBridge
bridge = CvBridge()
publisher = None

M = None


def imageCallback(msg):
    print("Received an image!")

    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except:
        print('Could not convert image')
        return

    # Save your OpenCV2 image as a jpeg
    cv2.imshow('front_camera', cv2_img)

    ipm_image = cv2.warpPerspective(cv2_img, M, [900, 320])

    cv2.imshow('ipm', ipm_image)
    cv2.waitKey(1)


def main():
    global M
    rospy.init_node('driver')
    # Define your image topic
    image_topic = "/front_camera/rgb/image_raw"
    # Set up your subscriber and define its callback
    rospy.Subscriber(image_topic, Image, imageCallback)
    global publisher

    M = np.load('/home/mike/catkin_ws/src/PSA-P2-21-22/psa_robot_description/scripts/ipm_file.npy')

    # Spin until ctrl + c
    rospy.spin()


if __name__ == '__main__':
    main()
