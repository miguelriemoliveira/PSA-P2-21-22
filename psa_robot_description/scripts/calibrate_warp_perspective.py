#! /usr/bin/python3

# rospy for the subscriber
from copy import deepcopy

import rospy

# OpenCV2 for saving an image
import cv2
import matplotlib.pyplot as plt
from vertical_stacking import verticalStacking
from geometry_msgs.msg import Twist

# ROS Image message
from sensor_msgs.msg import Image

# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError

# Global variables
# Instantiate CvBridge
bridge = CvBridge()
publisher = None

global image_rgb

clicked_xs = []
clicked_ys = []


def imageCallback(msg):
    global image_rgb
    print("Received an image!")

    try:
        # Convert your ROS Image message to OpenCV2
        image_rgb = bridge.imgmsg_to_cv2(msg, "bgr8")
    except:
        print('Could not convert image')
        return


def clickEvent(event, x, y, flags, params):

    # checking for left mouse clicks
    if event == cv2.EVENT_LBUTTONDOWN:

        global clicked_xs
        global clicked_ys
        clicked_xs.append(x)
        clicked_ys.append(y)


def main():
    global image_rgb
    image_rgb = None
    rospy.init_node('calibrate_warp_perspective')
    # Define your image topic
    image_topic = "/front_camera/rgb/image_raw"
    # Set up your subscriber and define its callback
    rospy.Subscriber(image_topic, Image, imageCallback)

    cv2.namedWindow('image_gui', cv2.WINDOW_GUI_NORMAL)
    cv2.setMouseCallback('image_gui', clickEvent)

    # Spin until ctrl + c

    rate = rospy.Rate(10)  # 10hz
    while True:

        image_gui = deepcopy(image_rgb)

        font = cv2.FONT_HERSHEY_SIMPLEX
        point_idx = 0
        for x, y in zip(clicked_xs, clicked_ys):

            point = (x, y)
            color = (0, 255, 255)
            cv2.line(image_gui, point, point, color, 4)
            cv2.putText(image_gui, str(point_idx), point, cv2.FONT_HERSHEY_SIMPLEX, 1, color, 1, cv2.LINE_AA)
            point_idx += 1

        cv2.imshow('image_gui', image_gui)
        cv2.waitKey(100)

        rate.sleep()
        cv2.waitKey(50)


if __name__ == '__main__':
    main()
