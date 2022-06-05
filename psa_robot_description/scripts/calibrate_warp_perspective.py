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
import numpy as np

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
    
    if event == cv2.EVENT_RBUTTONDOWN:
        if len(clicked_xs)==4:
            print('right click')
            p1 = [clicked_xs[0], clicked_ys[0]]
            p2 = [clicked_xs[1], clicked_ys[1]]
            p3 = [clicked_xs[2], clicked_ys[2]]
            p4 = [clicked_xs[3], clicked_ys[3]]
            
            d12 = np.linalg.norm([p1[0]-p2[0], p1[1]-p2[1]])
            d43 = np.linalg.norm([p4[0]-p3[0], p4[1]-p3[1]])
            
            coeff = 0.5
            
            p2c = [p1[0], p1[1]-coeff*d12]
            p3c = [p4[0], p4[1]-coeff*d43]
            
            input_points = np.float32([p1,p2,p3,p4])
            output_points = np.float32([p1,p2c,p3c,p4])
            
            M = cv2.getPerspectiveTransform(input_points, output_points)
            
            finalimage = cv2.warpPerspective(image_rgb, M, [800, 320])
            cv2.imshow('ipm', finalimage)
            cv2.waitKey(0)
            print('save file to disk')
            np.save('ipm_file.npy', M)


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
