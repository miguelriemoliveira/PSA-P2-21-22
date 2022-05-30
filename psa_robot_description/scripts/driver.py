#! /usr/bin/python3

# rospy for the subscriber
from copy import deepcopy

import rospy

# OpenCV2 for saving an image
import cv2
from geometry_msgs.msg import Twist

# ROS Image message
from sensor_msgs.msg import Image

# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError

# Global variables
# Instantiate CvBridge
bridge = CvBridge()
publisher = None


def imageCallback(msg):
    print("Received an image!")

    try:
        # Convert your ROS Image message to OpenCV2
        image_rgb = bridge.imgmsg_to_cv2(msg, "bgr8")
    except:
        print('Could not convert image')
        return

    # Preprocessing
    image_gui = deepcopy(image_rgb)
    image_gray = cv2.cvtColor(image_rgb, cv2.COLOR_BGR2GRAY)  # convert to grayscale
    _, image_thresh = cv2.threshold(image_gray, 127, 255, cv2.THRESH_BINARY)  # thresholding

    # Save your OpenCV2 image as a jpeg

    # How to process the image to define the best angle and speed?
    reference_y = 190
    height, width = image_thresh.shape
    middle_x = int(width / 2)

    # search from middle to right
    right_xs = []
    for x in range(middle_x, width):
        if image_thresh[reference_y, x] == 255:
            right_xs.append(x)

    # search from middle to left
    left_xs = []
    for x in range(middle_x-1, -1, -1):
        if image_thresh[reference_y, x] == 255:
            left_xs.append(x)

    # Drawing
    for x in right_xs:
        cv2.line(image_gui, (x, reference_y), (x, reference_y), (0, 0, 255), 4)

    for x in left_xs:
        cv2.line(image_gui, (x, reference_y), (x, reference_y), (255, 0, 0), 4)

    # make a driving decision
    angle = 0
    speed = 0

    # cv2.imshow('image_rgb', image_rgb)
    # cv2.imshow('image_gray', image_gray)
    # cv2.imshow('image_thresh', image_thresh)
    cv2.imshow('image_gui', image_gui)
    cv2.waitKey(20)

    # build a twist msg to publish
    twist = Twist()
    twist.linear.x = speed
    twist.angular.z = angle
    global publisher
    publisher.publish(twist)


def main():
    rospy.init_node('driver')
    # Define your image topic
    image_topic = "/front_camera/rgb/image_raw"
    # Set up your subscriber and define its callback
    rospy.Subscriber(image_topic, Image, imageCallback)
    global publisher
    publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    # Spin until ctrl + c
    rospy.spin()


if __name__ == '__main__':
    main()
