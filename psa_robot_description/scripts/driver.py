#! /usr/bin/python3

# rospy for the subscriber
from copy import deepcopy

import rospy

# OpenCV2 for saving an image
import cv2
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

    reference_y = 260
    reference_y_delta = 20
    height, width = image_thresh.shape
    middle_x = int(width / 2)
    minimum_number_white_pixels = 5

    image_stacked = verticalStacking(image=image_thresh,
                                     y_limits=[reference_y-reference_y_delta, reference_y + reference_y_delta])

    # search for white pixels in stacked image from lert to right
    white_xs = []
    for x in range(0, width):
        if image_stacked[x] > minimum_number_white_pixels:
            white_xs.append(x)

    groups = []
    # iterate through white_xs and create groups
    first = True
    group_idx = 0
    for x in white_xs:
        if first:
            group = {'idx': group_idx, 'xs': [x]}
            groups.append(group)
            group_idx += 1
            first = False
            continue

        # decide if a new group should be create
        last_x = groups[-1]['xs'][-1]
        if abs(x - last_x) > 1:  # create new group
            group = {'idx': group_idx, 'xs': [x]}
            groups.append(group)
            group_idx += 1
        else:
            groups[-1]['xs'].append(x)

    # Compute the average x for each group
    for group in groups:
        group['xavg'] = sum(group['xs']) / len(group['xs'])

    # Compute the distance between the average and the middle x
    for group in groups:
        group['dist_to_middle'] = abs(middle_x - group['xavg'])

    # select middle line as the group which is closed to the middle of the image
    smallest_distance = 9999
    for group in groups:
        if group['dist_to_middle'] < smallest_distance:
            smallest_distance = group['dist_to_middle']
            middle_line = group

    # print(middle_line)

    # -------------------------------------------
    # Drawing
    # -------------------------------------------
    for x in white_xs:
        cv2.line(image_gui, (x, reference_y), (x, reference_y), (0, 0, 255), 4)

    point = (int(middle_line['xavg']), reference_y)
    color = (0, 255, 255)
    cv2.line(image_gui, point, point, color, 4)
    cv2.putText(image_gui, 'ML', point, cv2.FONT_HERSHEY_SIMPLEX, 1, color, 1, cv2.LINE_AA)

    # make a driving decision
    angle = 0
    speed = 0.1

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
    # publisher.publish(twist)


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
