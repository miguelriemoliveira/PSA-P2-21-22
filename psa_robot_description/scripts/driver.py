#! /usr/bin/python3

# rospy for the subscriber
from copy import deepcopy
from re import I

import numpy as np
import rospy

# OpenCV2 for saving an image
import cv2
from colorama import Fore, Style
from vertical_stacking import verticalStacking
from geometry_msgs.msg import Twist

# ROS Image message
from sensor_msgs.msg import Image

# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError

# Global variables
# Instantiate CvBridge
bridge = CvBridge()


class Driver():

    def __init__(self):
        rospy.init_node('driver')

        # Define your image topic
        image_topic = "/front_camera/rgb/image_raw"

        # Set up your subscriber and define its callback
        self.subscriber = rospy.Subscriber(image_topic, Image, self.imageCallback)
        self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.decisionCallback)

        self.M = np.load('/home/mike/catkin_ws/src/PSA-P2-21-22/psa_robot_description/scripts/ipm_file.npy')

        self.middle_line = None
        self.left_line = None
        self.right_line = None
        self.groups = None
        self.image_gui = None
        self.angle_prev = None

    def decisionCallback(self, msg):
        # -------------------------------------------
        # Controller (make a driving decision)
        # -------------------------------------------
        if not self.middle_line == None:
            kp = 0.04
            # angle = kp * (self.image_gui.shape[1]/2 - self.middle_line['xavg'])
            angle = kp * (450 - self.middle_line['xavg'])

            if not self.angle_prev is None:
                diff = abs(self.angle_prev - angle)
                threshold = 1
                if diff > threshold:
                    print(Fore.RED + 'Angle variation too large!' + Style.RESET_ALL)
                    angle = self.angle_prev

            self.angle_prev = angle
            speed = 0.5

            # build a twist msg to publish
            twist = Twist()
            twist.linear.x = speed
            twist.angular.z = angle
            self.publisher.publish(twist)

            print("Decision angle=" + str(angle))

    def imageCallback(self, msg):
        print("Received an image!")

        try:
            # Convert your ROS Image message to OpenCV2
            image_rgb = bridge.imgmsg_to_cv2(msg, "bgr8")
        except:
            print('Could not convert image')
            return

        # Preprocessing
        image_rgb_ipm = cv2.warpPerspective(image_rgb, self.M, [900, 320])
        global image_gui
        self.image_gui = deepcopy(image_rgb_ipm)

        image_gray = cv2.cvtColor(image_rgb_ipm, cv2.COLOR_BGR2GRAY)  # convert to grayscale
        _, image_thresh = cv2.threshold(image_gray, 127, 255, cv2.THRESH_BINARY)  # thresholding

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

        self.groups = []
        self.middle_line = None
        self.left_line = None

        # iterate through white_xs and create groups
        first = True
        group_idx = 0
        for x in white_xs:
            if first:
                group = {'idx': group_idx, 'xs': [x]}
                self.groups.append(group)
                group_idx += 1
                first = False
                continue

            # decide if a new group should be create
            last_x = self.groups[-1]['xs'][-1]
            if abs(x - last_x) > 1:  # create new group
                group = {'idx': group_idx, 'xs': [x]}
                self.groups.append(group)
                group_idx += 1
            else:
                self.groups[-1]['xs'].append(x)

        # Compute the average x for each group
        for group in self.groups:
            group['xavg'] = sum(group['xs']) / len(group['xs'])

        # Compute the distance between the average and the middle x
        for group in self.groups:
            group['dist_to_middle'] = abs(middle_x - group['xavg'])

        # select middle line as the group which is closed to the middle of the image
        smallest_distance = 9999
        for group in self.groups:
            if group['dist_to_middle'] < smallest_distance:
                smallest_distance = group['dist_to_middle']
                self.middle_line = group

        if not self.middle_line is None:
            # Compute the distance between the middle line and avgx
            for group in self.groups:
                group['dist_to_middle_line'] = self.middle_line['xavg'] - group['xavg']

            # Find the left line a the first on the left of the middle line
            smallest_distance = 9999
            for group in self.groups:
                if group['dist_to_middle_line'] <= 0:
                    continue
                elif group['dist_to_middle_line'] < smallest_distance:
                    smallest_distance = group['dist_to_middle_line']
                    self.left_line = group

        # -------------------------------------------
        # Drawing
        # -------------------------------------------
        for x in white_xs:
            cv2.line(self.image_gui, (x, reference_y), (x, reference_y), (0, 0, 255), 4)

        if not self.middle_line == None:
            point = (int(self.middle_line['xavg']), reference_y)
            color = (0, 255, 255)
            cv2.line(self.image_gui, point, point, color, 4)
            cv2.putText(self.image_gui, 'ML', point, cv2.FONT_HERSHEY_SIMPLEX, 1, color, 1, cv2.LINE_AA)

        if not self.left_line == None:
            print('Drawing left line')
            point = (int(self.left_line['xavg']), reference_y)
            cv2.putText(self.image_gui, 'LL', point, cv2.FONT_HERSHEY_SIMPLEX, 1, color, 1, cv2.LINE_AA)

        # cv2.imshow('image_rgb', image_rgb)
        # cv2.imshow('image_gray', image_gray)
        # cv2.imshow('image_thresh', image_thresh)
        cv2.imshow('image_gui', self.image_gui)

        key = cv2.waitKey(10)
        print(key)
        if key == ord('q'):
            self.shutdown()

    def shutdown(self):

        print(Fore.RED + 'Shutting down' + Style.RESET_ALL)
        twist = Twist()  # build a twist msg to make sure the robot is stopped
        twist.linear.x = 0
        twist.angular.z = 0
        self.publisher.publish(twist)

        rospy.signal_shutdown("Because I want to")


def main():

    driver = Driver()
    rospy.spin()


if __name__ == '__main__':
    main()
