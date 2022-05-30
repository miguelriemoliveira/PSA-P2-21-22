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

import matplotlib.pyplot as plt




# Global variables
# Instantiate CvBridge
bridge = CvBridge()
publisher = None

global image_rgb


def imageCallback(msg):
    global image_rgb
    print("Received an image!")

    try:
        # Convert your ROS Image message to OpenCV2
        image_rgb = bridge.imgmsg_to_cv2(msg, "bgr8")
    except:
        print('Could not convert image')
        return

def click_event(event, x, y, flags, params):
     
    # checking for left mouse clicks
    if event == cv2.EVENT_LBUTTONDOWN:
 
        # displaying the coordinates
        # on the Shell
        print(x, ' ', y)
        
        font = cv2.FONT_HERSHEY_SIMPLEX
        b = image_rgb[y, x, 0]
        g = image_rgb[y, x, 1]
        r = image_rgb[y, x, 2]
        cv2.putText(image_rgb, str(b) + ',' +
                    str(g) + ',' + str(r),
                    (x,y), font, 1,
                    (255, 255, 0), 2)
        cv2.imshow('image', image_rgb)
 
 
    
    

def main():
    global image_rgb
    image_rgb = None
    rospy.init_node('driver')
    # Define your image topic
    image_topic = "/front_camera/rgb/image_raw"
    # Set up your subscriber and define its callback
    rospy.Subscriber(image_topic, Image, imageCallback)
    global publisher
    publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    
    
    

    while True:
        if image_rgb is not None:
            print(image_rgb.shape)
            cv2.imshow('image', image_rgb)
            cv2.setMouseCallback('image', click_event)
            cv2.waitKey(0)
            
            
    
    
    

    # Spin until ctrl + c
    #rospy.spin()


if __name__ == '__main__':
    main()
