#! /usr/bin/python3

# rospy for the subscriber
from copy import deepcopy


import numpy as np
import rospy

# OpenCV2 for saving an image
import cv2
from colorama import Fore, Style

from geometry_msgs.msg import Twist

# ROS Image message
from sensor_msgs.msg import Image

# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError

from model import CNN
import torch
from PIL import Image as ImagePIL
from torchvision import transforms

# Global variables
# Instantiate CvBridge
bridge = CvBridge()


class DriverDL():

    def __init__(self):
        rospy.init_node('driver')

        # Define your image topic
        image_topic = "/front_camera/rgb/image_raw"

        # Set up your subscriber and define its callback
        
        #self.timer = rospy.Timer(rospy.Duration(0.1), self.decisionCallback)

        self.folder = '/home/danc/Desktop/model_psa_1306_aula2/model.pth'

        self.model = CNN()
        self.model.load_state_dict(torch.load(self.folder))
        self.model.eval()
        
        self.cuda_available = torch.cuda.is_available()
        
        if self.cuda_available:
            self.model.cuda()
        
        
        self.rgb_transforms = transforms.Compose([
        transforms.Resize((66,200)),
        transforms.ToTensor(),
        transforms.Normalize([0.19054175422901043, 0.19067991892465383, 0.19061099275088064], [0.3224361491843105, 0.3223948440593156, 0.32235567748029115])
    ])

        self.image_input = None
        
        
        
        self.subscriber = rospy.Subscriber(image_topic, Image, self.imageCallback)
        self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    
    def imageCallback(self, msg):
        print("Received an image!")

        try:
            # Convert your ROS Image message to OpenCV2
            image_bgr = bridge.imgmsg_to_cv2(msg, "bgr8")
        except:
            print('Could not convert image')
            return

        #image_bgr = cv2.imread(f'/home/danc/Desktop/psa_1306_aula/00000.jpg', cv2.IMREAD_UNCHANGED)

        image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
        
        # convert numpy to PIL image
        PIL_image = ImagePIL.fromarray(image_rgb, mode='RGB')
        
        image = self.rgb_transforms(PIL_image)
        image = torch.unsqueeze(image, 0)
        
        print(image.shape)
        
        self.image_input = image
        
        
        

    
    def decisionMaking(self):
        # -------------------------------------------
        # Controller (make a driving decision)
        # -------------------------------------------
        
        if self.image_input is not None:
            if self.cuda_available:
                image_input = self.image_input.cuda()
            
            #steering = self.model(self.image_input).cpu().detach().numpy()
            steering = self.model(image_input)
            steering = steering.cpu()
            steering = steering.detach()
            steering = steering.numpy()
            
            
            return float(steering)

        
        
       
    
    def applyControls(self, steering, velocity):
         # build a twist msg to publish
        twist = Twist()
        twist.linear.x = velocity
        twist.angular.z = steering
        self.publisher.publish(twist)

        print("Decision angle=" + str(steering))

        

    def shutdown(self):

        print(Fore.RED + 'Shutting down' + Style.RESET_ALL)
        twist = Twist()  # build a twist msg to make sure the robot is stopped
        twist.linear.x = 0
        twist.angular.z = 0
        self.publisher.publish(twist)

        rospy.signal_shutdown("Because I want to")


def main():
    
    driver = DriverDL()
    
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
    
        steering = driver.decisionMaking()
        driver.applyControls(steering=steering, velocity=0.6)
        rate.sleep()

   
    
    
    
    


if __name__ == '__main__':
    main()
