#! /usr/bin/python3

# rospy for the subscriber
from colorsys import TWO_THIRD
import rospy

# OpenCV2 for saving an image
import cv2
from geometry_msgs.msg import Twist

# ROS Image message
from sensor_msgs.msg import Image

# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
import pandas as pd
import signal

class DataCollector():
    def __init__(self):
        
        self.image_topic = "/front_camera/rgb/image_raw"
        self.control_topic = "cmd_vel"
        rospy.init_node('driver')
        self.bridge = CvBridge()
        self.csv = pd.DataFrame(columns=['filename', 'angle', 'speed'])
        self.idx = 0
        
        self.folder = '/home/danc/Desktop/psa_dataset_1306'
        
        signal.signal(signal.SIGINT, self.saveCSV)
    
    def saveFrame(self):
        filename = f'{self.idx:04d}.png'
        image = self.getImage()
        
        commands = self.getControlCommands()
        
        self.addRow(commands, filename)
        
        cv2.imwrite(f'{self.folder}/{filename}', image)
        
        self.step()
        
        # save row
    
    def getImage(self):
        msg = rospy.wait_for_message(self.image_topic, Image)
        image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        return image

    def getControlCommands(self):
         msg = rospy.wait_for_message(self.control_topic, Twist)
         return {'angle' : msg.angular.z,
                 'speed' : msg.linear.x}
         
    def addRow(self, msg, filename):
        
        row = {'filename': filename,
               'angle'   : msg['angle'],
               'speed'   : msg['speed']}
        
        print(row)
        self.csv = self.csv.append(row, ignore_index=True)
    
    
    def step(self):
        self.idx += 1
    
    def saveCSV(self, signum, frame):
        print('saving csv and aborting...')
        self.csv.to_csv(f'{self.folder}/data.csv', index=False)
        exit(0)
     
 
def main():
    
    
    data_collector = DataCollector()
    
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        print(data_collector.idx)
        
        data_collector.saveFrame()
        
        #print(data_collector.csv)
        
        rate.sleep()
    


if __name__ == '__main__':
    main()
