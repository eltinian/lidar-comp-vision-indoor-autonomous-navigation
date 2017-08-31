#!/usr/bin/env python

from std_msgs.msg import String
#import roslib
import sys
import datetime
import time

import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import cv2

import rospy
from autobot.msg import drive_param

from sensor_msgs.msg import Image

bridge = CvBridge() 

i = 0
bridge = CvBridge(); 
currentImage = 0
currentAngle = 0
currentVel = 0
time = time.time()

def __init__(self):
    bridge = CvBridge(); 

def callback(temp):
    #print("current image updated")
    global currentImage
    try:
        currentImage = bridge.imgmsg_to_cv2(temp, desired_encoding="passthrough")
    except CvBridgeError as e:
        print(e)
    global currentAngle
    global currentVel
    global i
    if currentAngle > 0:
        filepath = "dataset2/right" + str(currentAngle) + "_" + str(i) + str(time) + ".png"
    elif currentAngle < 0:
        filepath = "dataset2/left" + str(currentAngle) + "_" + str(i) + str(time) + ".png"
    else:
        filepath = "dataset2/zero" + str(currentAngle) + "_" + str(i) + str(time) + ".png"
    i+=1
    if currentVel > 7.0:
        print("picture taken")
        cv2.imwrite(filepath, currentImage) 

def takePicture(data):
    
    #define file path
    global currentAngle
    global currentVel
    currentAngle = data.angle
    currentVel = data.velocity
    global i
    if currentAngle > 0:
        filepath = "dataset2/right" + str(currentAngle) + "_" + str(i) + str(time) + ".png"
    elif currentAngle < 0:
        filepath = "dataset2/left" + str(currentAngle) + "_" + str(i) + str(time) + ".png"
    else:
        filepath = "dataset2/zero" + str(currentAngle) + "_" + str(i) + str(time) + ".png"
    i+=1
    if currentVel > 7.0:
        print("picture taken")
        cv2.imwrite(filepath, currentImage) 

def listen():
    bridge = CvBridge();
    rospy.init_node('capture_image', anonymous=True)
    rospy.Subscriber("left/image_rect_color", Image, callback)  
    rospy.Subscriber("drive_parameters", drive_param, takePicture)
    rospy.spin()

if __name__ == '__main__':
    print("image capture initialized")
    print(time)
    listen()
