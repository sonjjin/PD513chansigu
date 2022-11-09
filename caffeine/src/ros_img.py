#!/usr/bin/env python
#-*- coding: utf-8 -*-

import numpy as np
import cv2
from cv_bridge import CvBridge
import matplotlib.pyplot as plt
import os

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32

class path:
    def __init__(self):
        self.cv_bridge = CvBridge()
        self.img = rospy.Subscriber('/img', Image, self.img_callback)
        self.cur_img = None
        
    def img_callback(self, data):
        img = self.cv_bridge.imgmsg_to_cv2(data, 'rgb8') # ros image를 cv2로 받아오기
        self.cur_img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        # print("front",  self.cur_img_front.dtype) 

    def main(self):
        img = self.cur_img
        cv2.imshow(img)
    
     
        
if __name__ == '__main__':
    rospy.init_node('lane_detection')
    r = rospy.Rate(10)
    rt = path()

    while not rospy.is_shutdown():        
        rt.main()
        r.sleep()
    
    rospy.spin()
        
        
    
    
