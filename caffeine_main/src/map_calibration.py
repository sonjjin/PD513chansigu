#!/usr/bin/env python
#-*- coding: utf-8 -*-


import numpy as np
import cv2
import math as m
from cv_bridge import CvBridge
from skimage.measure import label, regionprops
from utils import *
import os

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray


class map_calibaration():
    def __init__(self):
        self.sub_parkinglot = rospy.Subscriber('/camera/image_raw', Image, self.callback_img_parking) 
        self.pub_matrix = rospy.Publisher('/warp_matrix', Float32MultiArray, queue_size = 1)
        
        
        self.img_parking_path = None
        self.is_parking_path = False
        self.complete = False
        self.matrix = None
    
    def callback_img_parking(self, data):
        if not self.is_parkinglot:
            img = self.cv_bridge.imgmsg_to_cv2(data, 'rgb8') # ros image를 cv2로 받아오기
            self.img_parkinglot = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            # print("front",  self.cur_img_front.dtype) 
            self.is_parkinglot = True
    
    def publish_matrix(self, matrix):
        msg_matrix = Float32MultiArray()
        msg_matrix.data = matrix.reshape([-1])
        msg_matrix.layout.data_offset = 0
        msg_matrix.layout.dim = [MultiArrayDimension(), MultiArrayDimension()]
        
        msg_matrix.layout.dim[0].label = "row"
        msg_matrix.layout.dim[0].size = 3
        msg_matrix.layout.dim[0].stride = 9
        
        msg_matrix.layout.dim[1].label = "col"
        msg_matrix.layout.dim[1].size = 3
        msg_matrix.layout.dim[1].stride = 3
        self.pub_matrix.publish(msg_matrix)
    
    def process(self):
        if self.is_parkinglot:
            if not self.complete:
                img_parkinglot = self.img_parkinglot
                # print(img_parkinglot.shape)
                # cv2.imwrite('./dddd.png', img_parkinglot)
                img_parkinglot_hsv = hsv_parking(img_parkinglot, 'green')
                
                # cv2.imshow(img_parkinglot_hsv_2)
                # cv2.waitKey(1)
                label_parkinglot = label(img_parkinglot_hsv)
                regions = regionprops(label_parkinglot)
                center_point = []
                
                for props in regions:
                    y0, x0 = props.centroid
                    y0, x0 = round(y0), round(x0)
                    center_point.append([x0, y0])
                center_point.sort()
                # print(center_point) # ll, lu, ru, rl
                # y = cv2.line(y, (center_point[0][0], center_point[0][1]), (center_point[0][0], center_point[0][1]),(0,0,255),5)
                comp = 9
                comp2 = 6
                x_ll = 0 + comp2
                y_ll = 0 + comp2
                x_ru = 465 - comp
                y_ru = 186 - comp
                src = np.array(center_point, dtype=np.float32)
                dst = np.float32([(x_ll, y_ll),
                    (x_ll, y_ru),
                    (x_ru, y_ru),
                    (x_ru, y_ll)])
                
                try:    
                    M = cv2.getPerspectiveTransform(src, dst) # The transformation matrix
                    self.matrix = M
                    self.publish_matrix(self.matrix)
                    self.complete = True
                    
                except:
                    print('calibration error')
                    self.is_parkinglot = False
    
            else:
                self.publish_matrix(self.matrix)
                
                
        
    
if __name__ == '__maine__':
    rospy.init_node('ros_map_calibration')
    r = rospy.Rate(10)
    mc = map_calibaration()
    
    while not rospy.is_shutdown():        
        mc.process()
        r.sleep()

    rospy.spin()
        