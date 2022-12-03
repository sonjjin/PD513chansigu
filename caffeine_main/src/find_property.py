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


class find_property():
    def __init__(self):
        self.sub_warp_matrix = rospy.Subscriber('/warp_matrix', Float32MultiArray, self.callback_warp_matrix) 
        self.sub_parkinglot = rospy.Subscriber('/camera/image_raw', Image, self.callback_img_parking) 
        
        self.pub_properties = rospy.Publisher('/properties', Float32MultiArray, queue_size = 1)
        

        self.is_parkinglot = False
        self.img_parkinglot = None
        self.is_warp_matrix = False
        self.warp_matrix = None
        
    def callback_warp_matrix(self, data):
        if not self.is_warp_matrix:
            self.warp_matrix =  np.array(data.data).reshape([data.layout.dim[0].size, data.layout.dim[1].size])
            self.is_warp_matrix = True
    
    def callback_img_parking(self, data):
        if not self.is_parkinglot:
            img = self.cv_bridge.imgmsg_to_cv2(data, 'rgb8') # ros image를 cv2로 받아오기
            self.img_parkinglot = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            # print("front",  self.cur_img_front.dtype) 
            self.is_parkinglot = True

    
    def process(self):
        if self.is_warp_matrix:
            img_parkinglot = self.calibration_map()
            # cv2.imwrite(self.save_path + '/parkinglot.png',img_parkinglot)
            img_red = self.hsv_parking(img_parkinglot, 'red')
            img_blue = self.hsv_parking(img_parkinglot, 'blue')

            # cv2.imwrite(self.save_path + '/img_red.png', img_red)
            # cv2.imwrite(self.save_path + '/img_blue.png', img_blue)

            self.img_red = img_red
            self.img_blue = img_blue

            label_img_red = label(img_red)
            label_img_blue = label(img_blue)
            
            regions_red = regionprops(label_img_red)
            regions_blue = regionprops(label_img_blue)
            
            vector = []
            output = {}
            if len(regions_red) == 1:
                # print(regions_red[0].centroid)
                y0, x0 = regions_red[0].centroid[0], regions_red[0].centroid[1]
                y0, x0 = round(y0), round(x0)
                vector.append([x0, y0])
                is_red = True
                # img = cv2.line(img, (x0, y0), (x0, y0),(0,0,255),5)
            
            if len(regions_blue) == 1:
                # print(regions_blue[0].centroid[0])
                y0, x0 = regions_blue[0].centroid[0], regions_blue[0].centroid[1]
                y0, x0 = round(y0), round(x0)
                vector.append([x0, y0])
                is_blue = True
                # img = cv2.line(img, (x0, y0), (x0, y0),(255,0,0),5) 
            # print(cX, cY)
            try:
                cX = int(round((vector[0][0] + vector[1][0])/2))
                cY = int(round((vector[0][1] + vector[1][1])/2))
                X1_t = vector[0][0]
                X2_t = vector[1][0]
                Y1_t = 443 - vector[0][1]
                Y2_t = 443 - vector[1][1]
                X_t = X2_t - X1_t
                Y_t = Y2_t - Y1_t
                # print(X1_t, vector[0][1])
                
                angle = m.atan2(Y_t, X_t) + m.pi
                # if angle < 0:
                #     angle = -angle
                angle_deg = angle * 180 / m.pi
                
                # print(angle_deg)
                
            except:
                if self.is_cur_pose and self.is_angle and self.is_blue:
                    cX = self.cur_pose[0]
                    cX = self.cur_pose[1]
                    if self.is_angle[0] is not None:
                        angle_deg = self.cur_angle


            output["vehicle_center"] = [cX, cY]
            output["angle"] = angle_deg
            output_vc = Float32MultiArray()
            output_angle = angle_deg
            output_vc.data = output["vehicle_center"]

            self.pub_vehicle_center.publish(output_vc)
            self.pub_vehicle_angle.publish(output_angle)
            
            # print(img_parkinglot.shape)
            # img_parkinglot = cv2.cvtColor(img_parkinglot, cv2.COLOR_)
            self.img_map = cv2.line(img_parkinglot, (int(vector[0][0]), int(vector[0][1])), (int(vector[1][0]), int(vector[1][1])),(0,255,0), 4)
            # cv2.imwrite('prop.png', img)
            # cv2.waitKey(1)
            self.is_parkinglot = False
            
            return output
            
                
                
        
    
if __name__ == '__maine__':
    rospy.init_node('ros_map_calibration')
    r = rospy.Rate(10)
    fp = find_property()
    
    while not rospy.is_shutdown():        
        fp.process()
        r.sleep()

    rospy.spin()
        