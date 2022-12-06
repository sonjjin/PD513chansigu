#!/usr/bin/env python
#-*- coding: utf-8 -*-


import numpy as np
import cv2
import math as m
from cv_bridge import CvBridge
from skimage.measure import label, regionprops

from utils import hsv_parking
import os

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray


class find_property():
    def __init__(self, save_path):

        self.cv_bridge = CvBridge()

        self.sub_warp_matrix = rospy.Subscriber('/warp_matrix', Float32MultiArray, self.callback_warp_matrix) 
        self.sub_parkinglot = rospy.Subscriber('/camera/image_raw', Image, self.callback_img_parking) 
        
        self.pub_properties = rospy.Publisher('/properties', Float32MultiArray, queue_size = 1)
        

        self.iter = 0
        self.is_parkinglot = False
        self.img_parkinglot = None
        self.is_warp_matrix = False
        self.warp_matrix = None
        self.map_W = 465
        self.map_H = 443
        self.properties = np.zeros([3])
        self.is_red = False
        self.is_blue = False
        self.save_path = save_path
        
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

    def calibaration_map(self):
        M = self.warp_matrix # The transformation matrix
        img_parkinglot = self.img_parkinglot
        # Minv = cv2.getPerspectiveTransform(dst, src) # Inverse transformation

        warped_img = cv2.warpPerspective(img_parkinglot, M, (self.map_W, self.map_H))
        warped_img = cv2.flip(warped_img, 0)
        self.img_map = warped_img
        output = warped_img.copy()
        self.is_parkinglot = False
        return output

    def process(self):
        if self.is_warp_matrix:
            # print('find properties')
            warped_img = cv2.warpPerspective(self.img_parkinglot, self.warp_matrix, (self.map_W, self.map_H))
            warped_img = cv2.flip(warped_img, 0)
            img_parkinglot = warped_img
            # cv2.imwrite(self.save_path + '/parkinglot.png',img_parkinglot)
            img_red = hsv_parking(img_parkinglot, 'red')
            img_blue = hsv_parking(img_parkinglot, 'blue')
            cv2.imshow('parkinglot', img_parkinglot)
            cv2.waitKey(1)
            cv2.imwrite(self.save_path + '/parkinglot' + str(self.iter).zfill(4) + '.png', img_parkinglot)
            # cv2.imwrite(self.save_path + '/img_red.png', img_red)
            # cv2.imwrite(self.save_path + '/img_blue.png', img_blue)

            self.img_red = img_red
            self.img_blue = img_blue

            label_img_red = label(img_red)
            label_img_blue = label(img_blue)
            
            regions_red = regionprops(label_img_red)
            regions_blue = regionprops(label_img_blue)
            
            vector = []
            if len(regions_red) == 1:
                # print(regions_red[0].centroid)
                y0, x0 = regions_red[0].centroid[0], regions_red[0].centroid[1]
                y0, x0 = round(y0), round(x0)
                vector.append([x0, y0])
                self.is_red = True
                # img = cv2.line(img, (x0, y0), (x0, y0),(0,0,255),5)
            
            if len(regions_blue) == 1:
                # print(regions_blue[0].centroid[0])
                y0, x0 = regions_blue[0].centroid[0], regions_blue[0].centroid[1]
                y0, x0 = round(y0), round(x0)
                vector.append([x0, y0])
                self.is_blue = True
                # img = cv2.line(img, (x0, y0), (x0, y0),(255,0,0),5) 
            # print(cX, cY)

            if self.is_red and self.is_blue:
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
                # print(self.properties)

                

                
            else:
                try:
                    if self.iter != 0:
                        cX = self.properties[0]
                        cY = self.properties[1]
                        angle_deg = self.properties[2]
                        self.is_parkinglot = False
                        # print('complete properties')
                        # print(self.properties)
                    else:
                        if self.iter == 0:
                            self.properties[0] = 400
                            self.properties[1] = 73
                            self.properties[2] = 180
                        cX = self.properties[0]
                        cY = self.properties[1]
                        angle_deg = self.properties[2]
                        # print('complete properties')
                        # print(self.properties)


                except:
                    if self.iter != 0:
                        cX = self.properties[0]
                        cY = self.properties[1]
                        angle_deg = self.properties[2]
                        self.is_parkinglot = False
                        # print('complete properties')
                        # print(self.properties)
                    else:
                        if self.iter == 0:
                            self.properties[0] = 400
                            self.properties[1] = 73
                            self.properties[2] = 180
                        cX = self.properties[0]
                        cY = self.properties[1]
                        angle_deg = self.properties[2]
                        # print(self.properties)

                    # print('error properties')


            print('X: {0:0.3f}, y: {1:0.3f}, angle: {2:0.3f}'.format(self.properties[0], self.properties[1], self.properties[2]))
            self.properties = np.array([cX, cY, angle_deg])
            output_pub = Float32MultiArray()
            output_pub.data = self.properties
            self.pub_properties.publish(output_pub)
            # print('complete properties')
            self.iter += 1
            self.is_red = False
            self.is_blue = False
            self.is_parkinglot = False

                
        
    
if __name__ == '__maine__':
    rospy.init_node('ros_map_calibration')
    r = rospy.Rate(20)
    fp = find_property()
    
    while not rospy.is_shutdown():        
        fp.process()
        r.sleep()

    rospy.spin()
        