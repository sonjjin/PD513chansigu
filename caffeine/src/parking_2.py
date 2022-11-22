#!/usr/bin/env python
#-*- coding: utf-8 -*-

import numpy as np
import cv2
import math as m
from cv_bridge import CvBridge
from skimage.measure import label, regionprops

import matplotlib.pyplot as plt
import os
import csv

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray

class parking:
    def __init__(self):
        self.cv_bridge = CvBridge()
        
        self.sub_parking_path = rospy.Subscriber('/img_w_path', Image, self.img_path_callback)
        self.sub_parkinglot = rospy.Subscriber('/camera/image_raw', Image, self.img_parking_callback)
        self.sub_coord_x = rospy.Subscriber('/coord_x', Float32MultiArray, self.coord_x_callback) # cm
        self.sub_coord_y = rospy.Subscriber('/coord_y', Float32MultiArray, self.coord_y_callback) # cm
        self.sub_coord_ang = rospy.Subscriber('/coord_ang', Float32MultiArray, self.coord_ang_callback)
        self.sub_accX = rospy.Subscriber('/arduino_imu/accX', Float32, self.accel_x_callback) # cm/s
        self.sub_accY = rospy.Subscriber('/arduino_imu/accY', Float32, self.accel_y_callback) # cm/s
        self.sub_aglZ = rospy.Subscriber('/arduino_imu/aglZ', Float32, self.agl_callback) # degree sum
        
        self.pub_ctrl_motor = rospy.Publisher('/arduino_ctrl/ctrl_motor', Float32, queue_size=1)
        self.pub_ctrl_servo = rospy.Publisher('/arduino_ctrl/ctrl_servo', Float32, queue_size=1)
        
        # Path
        self.coord_x = None
        self.coord_y = None
        self.coord_ang = None
        
        self.is_coord_x = False
        self.is_coord_y = False
        self.is_coord_ang = False
        
        # imu
        self.accel_x = None
        self.accel_y = None
        self.agl = None
        
        self.is_accel_x = False
        self.is_accel_y = False
        self.is_agl = False
        
        # imgs
        self.img_parking_path = None
        self.is_parking_path = False
        
        self.img_parkinglot = None
        self.is_parkinglot = False

        self.is_red = False
        self.is_blue = False

        self.img_map = None
        self.map_W = 465
        self.map_H = 443
        self.m_per_pixel = 0.00252
        
        self.steer = None

        self.img_path = None
        self.img_vector = None
        self.img_roi = None
        self.img_red = None
        self.img_blue = None
        self.iter = 0


    """
    callback functions
    """
    def coord_x_callback(self, data):
        if not self.is_coord_x:
            self.coord_x = data
            self.is_coord_x = True
        
    def coord_y_callback(self, data):
        if not self.is_coord_y:
            self.coord_y = data
            self.is_coord_y = True
        
    def coord_ang_callback(self, data):
        if not self.is_coord_ang:
            self.coord_ang = data
            self.is_coord_ang = True
    
    def img_parking_callback(self, data):
        if not self.is_parkinglot:
            img = self.cv_bridge.imgmsg_to_cv2(data, 'rgb8') # ros image를 cv2로 받아오기
            self.img_parkinglot = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            # print("front",  self.cur_img_front.dtype) 
            self.is_parkinglot = True
    
    def img_path_callback(self, data):
        if not self.is_parking_path:
            img = self.cv_bridge.imgmsg_to_cv2(data, 'rgb8') # ros image를 cv2로 받아오기
            self.img_parking_path = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            # print("front",  self.cur_img_front.dtype) 
            self.is_parking_path = True

    def accel_x_callback(self, data):
        if not self.is_accel_x:
            self.accel_x = data
            self.is_accel_x = True

    def accel_y_callback(self, data):
        if not self.is_accel_x:
            self.accel_y = data
            self.is_accel_y = True

    def agl_callback(self, data):
        if not self.is_agl:
            self.agl = data
            self.is_agl = True
    
    """
    hsv
    """
    def hsv(self, img, color='yellow'):
        hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

        if color == 'green':
            mask = cv2.inRange(hsv, (25, 60, 50), (80, 255, 255))
            imask = mask > 0
            output = np.zeros_like(hsv, np.uint8)
            output[imask] = 255
            output = cv2.cvtColor(output, cv2.COLOR_RGB2GRAY)
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
            output = cv2.morphologyEx(output, cv2.MORPH_OPEN, kernel)
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))

            output = cv2.morphologyEx(output, cv2.MORPH_DILATE, kernel)
            # output = cv2.cvtColor(output, cv2.COLOR_GRAY2BGR)
            return output
        
        elif color == 'red':
            mask = cv2.inRange(hsv, (110, 50, 50), (150, 255, 255))

        elif color == 'blue':
            mask = cv2.inRange(hsv, (10, 100, 140), (53, 255, 255))
            imask = mask > 0
            output = np.zeros_like(hsv, np.uint8)
            output[imask] = 255
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))
            output = cv2.morphologyEx(output, cv2.MORPH_OPEN, kernel)
            # kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (24, 24))
            # output = cv2.morphologyEx(output, cv2.MORPH_OPEN, kernel)
            # output = cv2.cvtColor(output, cv2.COLOR_GRAY2BGR)
            return output

        elif color == 'yellow':
            mask = cv2.inRange(hsv, (80, 40, 145), (150, 255, 255))

        elif color == 'purple':
            mask = cv2.inRange(hsv, (130, 170, 130), (180, 255, 150))
            imask = mask > 0
            output = np.zeros_like(hsv, np.uint8)
            output[imask] = 255
            # mask = cv2.inRange(hsv, (80, 100, 145), (150, 255, 255))
            return output
        imask = mask > 0
        output = np.zeros_like(hsv, np.uint8)
        output[imask] = 255
        output = cv2.cvtColor(output, cv2.COLOR_RGB2GRAY)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        output = cv2.morphologyEx(output, cv2.MORPH_OPEN, kernel)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (20, 20))
        output = cv2.morphologyEx(output, cv2.MORPH_OPEN, kernel)
        # output = cv2.cvtColor(output, cv2.COLOR_GRAY2BGR)
        return output

    """
    calibarion mapping
    """
    def calibration_map(self):
        img_parkinglot = self.img_parkinglot
        # print(img_parkinglot.shape)
        # cv2.imwrite('./dddd.png', img_parkinglot)
        img_parkinglot_hsv = self.hsv(img_parkinglot, 'green')
        img_parkinglot_hsv_2 = cv2.cvtColor(img_parkinglot_hsv, cv2.COLOR_GRAY2BGR)
        cv2.imwrite('/home/hellobye/catkin_ws/src/caffeine/src/images/green_point.png',img_parkinglot_hsv_2)
        
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
        M = cv2.getPerspectiveTransform(src, dst) # The transformation matrix
        # Minv = cv2.getPerspectiveTransform(dst, src) # Inverse transformation


        warped_img = cv2.warpPerspective(img_parkinglot, M, (self.map_W, self.map_H))
        warped_img = cv2.flip(warped_img, 0)
        self.img_map = warped_img
        output = warped_img.copy()
        self.is_parkinglot = False
        return output

    """
    find the head of vehicle
    """
    def find_property(self):
        img_parkinglot = self.calibration_map()
        cv2.imwrite('/home/hellobye/catkin_ws/src/caffeine/src/images/parkinglot.png',img_parkinglot)
        img_red = self.hsv(img_parkinglot, 'red')
        img_blue = self.hsv(img_parkinglot, 'blue')

        cv2.imwrite('/home/hellobye/catkin_ws/src/caffeine/src/images/img_red.png', img_red)
        cv2.imwrite('/home/hellobye/catkin_ws/src/caffeine/src/images/img_blue.png', img_blue)

        self.img_red = img_red
        self.img_blue = img_blue

        label_img_red = label(img_red)
        label_img_blue = label(img_blue)
        
        regions_red = regionprops(label_img_red)
        regions_blue = regionprops(label_img_blue)
        
        vector = []
        output = {}
        if len(regions_red) == 1:
            y0, x0 = regions_red[0].centroid
            y0, x0 = round(y0), round(x0)
            vector.append([x0, y0])
            # img = cv2.line(img, (x0, y0), (x0, y0),(0,0,255),5)
        
        if len(regions_blue) == 1:
            # print(regions_blue[0].centroid[0])
            y0, x0 = regions_blue[0].centroid[0], regions_blue[0].centroid[1]
            y0, x0 = round(y0), round(x0)
            vector.append([x0, y0])
            # img = cv2.line(img, (x0, y0), (x0, y0),(255,0,0),5) 
        # print(cX, cY)
        cX = int(round((vector[0][0] + vector[1][0])/2))
        cY = int(round((vector[0][1] + vector[1][1])/2))
        
        X1_t = vector[0][0]
        X2_t = vector[1][0]
        Y1_t = 443 - vector[0][1]
        Y2_t = 443 - vector[1][1]
        X_t = X2_t - X1_t
        Y_t = Y2_t - Y1_t
        angle = m.atan2(Y_t, X_t)
        if angle < 0:
            angle = -angle
        
        angle_deg = angle * 180 / m.pi
        
        output["vehicle_center"] = [cX, cY]
        output["angle"] = angle_deg
        # print(img_parkinglot.shape)
        # img_parkinglot = cv2.cvtColor(img_parkinglot, cv2.COLOR_)
        img = cv2.line(img_parkinglot, (int(vector[0][0]), int(vector[0][1])), (int(vector[1][0]), int(vector[1][1])),(0,255,0), 4)
        cv2.imwrite('prop.png', img)
        cv2.waitKey(1)
        return output
    
    def get_roi(self, target):
        cX = target["vehicle_center"][0]
        cY = target["vehicle_center"][1]
        agl_glo = target["angle"]
        agl = (180 - agl_glo) # turn angle
        agl = agl_glo
        print(agl)
        
        img_parking_path = self.img_parking_path
        # img_parking_path = self.img_parkinglot

        cv2.imwrite('/home/hellobye/catkin_ws/src/caffeine/src/images/img_parking_path_ori.png', img_parking_path)

        img_parking_path = self.hsv(img_parking_path, 'purple')
        img_parking_path = cv2.cvtColor(img_parking_path, cv2.COLOR_BGR2GRAY)
        # cv2.imwrite('/home/hellobye/catkin_ws/src/caffeine/src/images/img_parking_path.png', img_parking_path)

        # print(img_parking_path.shape)
        # img_parking_path = self.img_parkinglot
        dx = self.map_W/2 - cX
        dy = self.map_H/2 - cY
        # print(dx, dy)
        mtrx = np.float32([[1, 0, dx],
                           [0, 1, dy]])
        img_trans = cv2.warpAffine(img_parking_path, mtrx, (self.map_W, self.map_H))   
        cv2.imwrite('./img_trans.png', img_trans)
        M = cv2.getRotationMatrix2D((self.map_W/2, self.map_H/2), agl, 1.0)
        rotate_img = cv2.warpAffine(img_trans, M, (self.map_W, self.map_H))
        roi_region = 400
        # print(cX, cY)
        cX_img = self.map_W/2
        cY_img = self.map_H/2
        rotate_img_save = cv2.line(rotate_img, (cX_img,cY_img), (cX_img,cY_img), (255,255,0), 3)
        cv2.imwrite('/home/hellobye/catkin_ws/src/caffeine/src/images/img_rotate.png', rotate_img_save)

          
        roi_x_under = int(cX_img-roi_region)
        roi_x_upper = int(cX_img)
        roi_y_under = int(cY_img-roi_region/2)
        roi_y_upper = int(cY_img+roi_region/2)
        if roi_x_under < 0:
            roi_x_under = 0
        if roi_y_under < 0:
            roi_y_under = 0
        # print(roi_x_under, roi_x_upper)
        # print(roi_y_under, roi_y_upper)
        cv2.imwrite('/home/hellobye/catkin_ws/src/caffeine/src/images/img_rotate.png', rotate_img)
        roi = rotate_img[roi_y_under:roi_y_upper, roi_x_under:roi_x_upper]
        roi = rotate_img
        cv2.imwrite('/home/hellobye/catkin_ws/src/caffeine/src/images/img_parking_path.png', img_parking_path)
        cv2.imwrite('/home/hellobye/catkin_ws/src/caffeine/src/images/roi.png',roi)
        # roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        self.img_roi = roi
        return roi
    
    
    def get_steer(self):
        self.calibration_map()
        target = self.find_property()
        roi = self.get_roi(target)
        
        gain_cte = 0.8
        gain_curv = -1
        look_a_head = roi.shape[0]*0.2
        
        path_idx = np.nonzero(roi)
        path_fit = np.polyfit(path_idx[0], path_idx[1], 2)
        # print(path_fit)
        # ploty = np.linspace(0, roi.shape[1]-1, roi.shape[1]) # y value
        # path_fitx = path_fit[0]*ploty**2+path_fit[1]*ploty+path_fit[2]
        # print(path_fitx)
        # path_fit_idx = np.stack((path_fitx, ploty), axis = -1).astype(int)
        # look_a_head = 70
        front_lane = path_fit[0]*look_a_head**2 + path_fit[1]*look_a_head + path_fit[2]
        front_curverad = ((1 + (2*path_fit[0]*look_a_head + path_fit[1])**2)**1.5) / (2*path_fit[0]) * self.m_per_pixel
        cte = look_a_head - front_lane
        steer = gain_cte * cte + gain_curv / front_curverad
        steer = max(min(steer, 20.0), -20.0)
        self.steer = -steer
        
        

    def parking(self):
        # if self.is_coord_x == True and self.is_coord_y == True and self.is_coord_ang == True:
        #     for i  in range(self.coord_x.shape[0] - 1):
        #         dx = self.coord_x[i+1] - self.coord_x[i]
        #         dy = self.coord_y[i+1] - self.coord_y[i]
        #         # ax =
        # if self.is_accel_x and self.is_accel_y and self.is_agl and self.is_img:
        if self.is_parking_path and self.is_parkinglot:
            self.get_steer()
            print(self.steer)
            
            cv2.imshow('roi', self.img_roi)
            # cv2.imshow('red', self.img_red)
            # cv2.imshow('blue', self.img_blue)

            cv2.imshow('map', self.img_map)
            cv2.imwrite('/home/hellobye/catkin_ws/src/caffeine/src/seq/roi/exp2/'+'img_roi'+ str(self.iter).zfill(4)+'.png', self.img_roi)
            cv2.imwrite('/home/hellobye/catkin_ws/src/caffeine/src/seq/vector2/exp2/'+'img_rotate'+ str(self.iter).zfill(4)+'.png', self.img_map)

            # print(self.iter)
            self.iter = self.iter+1
        else:
            print('wait for all receiving')
            
        if self.steer is not None:
            self.pub_ctrl_servo.publish(self.steer)
     
        
if __name__ == '__main__':
    rospy.init_node('ros_parking')
    r = rospy.Rate(10)
    pc = parking()

    while not rospy.is_shutdown():        
        pc.parking()
        r.sleep()

    rospy.spin()
        
        
    
    
