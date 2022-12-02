#!/usr/bin/env python
#-*- coding: utf-8 -*-


import numpy as np
import cv2
import math as m
from cv_bridge import CvBridge
from skimage.measure import label, regionprops
import time

import matplotlib.pyplot as plt
import os
import csv

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray

class Parking:
    def __init__(self, save_path):
        self.cv_bridge = CvBridge()
        
        self.sub_parking_path = rospy.Subscriber('/img_w_path1', Image, self.callback_img_path) # MATLAB
        self.sub_parkinglot = rospy.Subscriber('/camera/image_raw', Image, self.callback_img_parking) 
        self.sub_turn_dis = rospy.Subscriber('/turn_dis', Float32, self.callback_turn_dis)

        self.sub_accX = rospy.Subscriber('/arduino_imu/accX', Float32, self.callback_accel_x) # cm/s
        self.sub_accY = rospy.Subscriber('/arduino_imu/accY', Float32, self.callback_accel_y) # cm/s
        self.sub_aglZ = rospy.Subscriber('/arduino_imu/aglZ', Float32, self.callback_agl) # degree sum
        
        self.sub_img_front = rospy.Subscriber('/front_cam/image_raw', Image, self.callback_img_front)
        self.sub_img_left = rospy.Subscriber('/left_cam/image_raw', Image, self.callback_img_left)
        self.sub_img_right = rospy.Subscriber('/right_cam/image_raw', Image, self.callback_img_right)
        self.sub_img_back = rospy.Subscriber('/rear_cam/image_raw', Image, self.callback_img_back)
        self.sub_pv_distance = rospy.Subscriber('/vehicle_dis', Float32, self.callback_pv_dis)
        
        self.sub_cur_speed = rospy.Subscriber('/arduino_ctrl/ctrl_motor', Float32, self.callback_speed)
        self.sub_cur_steer = rospy.Subscriber('/arduino_ctrl/ctrl_servo', Float32, self.callback_steer)
        self.sub_cur_pose = rospy.Subscriber('/vehicle_point', Float32MultiArray, self.callback_vehicle_pose)
        self.sub_cur_angle = rospy.Subscriber('/vehicle_angle', Float32, self.callback_vehicle_angle)

        self.pub_ctrl_motor = rospy.Publisher('/arduino_ctrl/ctrl_motor', Float32, queue_size=1)
        self.pub_ctrl_servo = rospy.Publisher('/arduino_ctrl/ctrl_servo', Float32, queue_size=1)
        self.pub_vehicle_center = rospy.Publisher('/vehicle_point', Float32MultiArray, queue_size=1)
        self.pub_vehicle_angle = rospy.Publisher('/vehicle_angle', Float32MultiArray, queue_size=1)

        
        
        self.save_path = save_path
        self.local_coord = np.zeros([465, 443])
        
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
        
        self.pv_dis = None
        self.is_pv_dis = False
        
        self.turn_dis = None
        self.is_turn_dis = False
        
        # imgs
        self.img_parking_path = None
        self.is_parking_path = False
        
        self.img_parkinglot = None
        self.is_parkinglot = False

        self.is_angle = False
        self.is_blue = False
        
        self.is_front = False
        self.is_left = False
        self.is_right = False
        self.is_back = False
        
        self.cur_img_front = None
        self.cur_img_left = None
        self.cur_img_right = None
        self.cur_img_back = None
        

        self.forward_src = np.float32([
                    (125, 180),
                    (0, 440),
                    (500, 180),
                    (640, 440)
                ])

        self.left_src = np.float32([    
                    (100, 45),  
                    (5,415),    
                    (510, 45),  
                    (620, 410)  
                ])  
    
        self.right_src = np.float32([
                    (100, 45),
                    (5,415),
                    (510, 45),
                    (620, 410)
                ])

        self.forward_dst = np.float32([
                    (150, 90),
                    (170, 440),
                    (560, 90),
                    (470, 445)
                ])    

        self.left_dst = np.float32([
                    (140, 60),
                    (140, 460),
                    (480, 65),
                    (480, 445)
                ])
                
        self.right_dst = np.float32([
                    (140, 60),
                    (140, 450),
                    (480, 65),
                    (480, 460)
                ])

        # image
        self.img_map = None
        self.map_W = 465
        self.map_H = 443
        self.m_per_pixel = 0.00252
        
        # control
        self.steer = None
        self.speed = None
        
        self.is_cur_speed = False
        self.cur_speed = None
        self.is_cur_steer = False
        self.cur_steer = None
        self.is_cur_pose = False
        self.cur_pose = [400, 30]
        self.is_angle = False
        self.cur_angle = m.pi

        
        
        self.img_path = None
        self.img_vector = None
        self.img_roi = None
        self.img_red = None
        self.img_blue = None
        self.iter = 0
        self.cte = 0
        self.agl_cal = float(0)
        self.agl_init = float(0)
        self.start_time = time.time()


    """
    callback functions
    """
    # def callback_coord_x(self, data):
    #     if not self.is_coord_x:
    #         self.coord_x = data
    #         self.is_coord_x = True
        
    # def callback_coord_y(self, data):
    #     if not self.is_coord_y:
    #         self.coord_y = data
    #         self.is_coord_y = True
        
    # def callback_coord_ang(self, data):
    #     if not self.is_coord_ang:
    #         self.coord_ang = data
    #         self.is_coord_ang = True
    
    def callback_img_parking(self, data):
        if not self.is_parkinglot:
            img = self.cv_bridge.imgmsg_to_cv2(data, 'rgb8') # ros image를 cv2로 받아오기
            self.img_parkinglot = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            # print("front",  self.cur_img_front.dtype) 
            self.is_parkinglot = True
    
    def callback_img_path(self, data):
        if not self.is_parking_path:
            img = self.cv_bridge.imgmsg_to_cv2(data, 'rgb8') # ros image를 cv2로 받아오기
            self.img_parking_path = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            # print("front",  self.cur_img_front.dtype) 
            self.is_parking_path = True

    def callback_accel_x(self, data):
        if not self.is_accel_x:
            self.accel_x = data
            self.is_accel_x = True

    def callback_accel_y(self, data):
        if not self.is_accel_x:
            self.accel_y = data
            self.is_accel_y = True

    def callback_agl(self, data):
        if not self.is_agl:
            self.agl = data
            self.is_agl = True
            
    def callback_img_front(self, data):
        img = self.cv_bridge.imgmsg_to_cv2(data, 'rgb8') # ros image를 cv2로 받아오기
        self.cur_img_front = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        # print("front",  self.cur_img_front.dtype) 
        self.is_front = True

    def callback_img_left(self, data):
        img = self.cv_bridge.imgmsg_to_cv2(data, 'rgb8')
        self.cur_img_left = cv2.cvtColor(img, cv2.COLOR_BGR2RGB) 
        # print("left", self.cur_img_left.dtype) 
        self.is_left = True
    
    def callback_img_right(self, data):
        img = self.cv_bridge.imgmsg_to_cv2(data, 'rgb8')
        self.cur_img_right = cv2.cvtColor(img, cv2.COLOR_BGR2RGB) 
        # print("right", self.cur_img_right.dtype) 
        self.is_right = True
    
    def callback_img_back(self, data):
        img = self.cv_bridge.imgmsg_to_cv2(data, 'rgb8')
        self.cur_img_back = cv2.cvtColor(img, cv2.COLOR_BGR2RGB) 
        # print("rear", self.cur_img_back.dtype) 
        self.is_back = True
        
    def callback_pv_dis(self, data):
        if not self.is_pv_dis:
            self.pv_dis = data.data
            self.is_pv_dis = True
            
    def callback_turn_dis(self, data):
        if not self.is_turn_dis:
            self.turn_dis = data.data
            self.is_trun_dis = True
    
    def callback_speed(self, data):
        if not self.is_cur_speed:
            self.cur_speed = data.data
            self.is_cur_speed = True
            
    def callback_steer(self, data):
        if not self.is_cur_steer:
            self.cur_steer = data.data
            self.is_cur_steer = True

    def callback_vehicle_pose(self, data):
        if not self.is_cur_pose:
            self.cur_pose = data.data
            self.is_cur_pose = True    
            if self.cur_pose[0] == None:
                self.is_cur_pose = False

    def callback_vehicle_angle(self, data):
        if not self.is_angle:
            self.cur_angle = data.data
            self.is_angle = True    
            if self.cur_angle[0] == None:
                self.is_angle = False

    """
    Image warping
    """
    
    def front(self, img):
        IMAGE_H, IMAGE_W, _ = img.shape

        #print(img.shape)
        #img = np.concatenate([np.zeros((400,250,3)).astype(np.uint8),img,np.zeros((400,250,3)).astype(np.uint8)],1)

        src = self.forward_src#np.float32([[249, 399], [549, 399], [289, 0], [509, 0]])
        dst = self.forward_dst#np.float32([[279, 399], [519, 399], [0, 0], [799, 0]])
        #src = np.float32([[210,115], [210,180], [150,120], [150,175]])
        #dst = np.float32([[210,115], [210,180], [150,115], [150,180]])
        M = cv2.getPerspectiveTransform(src, dst) # The transformation matrix
        Minv = cv2.getPerspectiveTransform(dst, src) # Inverse transformation

        IMAGE_H, IMAGE_W, _ = img.shape

        warped_img = cv2.warpPerspective(img, M, (IMAGE_W, IMAGE_H))#[:300] # Image warping
        output = warped_img[90:,:-10]
        return output#cv2.resize(warped_img[200:,100:-100], dsize=(800, 400),interpolation=cv2.INTER_LINEAR)#warped_img

    def rear(self, img):
        IMAGE_H, IMAGE_W, _ = img.shape
    
        #img = np.concatenate([np.zeros((400,250,3)).astype(np.uint8),img,np.zeros((400,250,3)).astype(np.uint8)],1)
        src = self.backward_src#np.float32([[249, 399], [549, 399], [289, 0], [509, 0]])
        dst = self.backward_dst#np.float32([[279, 399], [519, 399], [0, 0], [799, 0]])
        #src = np.float32([[210,115], [210,180], [150,120], [150,175]])
        #dst = np.float32([[210,115], [210,180], [150,115], [150,180]])
        M = cv2.getPerspectiveTransform(src, dst) # The transformation matrix
        Minv = cv2.getPerspectiveTransform(dst, src) # Inverse transformation
    
        IMAGE_H, IMAGE_W, _ = img.shape
    
        warped_img = cv2.warpPerspective(img, M, (IMAGE_W, IMAGE_H))#[:300] # Image warping
        output = warped_img[90:,:]
        output = cv2.rotate(output, cv2.ROTATE_180)
        return output#cv2.resize(warped_img[200:,100:-100], dsize=(800, 400),interpolation=cv2.INTER_LINEAR)#warped_img
               
    
    """
    hsv
    """
    def hsv_parking(self, img, color='yellow'):
        hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

        if color == 'green':
            mask = cv2.inRange(hsv, (30, 90, 80), (80, 255, 255))
            imask = mask > 0
            output = np.zeros_like(hsv, np.uint8)
            output[imask] = 255
            output = cv2.cvtColor(output, cv2.COLOR_RGB2GRAY)
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (4, 4))
            output = cv2.morphologyEx(output, cv2.MORPH_OPEN, kernel)
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (15, 15))

            output = cv2.morphologyEx(output, cv2.MORPH_DILATE, kernel)
            # output = cv2.cvtColor(output, cv2.COLOR_GRAY2BGR)
            return output
        
        elif color == 'red':
            mask = cv2.inRange(hsv, (110, 100, 100), (150, 255, 255))
            imask = mask > 0
            output = np.zeros_like(hsv, np.uint8)
            output[imask] = 255
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
            output = cv2.morphologyEx(output, cv2.MORPH_OPEN, kernel)

            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (15, 15))
            output = cv2.morphologyEx(output, cv2.MORPH_DILATE, kernel)
            # kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (20, 20))
            # output = cv2.morphologyEx(output, cv2.MORPH_OPEN, kernel)
            # output = cv2.cvtColor(output, cv2.COLOR_GRAY2BGR)
            return output

        elif color == 'blue':
            mask = cv2.inRange(hsv, (0, 150, 100), (20, 255, 255))
            imask = mask > 0
            output = np.zeros_like(hsv, np.uint8)
            output[imask] = 255
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
            output = cv2.morphologyEx(output, cv2.MORPH_OPEN, kernel)

            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (15, 15))
            output = cv2.morphologyEx(output, cv2.MORPH_DILATE, kernel)
            # kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (20, 20))
            # output = cv2.morphologyEx(output, cv2.MORPH_OPEN, kernel)
            # output = cv2.cvtColor(output, cv2.COLOR_GRAY2BGR)
            return output

        elif color == 'yellow':
            mask = cv2.inRange(hsv, (80, 40, 145), (150, 255, 255))
            imask = mask > 0
            temp = np.zeros_like(hsv, np.uint8)
            temp[imask] = 255
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (10,10))
            clean = cv2.morphologyEx(temp[:,:,0], cv2.MORPH_OPEN, kernel)
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15,15))
            output = cv2.morphologyEx(clean, cv2.MORPH_CLOSE, kernel)
            return output

        elif color == 'purple':
            mask = cv2.inRange(hsv, (130, 170, 130), (180, 255, 150))
            imask = mask > 0
            output = np.zeros_like(hsv, np.uint8)
            output[imask] = 255
            # mask = cv2.inRange(hsv, (80, 100, 145), (150, 255, 255))
            return output

    """
    calibration mapping
    """
    def calibration_map(self):
        img_parkinglot = self.img_parkinglot
        # print(img_parkinglot.shape)
        # cv2.imwrite('./dddd.png', img_parkinglot)
        img_parkinglot_hsv = self.hsv_parking(img_parkinglot, 'green')
        img_parkinglot_hsv_2 = cv2.cvtColor(img_parkinglot_hsv, cv2.COLOR_GRAY2BGR)
        cv2.imwrite(self.save_path + '/green_point.png',img_parkinglot_hsv_2)
        
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
        cv2.imwrite(self.save_path + '/parkinglot.png',img_parkinglot)
        img_red = self.hsv_parking(img_parkinglot, 'red')
        img_blue = self.hsv_parking(img_parkinglot, 'blue')

        cv2.imwrite(self.save_path + '/img_red.png', img_red)
        cv2.imwrite(self.save_path + '/img_blue.png', img_blue)

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
        
        return output
    
    def get_roi(self, target):
        cX = target["vehicle_center"][0]
        cY = target["vehicle_center"][1]
        agl_glo = target["angle"]
        agl = (180 - agl_glo) # turn angle
        # agl = agl_glo
        # print(agl
        
        img_parking_path = self.img_parking_path
        # img_parking_path = self.img_parkinglot

        cv2.imwrite(self.save_path + '/img_parking_path_ori.png', img_parking_path)

        img_parking_path = self.hsv_parking(img_parking_path, 'purple')
        img_parking_path = cv2.cvtColor(img_parking_path, cv2.COLOR_BGR2GRAY)
        # cv2.imwrite('/home/hellobye/catkin_ws/src/caffeine/src/images/img_parking_path.png', img_parking_path)

        # print(img_parking_path.shape)
        # img_parking_path = self.img_parkinglot
        dx = self.map_W/2 - cX
        dy = self.map_H/2 - cY
        # print(cX, cY)
        # print(dx, dy)
        mtrx = np.float32([[1, 0, dx],
                           [0, 1, dy]])
        
        img_trans = cv2.warpAffine(img_parking_path, mtrx, (self.map_W, self.map_H)) 

        # cv2.imwrite('./img_trans.png', img_trans)
        M = cv2.getRotationMatrix2D((self.map_W/2, self.map_H/2), agl, 1.0)
        rotate_img = cv2.warpAffine(img_trans, M, (self.map_W, self.map_H))
        
        roi_region = 400
        # print(cX, cY)
        cX_img = self.map_W/2
        cY_img = self.map_H/2
        img_trans_ori = cv2.warpAffine(self.img_parkinglot, mtrx, (self.map_W, self.map_H))   
        rotate_img_ori = cv2.warpAffine(img_trans_ori, M, (self.map_W, self.map_H))
        rotate_img_save = cv2.line(rotate_img_ori, (cX_img,cY_img), (cX_img,cY_img), (255,255,0), 3)
        # cv2.imwrite(self.save_path + '/img_rotate.png', rotate_img_save)
        cv2.imwrite(self.save_path + '/rotate/' + str(self.iter).zfill(4) + '.png', rotate_img_save)
        cv2.imshow('d', rotate_img_save)
        # cv2.imshow('ddd', img_trans_ori)

        rotate_img_save = cv2.line(rotate_img, (cX_img,cY_img), (cX_img,cY_img), (255,255,0), 3)
        # cv2.imshow('dd',rotate_img)


        roi_x_under = int(cX_img-roi_region)
        roi_x_upper = int(cX_img)
        roi_y_under = int(cY_img-roi_region/2)
        roi_y_upper = int(cY_img+roi_region/2)
        # print(roi_x_under, roi_x_upper, roi_y_under, roi_y_upper)
        if roi_x_under < 0:
            roi_x_under = 0
        if roi_y_under < 0:
            roi_y_under = 0
        # print(roi_x_under, roi_x_upper)
        # print(roi_y_under, roi_y_upper)
        # cv2.imwrite('/home/hellobye/catkin_ws/src/caffeine/src/images/img_rotate.png', rotate_img)
        # roi = rotate_img[roi_y_under:roi_y_upper, roi_x_under:roi_x_upper]
        roi = rotate_img
        roi = cv2.rotate(roi, cv2.ROTATE_90_CLOCKWISE)
        roi = roi[:roi.shape[0]/2,:]
        # roi = rotate_img[roi_x_under:roi_x_upper, roi_y_under:roi_y_upper]

        # cv2.imwrite('/home/hellobye/catkin_ws/src/caffeine/src/images/img_parking_path.png', img_parking_path)
        # cv2.imwrite('/home/hellobye/catkin_ws/src/caffeine/src/images/roi.png',roi)
        # roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        self.img_roi = roi
        return roi
    
    def get_steer(self):
        # try:
        # self.calibration_map()
            # try:
        target = self.find_property()
        roi = self.get_roi(target)
    
        gain_cte = 0.3
        gain_curv = -1
        cte = 0
        # look_a_head = roi.shape[0]/2 * 0.9
        look_a_head = roi.shape[0] * 0.3
        ref = roi.shape[1]/2
        
        path_idx = np.nonzero(roi)
        path_fit = np.polyfit(path_idx[0], path_idx[1], 2)
        # path_fit = np.polyfit(path_idx[0], path_idx[1], 1)

        # print(path_fit)
        # print(path_fitx)
        # look_a_head = 70

        # print(path_fit)
        ploty = np.linspace(0, roi.shape[0]-1, roi.shape[0]) # y value
        path_fitx = path_fit[0]*ploty**2+path_fit[1]*ploty+path_fit[2]
        # path_fitx = path_fit[0]*ploty+path_fit[1]

        roi = cv2.cvtColor(roi, cv2.COLOR_GRAY2BGR)
        for i in range(len(ploty)):
            roi = cv2.line(roi, (int(round(path_fitx[i])), int(round(ploty[i]))), (int(round(path_fitx[i])), int(round(ploty[i]))), (255,0,0), 2)
        roi = cv2.line(roi, (int(round(roi.shape[1]/2)), int(round(look_a_head))), (int(round(roi.shape[1]/2)), int(round(look_a_head))), (0,255,0), 10)
        cv2.imwrite(self.save_path + '/roi/' + str(self.iter).zfill(4) + '.png', roi)


        front_lane = path_fit[0]*look_a_head**2 + path_fit[1]*look_a_head + path_fit[2]
        # front_lane = path_fit[0]*look_a_head + path_fit[1]

        roi = cv2.line(roi, (int(round(front_lane)), int(round(look_a_head))), (int(round(front_lane)), int(round(look_a_head))), (0,255,120), 10)

        cv2.imshow('dddd', roi)
        front_curverad = ((1 + (2*path_fit[0]*look_a_head + path_fit[1])**2)**1.5) / (2*path_fit[0]) * self.m_per_pixel
        cte = ref - front_lane
        steer = gain_cte * cte + gain_curv / front_curverad
        steer = max(min(steer, 20.0), -20.0)
        self.steer = steer
        self.cte = cte
        #     except:
        #         target = [None, None]
        #         self.pub_vehicle_center.publish(target)
        # except:
        #     print('calibration error')
            
    
    
    def get_speed(self):
        try:
            if self.is_turn_dis and self.turn_dis != -1:
                if self.turn_dis < 80:
                    self.speed = -1
                    print('speed change_turn')

            elif self.is_pv_dis and self.pv_dis < 80 and self.pv_dis > 20:
                self.speed = 0
                self.steer = 0
                print('speed change_pv')
            self.is_turn_dis = False
            self.is_pv_dis = False

        except:
            self.speed = 150
            self.is_turn_dis = False
            self.is_pv_dis = False
        

    def parking_process(self):
        # if self.is_coord_x == True and self.is_coord_y == True and self.is_coord_ang == True:
        #     for i  in range(self.coord_x.shape[0] - 1):
        #         dx = self.coord_x[i+1] - self.coord_x[i]
        #         dy = self.coord_y[i+1] - self.coord_y[i]
        #         # ax =
        # if self.is_accel_x and self.is_accel_y and self.is_agl and self.is_img:
        # if self.is_agl and self.agl_cal < 10:

        #     self.is_agl = False
        #     self.agl_init = self.agl_init + self.agl.data
        #     self.agl_cal = self.agl_cal + 1
        #     # print(self.agl_init)
        #     if self.agl_cal == 10:
        #         self.agl_init /= 10
        #         self.agl_cal = 1000
        #         print('angle calibaration')
            
            
        # elif self.is_agl and self.agl_cal == 1000:
        if self.iter == 0:
            self.is_parking_path = False
            self.is_parkinglot = False
            self.iter = self.iter+1

        if self.is_parking_path and self.is_parkinglot:
            self.start_time = time.time()
            
            agl = self.agl.data - self.agl_init
            # self.agl_init = self.agl.data
            # print(agl)                
            self.get_steer()
            self.get_speed()
            # print(self.steer)
            
            print("-----------------------")
            print('goal distance: {:.3}'.format(self.pv_dis))
            print('turn point distance: {:.3}'.format(self.turn_dis))
            print('speed: {:.3}'.format(self.cur_speed))
            print('steer: {:.3}'.format(self.cur_steer))
            print('control state: parking')
            print("-----------------------")
            
            
            cv2.imshow('roi', self.img_roi)
            # cv2.imshow('red', self.img_red)
            # cv2.imshow('blue', self.img_blue)

            cv2.imshow('map', self.img_map)
            cv2.waitKey(1)
            cv2.imwrite(self.save_path + '/path_w_car/' + str(self.iter).zfill(4) + '.png', self.img_map)

        
            # if  self.iter == 0:
            #     with open(self.save_path + '/angle.csv', 'w') as f:
            #         wr = csv.writer(f)
            #         wr.writerow([agl])
            # else:    
            #     with open(self.save_path + '/angle.csv', 'a') as f:
            #         wr = csv.writer(f)
            #         wr.writerow([agl])
                
            # if  self.iter == 0:
            #     with open(self.save_path + '/cte.csv', 'w') as f:
            #         wr = csv.writer(f)
            #         wr.writerow([self.cte])
            # else:    
            #     with open(self.save_path + '/cte.csv', 'a') as f:
            #         wr = csv.writer(f)
            #         wr.writerow([self.cte])

            # print(self.iter)
            self.iter = self.iter+1
            self.is_agl = False                

        else:
            print('wait for all receiving')
            
        if self.steer is not None:
            self.pub_ctrl_servo.publish(self.steer)


        if self.speed is not None:
            self.pub_ctrl_motor.publish(self.speed)
            
        self.is_cur_speed = False
        self.is_cur_steer = False
        # dt = time.time()-self.start_time
        # print('time :', dt)

        # if  self.iter == 0:
        #     with open(self.save_path + '/time.csv', 'w') as f:
        #         wr = csv.writer(f)
        #         wr.writerow([dt])
        # else:    
        #     with open(self.save_path + '/time.csv', 'a') as f:
        #         wr = csv.writer(f)
        #         wr.writerow([dt])

     
        
if __name__ == '__main__':
    save_path = '/home/hellobye/exp6'
    if not os.path.exists(save_path):
        os.makedirs(save_path)
        os.makedirs(save_path+'/roi')
        os.makedirs(save_path+'/path_w_car')
        
    rospy.init_node('ros_parking')
    r = rospy.Rate(10)
    pc = Parking(save_path)

    
    while not rospy.is_shutdown():        
        pc.parking_process()
        r.sleep()

    rospy.spin()
        
        
    
    
