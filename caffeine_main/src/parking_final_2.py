#!/usr/bin/env python
#-*- coding: utf-8 -*-


import numpy as np
import cv2
import math as m
from cv_bridge import CvBridge
from utils import *
import time

import os

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray

class Parking:
    def __init__(self, save_path):
        self.cv_bridge = CvBridge()
        
        self.sub_parking_path1 = rospy.Subscriber('/img_w_path1', Image, self.callback_img_path1) # MATLAB
        self.sub_parking_path2 = rospy.Subscriber('/img_w_path2', Image, self.callback_img_path2) # MATLAB
        self.sub_parking_path3 = rospy.Subscriber('/img_w_path3', Image, self.callback_img_path3) # MATLAB
        
        self.sub_proprties = rospy.Subscriber('/properties', Float32MultiArray, self.callback_properties)
        self.sub_pv_distance = rospy.Subscriber('/vehicle_dis', Float32, self.callback_pv_dis)
        self.sub_turn_distance = rospy.Subscriber('/turn_dis', Float32MultiArray, self.callback_turn_dis)
    
        self.sub_cur_speed = rospy.Subscriber('/arduino_ctrl/ctrl_motor', Float32, self.callback_speed)
        self.sub_cur_steer = rospy.Subscriber('/arduino_ctrl/ctrl_servo', Float32, self.callback_steer)
        
        self.pub_ctrl_motor = rospy.Publisher('/arduino_ctrl/ctrl_motor', Float32, queue_size=1)
        self.pub_ctrl_servo = rospy.Publisher('/arduino_ctrl/ctrl_servo', Float32, queue_size=1)
        
        self.save_path = save_path
        self.local_coord = np.zeros([465, 443])
        
        # properties
        self.pv_dis = None
        self.is_pv_dis = False
        
        self.turn_dis = None
        self.is_turn_dis = False

        self.properties = None
        self.is_properties = False
        
        # imgs
        self.img_parking_path1 = None
        self.is_parking_path1 = False
        
        self.img_parking_path2 = None
        self.is_parking_path2 = False
        
        self.img_parking_path3 = None
        self.is_parking_path3 = False
        
        # image
        self.img_map = None
        self.map_W = 465
        self.map_H = 443
        self.m_per_pixel = 0.00252
        
        # control
        self.steer = None
        self.speed = None
        
        self.cur_speed = None
        self.is_cur_speed = False
        self.cur_steer = None
        self.is_cur_steer = False
        
        self.img_path = None
        self.img_roi = None
        self.iter = 0
        self.cte = 0
        self.start_time = time.time()
        
        self.control_state = 1 # 1: path1, 2: path2, 3: path3
        self.parking_finish = False


    """
    callback functions
    """
    def callback_img_path1(self, data):
        if not self.is_parking_path1:
            img = self.cv_bridge.imgmsg_to_cv2(data, 'rgb8') # ros image를 cv2로 받아오기
            self.img_parking_path1 = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            # print("front",  self.cur_img_front.dtype) 
            self.is_parking_path1 = True
        
    def callback_img_path2(self, data):
        if not self.is_parking_path1:
            img = self.cv_bridge.imgmsg_to_cv2(data, 'rgb8') # ros image를 cv2로 받아오기
            self.img_parking_path2 = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            # print("front",  self.cur_img_front.dtype) 
            self.is_parking_path2 = True
    
    def callback_img_path3(self, data):
        if not self.is_parking_path3:
            img = self.cv_bridge.imgmsg_to_cv2(data, 'rgb8') # ros image를 cv2로 받아오기
            self.img_parking_path3 = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            # print("front",  self.cur_img_front.dtype) 
            self.is_parking_path3 = True
    
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

    def callback_properties(self, data):
        if not self.is_properties:
            self.properties = data.data
            self.is_properties = True
    
    def get_roi(self, target):
        cX = target[0]
        cY = target[1]
        agl_glo = target[2]
        agl = (180 - agl_glo) # turn angle
        if self.control_state == 1:
            img_parking_path = self.img_parking_path1

        if self.control_state == 2:
            img_parking_path = self.img_parking_path2
            
        if self.control_state == 3:
            img_parking_path = self.img_parking_path3

        dx = self.map_W/2 - cX
        dy = self.map_H/2 - cY

        mtrx = np.float32([[1, 0, dx],
                           [0, 1, dy]])
        
        img_trans_path = cv2.warpAffine(img_parking_path, mtrx, (self.map_W, self.map_H)) 

        M = cv2.getRotationMatrix2D((self.map_W/2, self.map_H/2), agl, 1.0)
        rotate_img = cv2.warpAffine(img_trans_path, M, (self.map_W, self.map_H))

        roi = rotate_img
        roi = cv2.rotate(roi, cv2.ROTATE_90_CLOCKWISE)
        roi = roi[:roi.shape[0]/2,:]

        return roi
    
    def get_steer(self):
        if self.is_properties:
            target = self.properties
            roi = self.get_roi(target)
        
            gain_cte = 0.3
            gain_curv = -1
            cte = 0

            look_a_head = roi.shape[0] * 0.3
            ref = roi.shape[1]/2
            
            path_idx = np.nonzero(roi)
            path_fit = np.polyfit(path_idx[0], path_idx[1], 2)

            ploty = np.linspace(0, roi.shape[0]-1, roi.shape[0]) # y value
            path_fitx = path_fit[0]*ploty**2+path_fit[1]*ploty+path_fit[2]


            roi = cv2.cvtColor(roi, cv2.COLOR_GRAY2BGR)
            for i in range(len(ploty)):
                roi = cv2.line(roi, (int(round(path_fitx[i])), int(round(ploty[i]))), (int(round(path_fitx[i])), int(round(ploty[i]))), (255,0,0), 2)
            roi = cv2.line(roi, (int(round(roi.shape[1]/2)), int(round(look_a_head))), (int(round(roi.shape[1]/2)), int(round(look_a_head))), (0,255,0), 10)
            cv2.imwrite(self.save_path + '/roi/' + str(self.iter).zfill(4) + '.png', roi)

            front_lane = path_fit[0]*look_a_head**2 + path_fit[1]*look_a_head + path_fit[2]

            roi = cv2.line(roi, (int(round(front_lane)), int(round(look_a_head))), (int(round(front_lane)), int(round(look_a_head))), (0,255,120), 10)
            self.img_roi = roi
            front_curverad = ((1 + (2*path_fit[0]*look_a_head + path_fit[1])**2)**1.5) / (2*path_fit[0]) * self.m_per_pixel
            cte = ref - front_lane
            steer = gain_cte * cte + gain_curv / front_curverad
            steer = max(min(steer, 20.0), -20.0)
            self.steer = steer
            self.cte = cte
            self.is_properties = False
        else:
            print('none properties')

            
    def get_speed(self):
        try:
            if self.is_turn_dis:
                if self.control_state == 1 and self.speed > 1 and self.turn_dis[0] != -1:
                    if self.turn_dis < 80:
                        self.speed = 0
                        self.control_state += 1

                elif self.control_state == 2 and self.turn_dis[0] < 80 and self.turn_dis[0] != -1:
                    self.speed = -100
                    if self.turn_dis[1] < 80:
                        self.control_state += 1
                        self.speed = 0
                
                elif self.control_state == 3 and self.turn_dis[1] < 80 and self.turn_dis[1] != -1:
                    self.speed = 150
                    
            elif self.is_pv_dis and self.pv_dis < 80 and self.pv_dis > 20:
                self.speed = 0
                self.steer = 0
                self.parking_finish = True
            self.is_turn_dis = False
            self.is_pv_dis = False

        except:
            self.speed = 150
            self.is_turn_dis = False
            self.is_pv_dis = False
        
    def process(self):
        if self.iter == 0:
            self.is_parking_path1 = False
            self.is_parking_path2 = False
            self.is_parking_path3 = False
            self.iter = self.iter+1

        else:
            if self.is_parking_path1 and self.is_parking_path2 and self.is_parking_path3:
                self.start_time = time.time()
          
                self.get_steer()
                self.get_speed()

                print("-----------------------")
                print('goal distance: {:.3}'.format(self.pv_dis))
                print('turn point1 distance: {:.3}'.format(self.turn_dis[0]))
                print('turn point1 distance: {:.3}'.format(self.turn_dis[1]))
                print('speed: {:.3}'.format(self.cur_speed))
                print('steer: {:.3}'.format(self.cur_steer))
                print('control state: parking')
                print("-----------------------")
                
                
                cv2.imshow('roi', self.img_roi)
                cv2.waitKey(1)
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


     
        
if __name__ == '__main__':
    save_path = '/home/hellobye/exp6'
    if not os.path.exists(save_path):
        os.makedirs(save_path)
        os.makedirs(save_path+'/roi')
        os.makedirs(save_path+'/path_w_car')
        
    rospy.init_node('ros_parking')
    r = rospy.Rate(20)
    pc = Parking(save_path)

    
    while not rospy.is_shutdown():        
        pc.process()
        r.sleep()

    rospy.spin()
        
        
    
    
