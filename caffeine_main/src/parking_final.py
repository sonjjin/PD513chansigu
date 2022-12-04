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
        
        self.sub_parking_path1 = rospy.Subscriber('/img_w_path1', Image, self.callback_img_path) # MATLAB
        self.sub_parkinglot = rospy.Subscriber('/camera/image_raw', Image, self.callback_img_parking) 
        
        self.sub_proprties = rospy.Subscriber('/properties', Float32MultiArray, self.callback_properties)

        self.sub_pv_distance = rospy.Subscriber('/vehicle_dis', Float32, self.callback_pv_dis)
        self.sub_turn_distance = rospy.Subscriber('/turn_dis', Float32, self.callback_turn_dis)
        

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
        
        self.img_parkinglot = None
        self.is_parkinglot = False

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
        
        
        self.img_path = None
        self.img_roi = None
        self.iter = 0
        self.cte = 0
        self.start_time = time.time()


    """
    callback functions
    """
    def callback_img_parking(self, data):
        if not self.is_parkinglot:
            img = self.cv_bridge.imgmsg_to_cv2(data, 'rgb8') # ros image를 cv2로 받아오기
            self.img_parkinglot = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            # print("front",  self.cur_img_front.dtype) 
            self.is_parkinglot = True
    
    def callback_img_path(self, data):
        if not self.is_parking_path1:
            img = self.cv_bridge.imgmsg_to_cv2(data, 'rgb8') # ros image를 cv2로 받아오기
            self.img_parking_path1 = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            # print("front",  self.cur_img_front.dtype) 
            self.is_parking_path1 = True
        
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
        # agl = agl_glo
        # print(agl
        
        img_parking_path1 = self.img_parking_path1
        # img_parking_path = self.img_parkinglot

        cv2.imwrite(self.save_path + '/img_parking_path_ori.png', img_parking_path1)

        img_parking_path1 = hsv_parking(img_parking_path1, 'purple')
        img_parking_path1 = cv2.cvtColor(img_parking_path1, cv2.COLOR_BGR2GRAY)
        # cv2.imwrite('/home/hellobye/catkin_ws/src/caffeine/src/images/img_parking_path.png', img_parking_path)

        # print(img_parking_path.shape)
        # img_parking_path = self.img_parkinglot
        dx = self.map_W/2 - cX
        dy = self.map_H/2 - cY
        # print(cX, cY)
        # print(dx, dy)
        mtrx = np.float32([[1, 0, dx],
                           [0, 1, dy]])
        
        img_trans_path1 = cv2.warpAffine(img_parking_path1, mtrx, (self.map_W, self.map_H)) 

        # cv2.imwrite('./img_trans.png', img_trans)
        M = cv2.getRotationMatrix2D((self.map_W/2, self.map_H/2), agl, 1.0)
        rotate_img = cv2.warpAffine(img_trans_path1, M, (self.map_W, self.map_H))
        
        roi_region = 400
        # print(cX, cY)
        cX_img = self.map_W/2
        cY_img = self.map_H/2
        # img_trans_ori = cv2.warpAffine(self.img_parkinglot, mtrx, (self.map_W, self.map_H))   
        # rotate_img_ori = cv2.warpAffine(img_trans_ori, M, (self.map_W, self.map_H))
        # rotate_img_save = cv2.line(rotate_img_ori, (cX_img,cY_img), (cX_img,cY_img), (255,255,0), 3)
        # # cv2.imwrite(self.save_path + '/img_rotate.png', rotate_img_save)
        # cv2.imwrite(self.save_path + '/rotate/' + str(self.iter).zfill(4) + '.png', rotate_img_save)
        # cv2.imshow('d', rotate_img_save)
        # # cv2.imshow('ddd', img_trans_ori)

        # rotate_img_save = cv2.line(rotate_img, (cX_img,cY_img), (cX_img,cY_img), (255,255,0), 3)
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
        if self.is_properties:
            target = self.properties
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
            self.is_properties = False
        else:
            print('none properties')
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
        
    def process(self):
        if self.iter == 0:
            self.is_parking_path1 = False
            self.iter = self.iter+1

        else:
            if self.is_parking_path1:
                self.start_time = time.time()
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

                # cv2.imshow('map', self.img_map)
                cv2.waitKey(1)
                # cv2.imwrite(self.save_path + '/path_w_car/' + str(self.iter).zfill(4) + '.png', self.img_map)

            
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
        pc.process()
        r.sleep()

    rospy.spin()
        
        
    
    
