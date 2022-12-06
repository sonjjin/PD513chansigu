#!/usr/bin/env python
#-*- coding: utf-8 -*-


import numpy as np
import cv2
import math as m
from cv_bridge import CvBridge
from utils import hsv
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
        self.img_front_sub = rospy.Subscriber('/front_cam/image_raw', Image, self.img_front_callback)
        self.img_back_sub = rospy.Subscriber('/rear_cam/image_raw', Image, self.img_back_callback)
        
        self.sub_proprties = rospy.Subscriber('/properties', Float32MultiArray, self.callback_properties)
        self.sub_pv_distance = rospy.Subscriber('/vehicle_dis', Float32, self.callback_pv_dis)
        self.sub_turn_distance = rospy.Subscriber('/turn_dis', Float32MultiArray, self.callback_turn_dis)
        self.sub_turnpoint = rospy.Subscriber('/turnpoint', Float32MultiArray, self.callback_turnpoint)
    
        self.sub_cur_speed = rospy.Subscriber('/arduino_ctrl/ctrl_motor', Float32, self.callback_speed)
        self.sub_cur_steer = rospy.Subscriber('/arduino_ctrl/ctrl_servo', Float32, self.callback_steer)
        
        self.pub_ctrl_motor = rospy.Publisher('/arduino_ctrl/ctrl_motor', Float32, queue_size=1)
        self.pub_ctrl_servo = rospy.Publisher('/arduino_ctrl/ctrl_servo', Float32, queue_size=1)
        
        self.save_path = save_path
        self.local_coord = np.zeros([465, 443])
        self.control_state = 2
        
        # properties
        self.pv_dis = None
        self.is_pv_dis = False
        
        self.turn_dis = None
        self.is_turn_dis = False

        self.properties = None
        self.is_properties = False
        
        self.turnpoint = None
        self.is_turnpoint = False
        
        # imgs
        self.img_parking_path1 = None
        self.is_parking_path1 = False
        
        self.img_parking_path2 = None
        self.is_parking_path2 = False
        
        self.img_parking_path3 = None
        self.is_parking_path3 = False
        
        # svm images
        self.cur_img_front = None
        self.is_cur_img_front = False
        self.cur_img_rear = None
        self.is_cur_img_rear = False
        
        # image_
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
        self.control_count = 0
        self.start_time = time.time()
        
        self.control_map = 1 # 1: path1, 2: path2, 3: path3
        self.parking_finish = False


        self.forward_src = np.float32([
            (125, 180),
            (0, 440),
            (500, 180),
            (640, 440)
        ])

        self.backward_src = np.float32([
            (125, 180),
            (0, 440),
            (500, 180),
            (640, 440)
        ])

        self.forward_dst = np.float32([
            (70, 90),
            (170, 440),
            (530, 90),
            (470, 445)
        ])
        
        self.backward_dst = np.float32([
            (90, 90),
            (180, 440),
            (530, 85),
            (460, 445)
        ])    


    """
    callback functions
    """
    def callback_img_path1(self, data):
        if not self.is_parking_path1:
            img = self.cv_bridge.imgmsg_to_cv2(data, 'rgb8') # ros image를 cv2로 받아오기
            self.img_parking_path1 = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            # print("front",  self.img_parking_path1.shape) 
            self.is_parking_path1 = True
        
    def callback_img_path2(self, data):
        if not self.is_parking_path1:
            img = self.cv_bridge.imgmsg_to_cv2(data, 'rgb8') # ros image를 cv2로 받아오기
            self.img_parking_path2 = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            # print("front",  self.img_parking_path2.shape) 
            self.is_parking_path2 = True
    
    def callback_img_path3(self, data):
        if not self.is_parking_path3:
            img = self.cv_bridge.imgmsg_to_cv2(data, 'rgb8') # ros image를 cv2로 받아오기
            self.img_parking_path3 = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            # print("front",  self.img_parking_path3.shape) 
            self.is_parking_path3 = True
            
    def img_front_callback(self, data):
        if not self.is_cur_img_front:
            img = self.cv_bridge.imgmsg_to_cv2(data, 'rgb8') # ros image를 cv2로 받아오기
            self.cur_img_front = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            # print("front",  self.cur_img_front.dtype) 
            self.is_cur_img_front = True
    
    def img_back_callback(self, data):
        if not self.is_cur_img_rear:
            img = self.cv_bridge.imgmsg_to_cv2(data, 'rgb8')
            self.cur_img_rear = cv2.cvtColor(img, cv2.COLOR_BGR2RGB) 
            # print("rear", self.cur_img_back.dtype) 
            self.is_cur_img_rear = True
    
    def callback_pv_dis(self, data):
        if not self.is_pv_dis:
            self.pv_dis = data.data
            self.is_pv_dis = True
            
    def callback_turn_dis(self, data):
        if not self.is_turn_dis:
            self.turn_dis = data.data
            self.is_turn_dis = True
    
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
    
    def callback_turnpoint(self, data):
        if not self.is_turnpoint:
            self.turnpoint =  np.array(data.data).reshape([data.layout.dim[0].size, data.layout.dim[1].size])
            self.is_turnpoint = True
    
    def get_roi(self, target):
        cX = target[0]
        cY = target[1]
        agl_glo = target[2]
        agl = (180 - agl_glo) # turn angle

        if self.control_map == 1:
            img_parking_path = self.img_parking_path1

        if self.control_map == 2:
            img_parking_path = self.img_parking_path2
            agl = 180 + agl
            
        if self.control_map == 3:
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
    
    '''
    Stop lane detection 
    input: warp image
    '''
    
    def stop_lane(self, img):
        img_hsv = hsv(img, 'yellow')
        img_bin = img_hsv
        
        nwindows = 10
        margin = 20
        minpix = 50

        window_width = np.int64(img_bin.shape[1]//nwindows)

        histogram = np.sum(img_bin[:,:img_bin.shape[1]-np.int32(img_bin.shape[1]/2)], axis=1)
        his_max = np.max(histogram)
        
        if his_max < 10000:
            return

        top_basey = np.argmax(histogram[:])

        nonzero = img_bin.nonzero()
        nonzeroy = np.array(nonzero[1])
        nonzerox = np.array(nonzero[0])

        top_current = top_basey
        top_lane_inds = []

        for window in range(nwindows):

            win_x_low = img_bin.shape[1] - (window+1)*window_width
            win_x_high = img_bin.shape[1] - window*window_width

            win_top_low = top_current - margin
            win_top_high = top_current + margin 

            good_top_inds = ((nonzerox >= win_top_low) & (nonzerox < win_top_high) & (nonzeroy >= win_x_low) & (nonzeroy < win_x_high)).nonzero()[0]
            top_lane_inds.append(good_top_inds)


            if(len(good_top_inds) > minpix):
                top_current = np.int32(np.mean(nonzerox[good_top_inds]))
                

        top_lane_inds = np.concatenate(top_lane_inds)

        topx = nonzerox[top_lane_inds]
        topy = nonzeroy[top_lane_inds] 

        top_fit = np.polyfit(topy, topx, 1)

        ref_x = np.int32(img_bin.shape[1]/2)
        ref_y = np.int32(top_fit[0]*ref_x + top_fit[1])
        print(ref_y)
        if ref_y > 330:
            self.control_state = 3
            
    def get_steer(self):
        if self.is_properties:
            try:
                target = self.properties
                roi = self.get_roi(target)
            
                gain_cte = 0.3
                gain_curv = -1
                cte = 0

                look_a_head = roi.shape[0] * 0.3
                ref = roi.shape[1]/2
                # if self.control_state == 2:
                #     look_a_head = roi.shape[0] * 0.5
                #     ref = roi.shape[1]*0.5

                
                path_idx = np.nonzero(roi)
                path_fit = np.polyfit(path_idx[0], path_idx[1], 2)

                ploty = np.linspace(0, roi.shape[0]-1, roi.shape[0]) # y value
                path_fitx = path_fit[0]*ploty**2+path_fit[1]*ploty+path_fit[2]

                # print(roi.shape)
                # roi = cv2.cvtColor(roi, cv2.COLOR_GRAY2BGR)
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

                if self.control_map == 2:
                    gain_cte = 0.6
                    steer = gain_cte * cte + gain_curv / front_curverad
                    self.steer = -steer
                    steer = max(min(steer, 20.0), -20.0)
                self.cte = cte
                self.is_properties = False
            
            except:
                self.steer = self.cur_steer
                self.is_properties = False

        else:
            print('none properties')

            
    def get_speed(self):
        try:
            if self.is_turn_dis:
                if self.control_map == 1 and self.turn_dis[0] != -1:
                    if self.turn_dis[0] < 50:
                        self.speed = 0
                        self.control_map = self.control_map + 1
                        time.sleep(1)

                elif self.control_map == 2 and self.turn_dis[0] < 80 and self.turn_dis[0] != -1:
                    self.speed = -150
                    if self.turn_dis[1] < 80 and self.turn_dis[1] != -1:
                        self.control_map = self.control_map + 1
                        self.speed = 0
                
                elif self.control_map == 3 and self.turn_dis[1] < 80 and self.turn_dis[1] != -1:
                    self.speed = 150
                    
            elif self.is_pv_dis and self.pv_dis < 70 and self.pv_dis > 20:
                self.speed = 0
                self.steer = 0
                self.parking_finish = True
        except:
            self.speed = self.cur_speed

        
    def process(self):
        if self.iter == 0:
            self.is_parking_path1 = False
            self.is_parking_path2 = False
            self.is_parking_path3 = False
            self.iter = self.iter + 1
        
        else:
            if self.control_state == 2:
                if self.is_parking_path1:
                    self.get_steer()
                    self.get_speed()
                    try:
                        if self.turnpoint[0][2] // 10 == 2:
                            if self.is_cur_img_rear:
                                img = self.cur_img_rear
                        else:
                            if self.is_cur_img_front:
                                img = self.cur_img_front
                        self.stop_lane(img)
                    except:
                        print('no imgs')
                        
                    print("-----------------------")
                    print('goal distance: {:.3}'.format(self.pv_dis))
                    print('turn point1 distance: {:.3}'.format(self.turn_dis[0]))
                    print('turn point2 distance: {:.3}'.format(self.turn_dis[1]))
                    print('speed: {:.3}'.format(self.cur_speed))
                    print('steer: {:.3}'.format(self.cur_steer))
                    print('control map: {}'.format(self.control_map))
                    print('control state: parking')
                    print("-----------------------")
                    
                    cv2.imshow('roi', self.img_roi)
                    cv2.waitKey(1)          

                else:
                    print('wait for all receiving')
                    
                if self.steer is not None:
                    self.pub_ctrl_servo.publish(self.steer)

                if self.speed is not None:
                    self.pub_ctrl_motor.publish(self.speed)
                
                self.is_cur_speed = False
                self.is_cur_steer = False
                self.is_turn_dis = False
                self.is_pv_dis = False
                self.is_cur_img_front = False
                self.is_cur_img_rear = False
                self.iter = self.iter + 1
                            
            if self.control_map == 3:
                self.count = self.conut + 1
                
            if self.count > 10:
                self.control_state = 4
                
        return self.control_state



if __name__ == '__main__':
    save_path = '/home/hellobye/exp6'
    if not os.path.exists(save_path):
        os.makedirs(save_path)
        os.makedirs(save_path+'/roi')
        os.makedirs(save_path+'/path_w_car')
        
    rospy.init_node('ros_parking')
    r = rospy.Rate(15)
    pc = Parking(save_path)

    while not rospy.is_shutdown():        
        pc.process()
        r.sleep()

    rospy.spin()