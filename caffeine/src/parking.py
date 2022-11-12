#!/usr/bin/env python
#-*- coding: utf-8 -*-

import numpy as np
import cv2
import math as m
from cv_bridge import CvBridge
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
        
        self.sub_img = rospy.Subscriber('/img_w_path', Image, self.img_callback)
        self.sub_coord_x = rospy.Subscriber('/coord_x', Float32MultiArray, self.coord_x_callback) # cm
        self.sub_coord_y = rospy.Subscriber('/coord_y', Float32MultiArray, self.coord_y_callback) # cm
        self.sub_coord_ang = rospy.Subscriber('/coord_ang', Float32MultiArray, self.coord_ang_callback)
        self.sub_accX = rospy.Subscriber('/arduino_imu/accX', Float32, self.accel_x_callback) # cm/s
        self.sub_accY = rospy.Subscriber('/arduino_imu/accY', Float32, self.accel_y_callback) # cm/s
        self.sub_aglZ = rospy.Subscriber('/arduino_imu/agl', Float32, self.agl_callback) # cm/s
        
        self.pub_ctrl_motor = rospy.Publisher('ctrl_motor', Float32, queue_size=1)
        self.pub_ctrl_servo = rospy.Publisher('ctrl_servo', Float32, queue_size=1)
        
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
        
        self.cur_img = None
        self.is_img = False


    # callback functions
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
    
    def img_callback(self, data):
        if not self.is_img:
            img = self.cv_bridge.imgmsg_to_cv2(data, 'rgb8') # ros image를 cv2로 받아오기
            self.cur_img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            # print("front",  self.cur_img_front.dtype) 
            self.is_img = True

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



    def main(self):
        # if self.is_coord_x == True and self.is_coord_y == True and self.is_coord_ang == True:
        #     for i  in range(self.coord_x.shape[0] - 1):
        #         dx = self.coord_x[i+1] - self.coord_x[i]
        #         dy = self.coord_y[i+1] - self.coord_y[i]
        #         # ax =
        dt = 0.1
        pos_x = 300
        pos_y = 360 
        if self.is_accel_x and self.is_accel_y and self.is_agl and self.is_img:
            accX = self.accel_x
            accY = self.accel_y
            pos_x = pos_x + round(accX*dt*dt)
            pos_y = pos_y + round(accY*dt*dt)
            # img = self.cur_img
            img = np.zeros_like((500,500))
            img = cv2.line(img, (pos_y, pos_x), (pos_y, pos_x), (0,0,255), 3)
            acc = [accX, accY]
            with open('./accel.csv','a') as f:
                writer = csv.writer(f)
                writer.writerow(acc)
            # print(img.shape)
            # print(type(img))
            cv2.imshow('path', cv2.resize(img))
            cv2.waitKey(1)
            self.is_img = False
            self.is_accle_x = False
            self.is_accle_y = False
            self.is_agl = False
            
        else:
            print('wait for all receiving')
     
        
if __name__ == '__main__':
    rospy.init_node('ros_parking')
    r = rospy.Rate(10)
    pc = parking()

    while not rospy.is_shutdown():        
        pc.main()
        r.sleep()

    rospy.spin()
        
        
    
    
