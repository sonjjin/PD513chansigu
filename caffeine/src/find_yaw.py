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
import time
from utils.Kalman import KalmanAngle
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray

class kalmanfilter:
    def __init__(self):
        self.cv_bridge = CvBridge()
        
        self.sub_accX = rospy.Subscriber('/arduino_imu/accX', Float32, self.accel_x_callback) # cm/s
        self.sub_accY = rospy.Subscriber('/arduino_imu/accY', Float32, self.accel_y_callback) # cm/s
        self.sub_gyroZ = rospy.Subscriber('/arduino_imu/gyZ', Float32, self.gyro_z_callback) # degree sum
     
        # imu
        self.accel_x = None
        self.accel_y = None
        self.gyro_z = None
        
        self.kalman = KalmanAngle()
        self.kalAngleZ = 0
        accZangle_init = (m.atan2(self.accel_y, self.accel_x) + m.pi)*180/m.pi
        self.kalman.setAngle(accZangle_init)
        
        self.is_accel_x = False
        self.is_accel_y = False
        self.is_gyro_z = False
        self.timer = time.time()
    """
    callback functions
    """    

    def accel_x_callback(self, data):
        if not self.is_accel_x:
            self.accel_x = data
            self.is_accel_x = True

    def accel_y_callback(self, data):
        if not self.is_accel_x:
            self.accel_y = data
            self.is_accel_y = True

    def gyro_z_callback(self, data):
        if not self.is_gyro_z:
            self.gyro_z = data
            self.is_gyro_z = True
        
    def kalman_filter(self):
        if self.is_accel_x and self.is_accel_y and self.is_gyro_z:
            accZangle = (m.atan2(self.accel_y, self.accel_x) + m.pi)*180/m.pi
            gyroZrate = self.gyro_z/131
            dt = time.time() - self.timer
            self.kalAngleZ = self.kalman.getAngle(accZangle, gyroZrate, dt)
            self.timer = time.time()
            print(self.kalAngleZ)
            
        else:
            print('wait for all receiving')
            

     
        
if __name__ == '__main__':
    rospy.init_node('kalman')
    r = rospy.Rate(10)
    pc = kalmanfilter()

    while not rospy.is_shutdown():        
        pc.kalman_filter()
        r.sleep()

    rospy.spin()
        
        
    
    
