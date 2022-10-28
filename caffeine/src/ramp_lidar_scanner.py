#!/usr/bin/env python
#-*- coding: utf-8 -*-

import numpy as np
import math
import cv2
import matplotlib.pyplot as plt
import csv
import rospy
from sensor_msgs.msg import LaserScan

class ramp_lidar_scanner:
    def __init__(self):
        self.laserscan_sub = rospy.Subscriber('/scan', LaserScan, self.laserscan_callback)
        
        self.is_laserscan = False
        self.laserscan = None
        self.angle = []
        self.ranges = []

    def laserscan_callback(self, data):
        self.laserscan = data
        self.is_laserscan = True

    def convert_polar2xy(self, laserscan):
        ranges_xy = np.zeros((len(laserscan.ranges), 2))
        angle = laserscan.angle_min
        for i in range(len(laserscan.ranges)):
            if not math.isinf(laserscan.ranges[i]):
                ranges_xy[i, 0] = laserscan.ranges[i] * math.cos(angle)
                ranges_xy[i, 1] = laserscan.ranges[i] * math.sin(angle)
            angle += laserscan.angle_increment
        return ranges_xy

    def process(self):
        if self.is_laserscan:
            # points_xy = self.convert_polar2xy(self.laserscan)
            with open('/home/juntae/catkin_ws/src/caffeine/src/laser.csv','a') as f:
                writer = csv.writer(f)
                writer.writerow(self.laserscan.ranges)
                # print('a')

            # points_xy = points_xy*100
            # points_xy = np.array(points_xy, dtype=np.int32)
            # # points_xy.astype(np.int64)
            # print(np.max(points_xy[:,0]), np.max(points_xy[:,1]))
            # print(np.min(points_xy[:,0]), np.min(points_xy[:,1]))
            # points_xy = points_xy + 700


            # # img_lidar = np.zeros((np.max(points_xy[:,0]+100), np.max(points_xy[:,1]+100)), dtype = np.uint8)
            # # img_lidar[points_xy[:,0]+50, points_xy[:,1]+50] = 255
            # cv2.imshow('lidar image', cv2.resize(cv2.rotate(img_lidar, cv2.ROTATE_180), dsize = (300,500)))
            # cv2.waitKey(1)




if __name__ == '__main__':
    rospy.init_node('ramp_lidar_scanner')
    r = rospy.Rate(10)
    rls = ramp_lidar_scanner()

    while not rospy.is_shutdown():        
        rls.process()
        r.sleep()
    
    rospy.spin()