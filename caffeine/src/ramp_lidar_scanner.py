#!/usr/bin/env python
#-*- coding: utf-8 -*-

import numpy as np
import math
import cv2
import matplotlib.pyplot as plt

import rospy
from sensor_msgs.msg import LaserScan

class ramp_lidar_scanner:
    def __init__(self):
        self.laserscan_sub = rospy.Subscriber('/scan', LaserScan, self.laserscan_callback)
        
        self.is_laserscan = False
        self.laserscan = None

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
            points_xy = self.convert_polar2xy(self.laserscan)
            plt.scatter(points_xy[:, 0], points_xy[:, 1])
            plt.show()




if __name__ == '__main__':
    rospy.init_node('ramp_lidar_scanner')
    r = rospy.Rate(10)
    rls = ramp_lidar_scanner()

    while not rospy.is_shutdown():        
        rls.process()
        r.sleep()
    
    rospy.spin()