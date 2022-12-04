#!/usr/bin/env python
#-*- coding: utf-8 -*-

from find_property import find_property
from map_calibration import map_calibaration
import rospy
import os



if __name__ == '__main__':
    rospy.init_node('recognition')
    r = rospy.Rate(20)
    fp = find_property()
    mc = map_calibaration()    
    while not rospy.is_shutdown():   
        fp.process()
        mc.process()
        r.sleep()

    rospy.spin()
        