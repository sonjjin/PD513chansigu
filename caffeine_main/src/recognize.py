#!/usr/bin/env python
#-*- coding: utf-8 -*-

from find_property import find_property
from map_calibration import map_calibaration
import os
import rospy

if __name__ == '__main__':
    save_path = '/home/hellobye/exp6'
    if not os.path.exists(save_path):
        os.makedirs(save_path)
        os.makedirs(save_path+'/parkinglot')
        
    rospy.init_node('recognition')
    r = rospy.Rate(20)
    fp = find_property(save_path)
    mc = map_calibaration()    
    while not rospy.is_shutdown():   
        mc.process()
        fp.process()
        r.sleep()

    rospy.spin()
        