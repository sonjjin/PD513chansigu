#!/usr/bin/env python
#-*- coding: utf-8 -*-

from ramp_tracker import Ramptracker
from parking import Parking
import rospy
import os



if __name__ == '__main__':
    save_path = '/home/hellobye/exp6'
    if not os.path.exists(save_path):
        os.makedirs(save_path)
        os.makedirs(save_path+'/roi')
        os.makedirs(save_path+'/path_w_car')
    rospy.init_node('main')
    r = rospy.Rate(10)
    rt = Ramptracker()
    pc = Parking(save_path)
    control_state = 0 # 0: ramp, 1: parking
    
    while not rospy.is_shutdown():   
        if control_state == 0:     
            control_state = rt.ramp_process()
        elif control_state == 1:
            pc.parking_process()
        r.sleep()
        
        
    
    rospy.spin()
        