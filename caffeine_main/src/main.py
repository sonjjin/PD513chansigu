#!/usr/bin/env python
#-*- coding: utf-8 -*-

from ramp_tracker import Ramptracker
from parking_final_2 import Parking
from final_stop import Finalstop
import rospy
import os



if __name__ == '__main__':
    save_path = '/home/hellobye/exp6'
    if not os.path.exists(save_path):
        os.makedirs(save_path)
        os.makedirs(save_path+'/roi')
        os.makedirs(save_path+'/path_w_car')
    rospy.init_node('main')
    r = rospy.Rate(20)
    rt = Ramptracker(start_position = 0) # 1: look right, 0: look left, 2: both lane
    pc = Parking(save_path)
    fs = Finalstop(save_path)
    control_state = 0
    
    while not rospy.is_shutdown():   
        if control_state == 0 or control_state == 1:     
            control_state = rt.process()
            # print(control_state)
            r.sleep()
            
        elif control_state == 2 or control_state == 3:
            # print(control_state)
            conrol_state = pc.process()
            r.sleep()

        elif control_state == 4:
            control_state = fs.process()
            r.sleep()

    rospy.spin()
        