#!/usr/bin/env python
#-*- coding: utf-8 -*-

import numpy as np
import math as m

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray

class find_distance():

    def __init__(self):
        self.sub_vehicle_pose = rospy.Subscriber('/properties', Float32MultiArray, self.callback_vehicle)
        self.sub_parking_point = rospy.Subscriber('/parking_point', Float32MultiArray, self.callback_parkinglot)
        self.sub_turnpoint = rospy.Subscriber('/turnpoint', Float32MultiArray, self.callback_turnpoint)
        self.sub_vehicle_spped = rospy.Subscriber('/arduino_ctrl/ctrl_motor', Float32, self.callback_vspeed)
        
        self.pub_vehivle_dis = rospy.Publisher('/vehicle_dis', Float32, queue_size=1)
        self.pub_turn_dis = rospy.Publisher('/turn_dis', Float32MultiArray, queue_size=1)
        
        
        self.vehicle_pose = []
        self.parkinglot_pose = []
        self.turnpoint = []
        self.vspeed = None
        
        self.is_vehicle_pose = False
        self.is_parkinglot_pose = False
        self.is_turnpoint = False
        self.is_vspeed = False
        self.pub_turndis = None
        self.pub_dis = None
        
    def callback_vehicle(self, data):
        if not self.is_vehicle_pose:
            self.vehicle_pose = data.data
            self.is_vehicle_pose = True    
            if self.vehicle_pose[0] == None:
                self.is_vehicle_pose = False
            
    def callback_parkinglot(self, data):
        if not self.is_parkinglot_pose:
            self.parkinglot_pose = data.data
            self.is_parkinglot_pose = True
            
    def callback_turnpoint(self, data):
        if not self.is_turnpoint:
            self.turnpoint =  np.array(data.data).reshape([data.layout.dim[0].size, data.layout.dim[1].size])
            self.is_turnpoint = True
    
    def callback_vspeed(self, data):
        if not self.is_vspeed:
            self.vspeed = data.data
            self.is_vspeed = True
            
    
    def distance(self):
        try:
            # print(self.turnpoint[0][2])
            # print(self.turnpoint[1][0])

            if self.is_vehicle_pose and self.is_parkinglot_pose:
                dis = m.sqrt((self.vehicle_pose[0] - self.parkinglot_pose[0])**2 + (self.vehicle_pose[1] - (465 - self.parkinglot_pose[1]))**2)
                self.pub_dis = dis
                self.is_vehicle_pose = False
                # self.is_parkinglot = False
            
            
            # print(self.turnpoint)
            if self.is_turnpoint:
                if self.turnpoint[0][2] != 14 and self.turnpoint[0][2] // 10 != 2:
                    turndis1 = -1
                    turndis2 = -1
                else:
                    turndis1 = m.sqrt((self.vehicle_pose[0] - self.turnpoint[0][0])**2 + ((self.vehicle_pose[1]) - (465 - self.turnpoint[1][0]))**2)
                    if self.turnpoint[0][2] == 14:
                        turndis2 = m.sqrt((self.vehicle_pose[0] - self.turnpoint[0][1])**2 + ((self.vehicle_pose[1]) - (465 - self.turnpoint[1][1]))**2)
                    else:
                        turndis2 = -1
                    

            turndis_pub = Float32MultiArray()
            turndis_pub.data = np.array([turndis1, turndis2])
            self.pub_turn_dis.publish(turndis_pub)
            
            
            self.pub_vehivle_dis.publish(self.pub_dis)
            self.pub_turn_dis.publish(turndis_pub)
            print('goal distance: {}'.format(self.pub_dis))
            print('turn point1 distance: {}'.format(turndis_pub.data[0]))
            print('turn point1 distance: {}'.format(turndis_pub.data[1]))

        except:
            print('wait')

if __name__ == '__main__':
    rospy.init_node('distance')
    r = rospy.Rate(10)
    d = find_distance()
    
    while not rospy.is_shutdown():
        d.distance()
        r.sleep()
        
    rospy.spin()