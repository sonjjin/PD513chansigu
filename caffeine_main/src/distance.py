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
        self.sub_vehicle_pose = rospy.Subscriber('/vehicle_point', Float32MultiArray, self.callback_vehicle)
        self.sub_parking_point = rospy.Subscriber('/parking_point', Float32MultiArray, self.callback_parkinglot)
        self.sub_turnpoint = rospy.Subscriber('/turnpoint', Float32MultiArray, self.callback_turnpoint)
        self.sub_vehicle_spped = rospy.Subscriber('/arduino_ctrl/ctrl_motor', Float32, self.callback_vspeed)
        
        self.pub_vehivle_dis = rospy.Publisher('/vehicle_dis', Float32, queue_size=1)
        self.pub_turn_dis = rospy.Publisher('/turn_dis', Float32, queue_size=1)
        
        
        self.vehicle_pose = []
        self.parkinglot_pose = []
        self.turnpoint = []
        self.vspeed = None
        
        self.is_vehicle = False
        self.is_parkinglot = False
        self.is_turnpoint = False
        self.is_vspeed = False

        self.pub_turndis = None
        self.pub_dis = None
        
    def callback_vehicle(self, data):
        if not self.is_vehicle:
            self.vehicle_pose = data.data
            self.is_vehicle = True    
            if self.vehicle_pose[0] == None:
                self.is_vehicle = False
            
    def callback_parkinglot(self, data):
        if not self.is_parkinglot:
            self.parkinglot_pose = data.data
            self.is_parkinglot = True
            
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
            # print(self.turnpoint[0][0])
            # print(self.turnpoint[1][0])

            if self.is_vehicle and self.is_parkinglot:
                dis = m.sqrt((self.vehicle_pose[0] - self.parkinglot_pose[0])**2 + (self.vehicle_pose[1] - (465 - self.parkinglot_pose[1]))**2)
                # print(self.vehicle_pose)
                self.pub_dis = dis
                turndis = None
                self.pub_turndis = turndis
                self.is_vehicle = False
                # self.is_parkinglot = False
            
            # print(self.turnpoint)
            if self.is_turnpoint and self.turnpoint[0][2] == 14:
                turndis = m.sqrt((self.vehicle_pose[0] - self.turnpoint[0][0])**2 + ((465 - self.vehicle_pose[1]) - self.turnpoint[1][0])**2)
                # print('b')
                
                if self.vspeed < 0:
                    turndis = m.sqrt((self.vehicle_pose[0] - self.turnpoint[0][1])**2 + ((465 - self.vehicle_pose[1]) - self.turnpoint[1][1])**2)
                # self.pub_turn_dis.publish(turn_dis)
                self.pub_turndis = turndis

                
            elif self.is_turnpoint and self.turnpoint[0][2] // 10 == 2:
                # print('c')
                
                turndis = m.sqrt((self.vehicle_pose[0] - self.turnpoint[0][0])**2 + ((465 - self.vehicle_pose[1]) - self.turnpoint[1][0])**2)
                # self.pub_turn_dis.publish(turn_dis)
                self.pub_turndis = turndis

                
            else:
                # print('d')

                turndis = -1
                self.pub_turndis = turndis


            self.pub_vehivle_dis.publish(self.pub_dis)
            self.pub_turn_dis.publish(self.pub_turndis) 
            print(self.pub_dis, self.pub_turndis)

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