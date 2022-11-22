#!/usr/bin/env python3
#-*- coding: utf-8 -*-

import os
import rospy
from std_msgs.msg import Float32

class Joystick:
    def __init__(self):
        self.ros_input_servo = 0.0
        self.ros_input_motor = 0.0

        self.is_keyboard_input = False
        self.keyboard_input = None

        self.pub_ctrl_motor = rospy.Publisher('/arduino_ctrl/ctrl_motor', Float32, queue_size=1)
        self.pub_ctrl_servo = rospy.Publisher('/arduino_ctrl/ctrl_servo', Float32, queue_size=1)

        rospy.init_node('joystick', anonymous=False)

        self.is_init = False
    
    
    def execute(self):
        while not rospy.is_shutdown():
            # if self.is_init == False:
            #     rospy.sleep(5.)
            #     self.is_keyboard_input = True
            #     self.keyboard_input = 'w'
            #     self.set_ctrl(self.keyboard_input)
            #     self.is_init = True
            #     self.publish_ctrl()
            #     print("Started")
            
            self.get_keyboard_input()
            if self.is_keyboard_input:
                self.publish_ctrl()
                self.is_keyboard_input = False
    
    def get_keyboard_input(self):
        print("--------------  조작법  -------------------\n")
        print("           w(전진)\n")
        print("a(왼쪽 조향) s(정지/가운데 조향) d(오른쪽 조향)\n")
        print("           x(후진)\n")
        print("조작 입력 : \n")
        self.keyboard_input = input()
        self.is_keyboard_input = True
        self.set_ctrl(self.keyboard_input)
        

    def set_ctrl(self, keyboard_input, verbose = False):
        motor_duty  = 150.0    # 0~255
        # motor_duty  = 120.0    # 0~255

        servo_angle = 20.0     # 0~20 [deg]
        servo_angle_0 = 0.0
        
        if keyboard_input == 'w':          # 앞
            self.ros_input_motor = motor_duty
            self.ros_input_servo = servo_angle_0
        
        elif keyboard_input == 's':        # 정지
            self.ros_input_motor = 0.0
            self.ros_input_servo = servo_angle_0
        
        elif keyboard_input == 'x':        # 후진
            self.ros_input_motor = -motor_duty
            self.ros_input_servo = servo_angle_0
        
        elif keyboard_input == 'a':         # 왼쪽 조향
            self.ros_input_servo = servo_angle
        
        elif keyboard_input == 'd':         # 오른쪽 조향
            self.ros_input_servo = -servo_angle
        
        else:
            self.ros_input_motor = 0.0
            self.ros_input_servo = servo_angle_0
        

    def publish_ctrl(self):
        self.pub_ctrl_motor.publish(self.ros_input_motor)
        self.pub_ctrl_servo.publish(self.ros_input_servo)


if __name__ == '__main__':
    joystick = Joystick()
    joystick.execute()


# if __name__ == '__main__':
#     joystick = Joystick()
#     # joystick.execute()
#     while not rospy.is_shutdown():
#         joystick.execute()
    
#     rospy.spin()