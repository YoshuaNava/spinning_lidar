#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from spinning_lidar_motor_control.msg import MotorState



def callback(data):
    print(data)


def listener():
    rospy.init_node('spinning_lidar_motor_control_tester', anonymous=True)
    
    rospy.Subscriber("/spinning_lidar/motor_state", MotorState, callback)
    
    rospy.spin()



if __name__ == '__main__':
    listener()