#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from spinning_lidar_motor_control.msg import MotorState



def callback(data):
    print(data)


def listener():
    global prev_time
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)
    
    rospy.Subscriber("/spinning_lidar/motor_state", MotorState, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()



if __name__ == '__main__':
    listener()