#!/usr/bin/env python
# coding=utf-8

'''
控制一架飞机躲避躲避障碍物，纯位置控制
打击目标为人，移动障碍物为小球
'''

import rospy
import os
import argparse
import json
import numpy as np
from swarm_msgs.msg import Pipeline, Pipeunit
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from geometry_msgs.msg import PoseStamped, TwistStamped, Point32
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import UInt64
from rflysim_ros_pkg.msg import Obj

#keyboard input
import threading
import tkinter
import time

def drone1_state_cb(msg):
    global state_1
    state_1 = msg

def drone2_state_cb(msg):
    global state_2
    state_2 = msg

def drone3_state_cb(msg):
    global state_3
    state_3 = msg

if __name__ == '__main__':
    rospy.init_node('is_attack_node', anonymous=True)
    state_1 = UInt64()
    state_2 = UInt64()
    state_3 = UInt64()
    attack = UInt64()
    drone1_state_sub = rospy.Subscriber("/drone_1/state", UInt64, drone1_state_cb)
    drone2_state_sub = rospy.Subscriber("/drone_2/state", UInt64, drone2_state_cb)
    drone3_state_sub = rospy.Subscriber("/drone_3/state", UInt64, drone3_state_cb)
    drone_state_pub = rospy.Publisher("/is_attrack", UInt64, queue_size=10)
    rate = rospy.Rate(20)

    while (rospy.is_shutdown() is False):
        if state_1.data == 40 and state_2.data == 40 and state_3.data == 40:
            attack.data = 20
            drone_state_pub.publish(attack)
            rate.sleep()
    print("Finish")