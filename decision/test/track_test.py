#!/usr/bin/env python
# coding=utf-8

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


class Px4Controller:
    def __init__(self, drone_id, drone_num, feb_bias):
        self.arm_state = False
        self.offboard_state = False
        self.state = None
        self.command = TwistStamped()
        self.start_point = PoseStamped()
        self.drone_id = drone_id
        self.drone_num = drone_num
        self.drone_pos_feb = feb_bias
        '''
        ros publishers
        '''
        # self.bias_sub = rospy.Sublisher('/drone_%s/original_bias'%(self.drone_id), Point32, bias_cb)
        self.vel_pub = rospy.Publisher('/drone_%s/mavros/setpoint_velocity/cmd_vel'%(self.drone_id), TwistStamped, queue_size=10)
        self.pos_pub = rospy.Publisher('/drone_%s/mavros/setpoint_position/local'%(self.drone_id), PoseStamped, queue_size=10)
        self.pos_pub = rospy.Publisher('/drone_%s/mavros/setpoint_position/local'%(self.drone_id), PoseStamped, queue_size=10)
        '''
        ros services
        '''
        self.armService = rospy.ServiceProxy('/drone_%s/mavros/cmd/arming'%(self.drone_id), CommandBool)
        self.flightModeService = rospy.ServiceProxy('/drone_%s/mavros/set_mode'%(self.drone_id), SetMode)
        self.takeoffService = rospy.ServiceProxy('/drone_%s/mavros/cmd/takeoff'%(self.drone_id), CommandTOL)
        print("Px4 Controller Initialized!")

    def start(self):
        rate = rospy.Rate(20)

        for _ in range(10):
            self.vel_pub.publish(self.command)
            self.arm_state = self.arm()
            self.offboard_state = self.offboard()
            rate.sleep()

        for _ in range(100):
            self.pos_pub.publish(self.start_point)
            rate.sleep()

        while (rospy.is_shutdown() is False):
            self.start_point.pose.position.x = 113
            self.start_point.pose.position.y = 1.7
            self.start_point.pose.position.z = 0
            target = np.array([113, 1.7, 4])
            feb = np.array([self.drone_pos_feb.x, self.drone_pos_feb.y, self.drone_pos_feb.z])
            desire = self.pos_control(target, feb, 0.8, 5)
            self.command.twist.linear.x = desire[0]
            self.command.twist.linear.y = desire[1]
            self.command.twist.linear.z = desire[2]
            self.command.twist.angular.z = 0
            self.vel_pub.publish(self.command)
            rate.sleep()



    def arm(self):
        if self.armService(True):
            return True
        else:
            print("Vehicle arming failed!")
            return False

    #期望位置，反馈位置，位置比例系数，速读限幅
    def pos_control(self, target_pos, feb_pos, kp, sat_vel):
        err_pos = target_pos - feb_pos
        cmd_pos_vel = self.sat(kp * err_pos, sat_vel)
        return [cmd_pos_vel[0], cmd_pos_vel[1], cmd_pos_vel[2]]

    def disarm(self):
        if self.armService(False):
            return True
        else:
            print("Vehicle disarming failed!")
            return False

    def offboard(self):
        if self.flightModeService(custom_mode='OFFBOARD'):
            return True
        else:
            print("Vechile Offboard failed")
            return False

    def takeoff(self):
        if self.takeoffService(altitude=3):
            return True
        else:
            print("Vechile Takeoff failed")
            return False

    def sat(self, a, maxv):
        n = np.linalg.norm(a)
        if n > maxv:
            return a/n*maxv
        else:
            return a

def bias_cb(msg):
    global feb_pos
    feb_pos.x = msg.x
    feb_pos.y = msg.y
    feb_pos.z = msg.z


if __name__ == '__main__':
    rospy.init_node('decision_node', anonymous=True)
    feb_pos = Point32()
    param_id = rospy.get_param("~drone_id")
    param_num = rospy.get_param("~drone_num")
    bias_pos_sub = rospy.Subscriber('/drone_%s/mavros/local_position/pose_cor'%(param_id), Point32, bias_cb)

    px4 = Px4Controller(param_id,param_num,feb_pos)
    px4.start()
    rospy.spin()
    print("Finish")