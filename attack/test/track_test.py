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


class Px4Controller:
    def __init__(self):
        self.arm_state = False
        self.offboard_state = False
        self.state = None
        self.command = TwistStamped()
        self.start_point = PoseStamped()
        self.feb_pos = PoseStamped()
        self.obs = Obj()
        self.obs_vel = Point32()

        self.start_point.pose.position.x = 0
        self.start_point.pose.position.y = 30
        self.start_point.pose.position.z = 4
        '''
        ros publishers
        '''
        self.vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
        '''
        ros subscriber
        '''
        self.local_pos_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.local_pos_cb)
        self.obs_sub = rospy.Subscriber("ue4_ros/obj", Obj, self.obj_cb)
        '''
        ros services
        '''
        self.armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.flightModeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.takeoffService = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
        print("obj Controller Initialized!")

    def start(self):
        rate = rospy.Rate(20)

        for _ in range(10):
            self.vel_pub.publish(self.command)
            self.arm_state = self.arm()
            self.offboard_state = self.offboard()
            rate.sleep()


        while (rospy.is_shutdown() is False):
            target = np.array([self.start_point.pose.position.x, self.start_point.pose.position.y, \
                               self.start_point.pose.position.z])
            feb = np.array([self.feb_pos.pose.position.x, self.feb_pos.pose.position.y, \
                               self.feb_pos.pose.position.z])
            desire = self.pos_control(target, feb, 0.8, 5)
            self.obs_vel.x = 0
            self.obs_vel.y = 0
            self.obs_vel.z = 0

            dir_obs = (self.feb_pos.pose.position.y - self.obs.position.y)/abs(self.feb_pos.pose.position.y - self.obs.position.y)
            if abs(self.feb_pos.pose.position.y - self.obs.position.y) < 6:
                self.obs_vel.x = dir_obs * 0.5
            self.command.twist.linear.x = desire[0] + self.obs_vel.x
            self.command.twist.linear.y = desire[1] + self.obs_vel.y
            self.command.twist.linear.z = desire[2] + self.obs_vel.z
            self.command.twist.angular.z = 0
            self.vel_pub.publish(self.command)
            # print("feb pos_x is {}".format(self.feb_pos.pose.position.x))
            rate.sleep()
        rospy.spin()

    def arm(self):
        if self.armService(True):
            return True
        else:
            print("Vehicle arming failed!")
            return False

    def local_pos_cb(self, msg):
        self.feb_pos = msg

    def obj_cb(self, msg):
        self.obs = msg

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



if __name__ == '__main__':
    rospy.init_node('attack_node', anonymous=True)
    
    px4 = Px4Controller()
    px4.start()
    # rospy.spin()
    print("Finish")