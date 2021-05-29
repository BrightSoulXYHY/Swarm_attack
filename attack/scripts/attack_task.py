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


class Px4Controller:
    def __init__(self, mav_id, mav_num):
        self.arm_state = False
        self.offboard_state = False
        self.state = None
        self.command = TwistStamped()
        self.start_point = PoseStamped()
        self.feb_pos = PoseStamped()
        self.feb_bias_pos = Point32()
        self.obs = Point32()
        self.obs_vel = Point32()
        self.drone_id = mav_id
        self.drone_num = mav_num
        self.is_attack = False
        self.state_drone = UInt64()
        self.attack_num = UInt64()
        self.attack_pos = [195,10,1.5]
        mav_x = [0, -30, 0]
        mav_y = [60, 30, 0]

        self.start_point.pose.position.x = mav_x[self.drone_id - 1]
        self.start_point.pose.position.y = mav_y[self.drone_id - 1]
        self.start_point.pose.position.z = 4
        '''
        ros publishers
        '''
        self.vel_pub = rospy.Publisher('/drone_%s/mavros/setpoint_velocity/cmd_vel'%(self.drone_id), TwistStamped, queue_size=10)
        self.drone_state_pub = rospy.Publisher("/drone_%s/state"%(self.drone_id), UInt64, queue_size=10)
        '''
        ros subscriber
        '''
        # self.local_pos_sub = rospy.Subscriber('/drone_%s/mavros/setpoint_position/local'%(self.drone_id), PoseStamped, self.local_pos_cb)
        self.bias_pos_sub = rospy.Subscriber('/drone_%s/mavros/local_position/pose_cor'%(self.drone_id), PoseStamped, self.bias_cb)
        self.obs_sub = rospy.Subscriber("ue4_ros/drone_3/pos", Point32, self.obj_cb)
        self.state_sub = rospy.Subscriber("/drone_%s/state"%(self.drone_id), UInt64, self.state_cb)
        self.attack_key = rospy.Subscriber("/is_attrack", UInt64, self.attack_cb)
        '''
        ros services
        '''
        self.armService = rospy.ServiceProxy('/drone_%s/mavros/cmd/arming'%(self.drone_id), CommandBool)
        self.flightModeService = rospy.ServiceProxy('/drone_%s/mavros/set_mode'%(self.drone_id), SetMode)
        self.takeoffService = rospy.ServiceProxy('/drone_%s/mavros/cmd/takeoff'%(self.drone_id), CommandTOL)
        print("obj Controller Initialized!")

    def start(self):
        rate = rospy.Rate(20)

        for _ in range(10):
            self.vel_pub.publish(self.command)
            self.arm_state = self.arm()
            self.offboard_state = self.offboard()
            rate.sleep()

        desire = [0,0,0]
        while (rospy.is_shutdown() is False):
            # target = np.array([self.start_point.pose.position.x, self.start_point.pose.position.y, \
            #                    self.start_point.pose.position.z])
            feb = np.array([self.feb_bias_pos.x, self.feb_bias_pos.y, \
                               self.feb_bias_pos.z])
            # if self.is_attack == True:
            # if self.state_drone_1 == 40 and self.state_drone_2 == 40 and self.state_drone_3 == 40:
            #     self.is_attack == True
            if self.attack_num.data == 20:
                self.is_attack = True

            if self.is_attack == True:
                drone_state = 50
                self.drone_state_pub.publish(UInt64(drone_state))
                target = np.array([self.attack_pos[0], self.attack_pos[1], self.attack_pos[2]])
                desire = self.pos_control(target, feb, 0.8, 5)
            else:
                desire[0] = 0
                desire[1] = 0
                desire[2] = 0

            self.obs_vel.x = 0
            self.obs_vel.y = 0
            self.obs_vel.z = 0

            direct_obs = -1 #(self.feb_bias_pos.x - self.obs.x)/abs(self.feb_bias_pos.x - self.obs.x)
            if abs(self.feb_bias_pos.x - self.obs.x) < 8 and self.drone_id == 1:
                self.obs_vel.y = direct_obs * 2
            self.command.twist.linear.x = desire[0] + self.obs_vel.x
            self.command.twist.linear.y = desire[1] + self.obs_vel.y
            self.command.twist.linear.z = desire[2] + self.obs_vel.z
            self.command.twist.angular.z = 0
            if self.state_drone.data == 50:
                self.vel_pub.publish(self.command)
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
    
    def state_cb(self,msg):
        self.state_drone = msg

    def attack_cb(self, msg):
        self.attack_num = msg

    def bias_cb(self, msg):
        self.feb_bias_pos.x = msg.pose.position.x
        self.feb_bias_pos.y = msg.pose.position.y
        self.feb_bias_pos.z = msg.pose.position.z

    def obj_cb(self, msg):
        self.obs = msg

    #期望位置，反馈位置，位置比例系数，速读限幅
    def pos_control(self, target_pos, feb_pos, kp, sat_vel):
        err_pos = target_pos - feb_pos
        cmd_pos_vel = self.sat(kp * err_pos, sat_vel)
        return [cmd_pos_vel[0], cmd_pos_vel[1], cmd_pos_vel[2]]

    def read_kbd_input(self):
        win = tkinter.Tk()
        frame = tkinter.Frame(win,width=200,height=120)
        frame.bind("<Key>",self.call)
        frame.focus_set()
        frame.pack()
        win.mainloop()

    def call(self, event):
        k = event.keysym
        if k == "a":
            self.is_attack = True
        time.sleep(0.02)

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
    param_id = rospy.get_param("~drone_id")
    param_num = rospy.get_param("~drone_num")

    px4 = Px4Controller(param_id, param_num)

    # inputThread = threading.Thread(target=px4.read_kbd_input)
    # inputThread.start()

    px4.start()
    # rospy.spin()
    print("Finish")