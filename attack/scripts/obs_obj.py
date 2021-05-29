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


class Obj_Move_Controller:
    def __init__(self, mav_id, mav_num):
        self.arm_state = False
        self.offboard_state = False
        self.state = None
        self.mav_pos = PoseStamped()
        self.mav_bias = Point32()
        self.start_sphere = Point32()

        self.drone_id = mav_id
        self.drone_num = mav_num

        self.sphere_msg = Obj()
        self.people_msg = Obj()
        
        self.sphere_msg.id = 60 + self.drone_id
        self.sphere_msg.type = 152
        self.sphere_msg.size.x = 0.1
        self.sphere_msg.size.y = 0.1
        self.sphere_msg.size.z = 0.1
        self.sphere_msg.position.x = 0
        self.sphere_msg.position.y = 30
        self.sphere_msg.position.z = 0

        self.people_msg.id = 50
        self.people_msg.type = 30
        self.people_msg.position.x = 0
        self.people_msg.position.y = 30
        self.people_msg.position.z = 0
        self.people_msg.angule.z = 0
        self.people_msg.size.x = 1
        self.people_msg.size.y = 1
        self.people_msg.size.z = 1

        self.start_sphere.x = self.people_msg.position.x
        self.start_sphere.y = self.people_msg.position.y
        self.start_sphere.z = self.people_msg.position.z
        '''
        ros publishers
        '''
        self.obj_pub = rospy.Publisher("ue4_ros/obj", Obj, queue_size=10)
        self.obj_pos_pub = rospy.Publisher("ue4_ros/drone_%s/pos"%(param_id), Point32, queue_size=10)
        '''
        ros subscriber
        '''
        # self.local_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.local_pos_cb)
        self.bias_pos_sub = rospy.Subscriber('/drone_%s/mavros/local_position/pose_cor'%(param_id), Point32, self.bias_cb)
      
        print("obj Controller Initialized!")

    def start(self):
        rate = rospy.Rate(20)
        obj_pos = Point32()
        cnt = -1
        while (rospy.is_shutdown() is False):

            start = self.start_sphere.y
            end = self.mav_bias.y
             
            self.people_msg.angule.z = self.people_msg.angule.z + 0.1

            if abs(start - end) < 15:
                if self.drone_id == 3:
                    self.sphere_msg.position.y = self.sphere_msg.position.y - 0.1
                    # self.sphere_msg.position.x = 0
                elif self.drone_id == 1:
                    self.sphere_msg.position.y = self.sphere_msg.position.y + 0.1
                    # self.sphere_msg.position.x = 0
                elif self.drone_id == 2:
                    self.sphere_msg.position.y = 0
                    # self.sphere_msg.position.x = self.sphere_msg.position.x - 0.1

            start_x = self.start_sphere.x
            end_x = self.mav_bias.x
            if abs(start_x - end_x) < 15:
                if self.drone_id == 3:
                    # self.sphere_msg.position.y = self.sphere_msg.position.y - 0.1
                    self.sphere_msg.position.x = 0
                elif self.drone_id == 1:
                    # self.sphere_msg.position.y = self.sphere_msg.position.y + 0.1
                    self.sphere_msg.position.x = 0
                elif self.drone_id == 2:
                    # self.sphere_msg.position.y = 0
                    self.sphere_msg.position.x = self.sphere_msg.position.x - 0.1

                if self.sphere_msg.position.z != self.mav_bias.z:
                    sphere_vel = self.sphere_control(self.mav_bias.z, self.sphere_msg.position.z, 0.8, 0.1)
                    self.sphere_msg.position.z = sphere_vel + self.sphere_msg.position.z
            
            obj_pos.x = self.sphere_msg.position.x
            obj_pos.y = self.sphere_msg.position.y
            obj_pos.z = self.sphere_msg.position.z
            # print("sphere_x:{}".format(self.sphere_msg.position.y))
            if self.drone_id == 1:
                self.obj_pub.publish(self.people_msg)
            self.obj_pub.publish(self.sphere_msg)
            self.obj_pos_pub.publish(obj_pos)
            rate.sleep()
        rospy.spin
    
    def local_pos_cb(self, msg):
        self.mav_pos = msg

    def bias_cb(self, msg):
        self.mav_bias = msg

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

    def sphere_control(self, target_pos, feb_pos, kp, sat_vel):
        err_pos = target_pos - feb_pos
        cmd_pos_vel = self.sat(kp * err_pos, sat_vel)
        return cmd_pos_vel

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
    rospy.init_node('move_obj_node', anonymous=True)
    param_id = rospy.get_param("~drone_id")
    param_num = rospy.get_param("~drone_num")
    
    Obj_Move = Obj_Move_Controller(param_id, param_num)
    Obj_Move.start()
    print("Finish")