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
    def __init__(self):
        self.arm_state = False
        self.offboard_state = False
        self.state = None
        self.command = TwistStamped()
        self.start_point = PoseStamped()
        self.start_point.pose.position.z = 4
        '''
        ros publishers
        '''
        self.vel_pub = rospy.Publisher('mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
        self.pos_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
        '''
        ros services
        '''
        self.armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.flightModeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.takeoffService = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
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

        self.start_point.pose.position.x = 113
        self.start_point.pose.position.y = 1.7
        self.start_point.pose.position.z = 4
        for _ in range(300):
            self.pos_pub.publish(self.start_point)
            rate.sleep()

    def arm(self):
        if self.armService(True):
            return True
        else:
            print("Vehicle arming failed!")
            return False

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


class Decision:
    def __init__(self, play):
        self.pipe_pub = rospy.Publisher('decision_info', Pipeline , queue_size=10)
        self.rate = rospy.Rate(1)
        self.play = play
        print("Decision Initialized!")

    def start(self):
        global drone_state, drone_state_pub
        cnt_pipe = 0
        while not rospy.is_shutdown():
            pipes = self.play["Drone1"]
            len_pipe = len(pipes)
            if drone_state == 20:
                a = Pipeline()
                a.pipetype.data = pipes[cnt_pipe]["pipetype"]
                for u in pipes[cnt_pipe]["units"]:
                    b = Pipeunit()
                    b.middle = Point32(u[0][0], u[0][1], u[0][2])
                    b.left = Point32(u[1][0], u[1][1], u[1][2])
                    b.right = Point32(u[2][0], u[2][1], u[2][2])
                    if len(u) > 3:
                        b.bottom = Point32(u[3][0], u[3][1], u[3][2])
                        b.up = Point32(u[4][0], u[4][1], u[4][2])
                    a.units.append(b)
                print(a)
                cnt_pipe += 1
                drone_state = 30
                drone_state_pub.publish(UInt64(drone_state))
            self.pipe_pub.publish(a)
            if cnt_pipe >= len_pipe: break
            self.rate.sleep()

def drone_state_cb(msg):
    global drone_state
    drone_state = msg.data


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-p', '--play', default=os.path.join(os.path.expanduser('~'),"Swarm_ws/src/decision/scenarios","play.json"), help='play file path')
    args = parser.parse_args()
    print(args)
    play_file = open(args.play)
    play = json.load(play_file)
    print(json.dumps(play))

    rospy.init_node('decision_node', anonymous=True)
    drone_state_sub = rospy.Subscriber('drone_1/state', UInt64, drone_state_cb)
    drone_state_pub = rospy.Publisher('drone_1/state', UInt64, queue_size=10)
    drone_state = 10
    drone_state_pub.publish(UInt64(drone_state))
    px4 = Px4Controller()
    px4.start()
    drone_state = 20
    drone_state_pub.publish(UInt64(drone_state))
    dd = Decision(play)
    dd.start()
    print("Finish")