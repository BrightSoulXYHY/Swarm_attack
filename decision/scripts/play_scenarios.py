#!/usr/bin/env python
# coding=utf-8

import rospy
import os, sys
import json
import numpy as np
from swarm_msgs.msg import Pipeline, Pipeunit
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from geometry_msgs.msg import PoseStamped, TwistStamped, Point32
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import UInt64


class Px4Controller:
    def __init__(self, drone_name):
        self.arm_state = False
        self.offboard_state = False
        self.state = None
        self.command = TwistStamped()
        self.start_point = PoseStamped()
        self.start_point.pose.position.z = 4
        self.drone_name = drone_name
        self.rate = rospy.Rate(20)
        '''
        ros publishers
        '''
        # self.vel_pub = rospy.Publisher('{}/mavros/setpoint_velocity/cmd_vel'.format(self.drone_name), TwistStamped, queue_size=10)
        # self.pos_pub = rospy.Publisher('{}/mavros/setpoint_position/local'.format(self.drone_name), PoseStamped, queue_size=10)
        self.vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
        self.pos_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        '''
        ros services
        '''
        # self.armService = rospy.ServiceProxy('{}/mavros/cmd/arming'.format(self.drone_name), CommandBool)
        # self.flightModeService = rospy.ServiceProxy('{}/mavros/set_mode'.format(self.drone_name), SetMode)
        # self.takeoffService = rospy.ServiceProxy('{}/mavros/cmd/takeoff'.format(self.drone_name), CommandTOL)
        self.armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.flightModeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.takeoffService = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
        print("Px4 Controller Initialized!")

    def start(self):

        for _ in range(10):
            self.vel_pub.publish(self.command)
            self.arm_state = self.arm()
            self.offboard_state = self.offboard()
            self.rate.sleep()

        for _ in range(100):
            self.pos_pub.publish(self.start_point)
            self.rate.sleep()

        self.start_point.pose.position.x = 113
        self.start_point.pose.position.y = 1.7
        self.start_point.pose.position.z = 4
        for _ in range(300):
            self.pos_pub.publish(self.start_point)
            self.rate.sleep()

    def idle(self):
        print("I'm in idle state!")
        global drone_state, drone_state_pub
        idle_cmd = TwistStamped()
        while not rospy.is_shutdown():
            if drone_state == 40:
                self.vel_pub.publish(idle_cmd)
                drone_state_pub.publish(UInt64(40))
            self.rate.sleep()

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
    def __init__(self, play, drone_name):
        self.rate = rospy.Rate(1)
        self.play = play
        self.drone_name = drone_name
        self.pipe_pub = rospy.Publisher("{}/decision_info".format(self.drone_name), Pipeline , queue_size=10)
        print("{} Decision Initialized!".format(self.drone_name))

    def start(self):
        global drone_state, drone_state_pub
        cnt_pipe = 0
        while not rospy.is_shutdown():
            pipes = self.play[self.drone_name]
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
                # print(a)
                cnt_pipe += 1
                print("{} cnt_pipe is {}".format(self.drone_name, cnt_pipe))
                drone_state = 30
                drone_state_pub.publish(UInt64(drone_state))
                for _ in range(2):
                    self.pipe_pub.publish(a)
                    rospy.sleep(0.3)
            if cnt_pipe >= len_pipe:
                break
            self.rate.sleep()


def drone_state_cb(msg):
    global drone_state
    drone_state = msg.data



if __name__ == '__main__':
    # Load play file
    # file_path = os.path.join(os.path.expanduser('~'),"Swarm_ws/src/decision/scenarios","play_6directions_12drones.json")
    file_path = os.path.join(os.path.expanduser('~'),"Swarm_ws/src/decision/scenarios","play_2directions_12drones.json")
    play_file = open(file_path)
    play = json.load(play_file)
    print(json.dumps(play))

    # ROS init and get params
    rospy.init_node('decision_node', anonymous=True)
    fb_pos = Point32()
    param_id = rospy.get_param("~drone_id")
    drone_name = "drone_{}".format(param_id)

    # param_num = rospy.get_param("~drone_num")
    # Check validity
    # if len(play) != param_num:
    #     print("play file or launch file Error! len(play) != param_num")
    #     sys.exit(1)
    if drone_name not in play.keys():
        print("Name Error! {} not in {}".format(drone_name, file_path))
        sys.exit(1)
    
    # Subscribers and Publishers
    drone_state_sub = rospy.Subscriber("{}/state".format(drone_name), UInt64, drone_state_cb)
    drone_state_pub = rospy.Publisher("{}/state".format(drone_name), UInt64, queue_size=10)

    # Takeoff and fly to start point
    drone_state = 10
    drone_state_pub.publish(UInt64(drone_state))
    px4 = Px4Controller(drone_name)
    px4.start()

    # Interaction with Indoor controller
    drone_state = 20
    drone_state_pub.publish(UInt64(drone_state))
    dd = Decision(play, drone_name)
    dd.start()
    print("Finish")

    # Wait for next missions
    rospy.sleep(2)
    while True:
        print("{} drone_state is {}".format(drone_name, drone_state))
        if drone_state == 20:
            drone_state = 40
            drone_state_pub.publish(UInt64(drone_state))
            px4.idle()
            break
        else:
            rospy.sleep(0.2)