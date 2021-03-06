#!/usr/bin/env python
# coding=utf-8

# 本程序为决策模块主要实现。负责调度任务全流程，从剧本中拿到管道并与集群管道控制器交互

import rospy
import os, sys
import json
import threading
import pickle
import numpy as np
import time

from swarm_msgs.msg import Pipeline, Pipeunit
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from geometry_msgs.msg import PoseStamped, TwistStamped, Point32
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import UInt64
from prometheus_msgs.msg import DetectionInfo

p_mav_e = np.array([0., 0., 0.])
yaw_mav = 0
R_ce = np.identity(3)
passable_list = []
drone_state = 0         # 当前无人机状态机，含义见state.jpg
drone1_state = 0        # 1号无人机状态机
drone2_state = 0        # 2号无人机状态机

# 无人机控制类
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

    # 任务开始前的一些动作，包括解锁、进offboard、飞到厂房前
    def start(self):
        for _ in range(10):
        # while self.arm_state!=True or self.offboard_state!=True:
            vel_pub.publish(self.command)
            self.arm_state = self.arm()
            self.offboard_state = self.offboard()
            self.rate.sleep()

        for _ in range(100):
            pos_pub.publish(self.start_point)
            self.rate.sleep()

        self.start_point.pose.position.x = 113
        self.start_point.pose.position.y = 1.7
        self.start_point.pose.position.z = 4
        for _ in range(300):
            pos_pub.publish(self.start_point)
            self.rate.sleep()

    # 悬停
    def idle(self):
        print("I'm in idle state!")
        global drone_state, drone_state_pub
        idle_cmd = TwistStamped()
        while not rospy.is_shutdown():
            if drone_state == 40:
                vel_pub.publish(idle_cmd)
                drone_state_pub.publish(UInt64(40))
            self.rate.sleep()

    # 解锁
    def arm(self):
        if armService(True):
            return True
        else:
            print("Vehicle arming failed!")
            return False

    # 上锁
    def disarm(self):
        if armService(False):
            return True
        else:
            print("Vehicle disarming failed!")
            return False

    # 进offboard模式
    def offboard(self):
        if flightModeService(custom_mode='OFFBOARD'):
            return True
        else:
            print("Vechile Offboard failed")
            return False

    # 起飞
    def takeoff(self):
        if takeoffService(altitude=3):
            return True
        else:
            print("Vechile Takeoff failed")
            return False


# 无人机搜索类，用于控制前两架飞机搜索
class Search:
    def __init__(self, param_id):
        self.id = param_id
        self.search_state = 0
        self.dis_th = 1
        self.search_vel = 3
        self.k_yawrate = 0.2
        if self.id == 1:
            self.waypoints = [[130,-8,4,0], [258,-10,4,np.pi/2], [255,29,4,np.pi], [142,27,4,3*np.pi/2], [113,1.7,4,2*np.pi]]
        elif self.id == 2:
            self.waypoints = [[142,27,4,0], [258,29,4,-np.pi/2], [255,-10,4,-np.pi], [130,-8,4,-3*np.pi/2], [113,1.7,4,-2*np.pi]]
        self.start_time = time.time()

    # 每步迭代
    def step(self):
        global drone_state, drone1_state, drone2_state
        if self.id == 1:
            self.waypointsFlight()
            return drone_state == 20 
        elif self.id == 2:
            self.waypointsFlight()
            return drone_state == 20
        else:
            self.idle()
            return False if time.time() - self.start_time < 110 else True

    # 搜索的路径点
    def waypointsFlight(self):
        global drone_state
        if self.search_state >= 5:
            drone_state = 20
            self.idle()
            return
        self.pointFlight(self.waypoints[self.search_state])

    # 定点飞行，P控制
    def pointFlight(self, target):
        global p_mav_e, yaw_mav
        direction = np.array(target[:3]) - p_mav_e
        # if self.id == 1:
        #     print("id: {}, search_state: {}, delta_pos: {}, target: {}, mav_pos: {}".format(self.id, self.search_state, np.linalg.norm(direction), target, p_mav_e))
        if np.linalg.norm(direction) < self.dis_th:
            self.search_state += 1
        direction /= np.linalg.norm(direction)
        v_d = self.search_vel * direction
        yawrate_d = self.k_yawrate*self._minAngleDiff(target[3], yaw_mav)

        point_cmd = TwistStamped()
        point_cmd.twist.linear.x = v_d[0]
        point_cmd.twist.linear.y = v_d[1]
        point_cmd.twist.linear.z = v_d[2]
        point_cmd.twist.angular.z = yawrate_d
        vel_pub.publish(point_cmd)

    # 悬停等待
    def idle(self):
        global drone_state, drone_state_pub
        idle_cmd = TwistStamped()
        vel_pub.publish(idle_cmd)
        drone_state_pub.publish(UInt64(drone_state))

    # 最小角度。因为yaw角定义为[-pi, pi]，找到与期望角最小的角度差
    def _minAngleDiff(self, a, b):
        diff = a - b
        if diff < 0:
            diff += 2*np.pi
        if diff < np.pi:
            return diff
        else:
            return diff - 2*np.pi


# 无人机决策类，用于与集群管道飞行交互和状态切换
class Decision:
    def __init__(self, play, drone_name):
        self.rate = rospy.Rate(1)
        self.play = play
        self.drone_name = drone_name
        self.pipe_pub = rospy.Publisher("{}/decision_info".format(self.drone_name), Pipeline , queue_size=10)
        print("{} Decision Initialized!".format(self.drone_name))

    # 实际负责交互的内容
    def start(self):
        global drone_state, drone_state_pub
        cnt_pipe = 0
        while not rospy.is_shutdown():
            # 从剧本中拿到属于自己的管道
            pipes = self.play[self.drone_name]
            len_pipe = len(pipes)
            # 状态为20才准备发送管道，否则应该正在飞行管道过程中
            if drone_state == 20:
                # 构造第cnt_pipe节管道片段
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
                # 发送管道且状态机为30
                drone_state = 30
                for _ in range(10):
                    drone_state_pub.publish(UInt64(30))
                    self.pipe_pub.publish(a)
                    rospy.sleep(0.1)
            # 所有管道发送完，结束集群飞行阶段
            if cnt_pipe >= len_pipe:
                break
            self.rate.sleep()


# 各种回调函数，与前面Subscriber对应
def drone_state_cb(msg):
    global drone_state
    drone_state = msg.data

def drone1_state_cb(msg):
    global drone1_state
    drone1_state = msg.data

def drone2_state_cb(msg):
    global drone2_state
    drone2_state = msg.data

def bias_cb(msg):
    global p_mav_e, yaw_mav, R_ce
    p_mav_e = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
    q0, q1, q2, q3 = msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z
    yaw_mav = np.arctan2(2*(q0*q3 + q1*q2), 1-2*(q2*q2 + q3*q3))
    R_be = np.array([[q0**2+q1**2-q2**2-q3**2, 2*(q1*q2-q0*q3), 2*(q1*q3+q0*q2)],
                     [2*(q1*q2+q0*q3), q0**2-q1**2+q2**2-q3**2, 2*(q2*q3-q0*q1)],
                     [2*(q1*q3-q0*q2), 2*(q2*q3+q0*q1), q0**2-q1**2-q2**2+q3**2]])
    R_cb = np.array([[0,0,1],[-1,0,0],[0,-1,0]])
    R_ce = R_cb.dot(R_be)
    # print("p_mav_e: {}".format(p_mav_e))

def ellipse_cb(msg):
    global p_mav_e, R_ce, passable_list
    p_ell_c = np.array(msg.position)
    p_ell_e = p_mav_e + R_ce * p_ell_c
    passable_list.append(p_ell_e)

def yolo_cb(msg):
    global p_mav_e, R_ce, passable_list
    # if not (msg.detected and msg.category==2): return
    if not msg.detected: return
    p_rec_c = np.array(msg.position)
    p_rec_e = p_mav_e + R_ce * p_rec_c
    passable_list.append(p_rec_e)

def spin():
    rospy.spin()


if __name__ == '__main__':
    # 载入剧本文件
    # file_path = os.path.join(os.path.expanduser('~'),"Swarm_ws/src/decision/scenarios","play_6directions_12drones.json")
    file_path = os.path.join(os.path.expanduser('~'),"Swarm_ws/src/decision/scenarios","play_2directions_12drones.json")
    play_file = open(file_path)
    play = json.load(play_file)
    # print(json.dumps(play))

    # ROS初始化，从launch文件获取参数
    rospy.init_node('decision_node', anonymous=True)
    spin_thread = threading.Thread(target = spin)
    spin_thread.start()
    fb_pos = Point32()
    param_id = rospy.get_param("~drone_id")
    drone_name = "drone_{}".format(param_id)

    # 判断是否有对应的剧本
    if drone_name not in play.keys():
        print("Name Error! {} not in {}".format(drone_name, file_path))
        sys.exit(1)
    
    # Subscribers 和 Publishers
    drone_state_sub = rospy.Subscriber("{}/state".format(drone_name), UInt64, drone_state_cb)                       # 订阅状态机
    drone_state_pub = rospy.Publisher("{}/state".format(drone_name), UInt64, queue_size=10)                         # 发布状态机

    bias_pos_sub = rospy.Subscriber("{}/mavros/local_position/pose_cor".format(drone_name), PoseStamped, bias_cb)   # 订阅校正后的位姿
    ellipse_sub = rospy.Subscriber("/prometheus/object_detection/ellipse_det", DetectionInfo, ellipse_cb)           # 订阅椭圆检测
    yolo_sub = rospy.Subscriber("/prometheus/object_detection/yolo_det", DetectionInfo, yolo_cb)                    # 订阅目标检测
    # mavros topics
    vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)                     # 发布速度指令（重要）
    pos_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)                        # 发布位置指令（初始化飞到厂房前使用）
    armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)                                              # 解锁服务
    flightModeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)                                             # 飞行模式服务
    takeoffService = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)                                          # 起飞服务
    print("Px4 Controller Initialized with {}".format(drone_name))

    # 飞机初始化，解锁、offboard、飞到厂房前
    drone_state = 10
    drone_state_pub.publish(UInt64(drone_state))
    px4 = Px4Controller(drone_name)
    px4.start()

    # 搜索可穿行孔洞
    drone_state = 15
    drone_state_pub.publish(UInt64(drone_state))
    s = Search(int(param_id))       # 实例化搜索类，工作都在Search()里进行
    search_done = False
    if int(param_id) == 1:          # 订阅1、2号飞机状态。为防止消息死锁，判断自己id，不订阅自己状态
        drone2_state_sub = rospy.Subscriber("drone_2/state".format(drone_name), UInt64, drone2_state_cb)
    elif int(param_id) == 2:
        drone1_state_sub = rospy.Subscriber("drone_1/state".format(drone_name), UInt64, drone1_state_cb)
    else:
        drone1_state_sub = rospy.Subscriber("drone_1/state".format(drone_name), UInt64, drone1_state_cb)
        drone2_state_sub = rospy.Subscriber("drone_2/state".format(drone_name), UInt64, drone2_state_cb)
    while not search_done:          # 搜索执行中
        search_done = s.step()
        rospy.sleep(0.02)
    # 搜索完成，打印并保存可穿行孔洞
    print("passable_list: {}".format(passable_list))
    f = open(os.path.join(os.path.expanduser('~'),"Swarm_ws/src","passable_list.pkl"), 'w')
    pickle.dump(passable_list, f)
    f.close()

    # 集群飞行管道
    drone_state = 20
    drone_state_pub.publish(UInt64(drone_state))
    dd = Decision(play, drone_name) # 实例化并开始管道飞行
    dd.start()
    print("Finish")

    # 状态机为40，等待后续任务
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
    rospy.spin()
