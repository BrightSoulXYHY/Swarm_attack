#!/usr/bin/env python
# coding=utf-8

# 本程序用于计算集群的全局坐标。PX4只能获取无人机相对于自己home点的局部坐标，我们通过无人机的初始摆放，计算一个所有无人机相对于全局home点的全局统一坐标。

import rospy
import os
import argparse
import numpy as np
from geometry_msgs.msg import PoseStamped, TwistStamped
param_id = 0
param_num = 0
local_pos_bia = PoseStamped()

# 局部坐标上添加偏移量，得到全局坐标
def local_pos_cb(msg):
    global local_pos_pub, param_num, param_id,local_pos_bia
    '''
    # 以及UE4生成的飞机法则进行编号，如果ue4生成方式变化，这里也需要编号
    cloum = (param_num+1)//2
    if param_id > cloum:
        bias_ue4_x = 2
    else:
        bias_ue4_x = 0
    bias_ue4_y = 2 * ((param_id -1) % cloum)
    '''
    cloum = int(np.sqrt(param_num-1)) + 1
    bias_ue4_x = 2 * ((param_id - 1)//cloum)        # UE4中x偏移
    bias_ue4_y = 2 * ((param_id - 1) % cloum)       # UE4中y偏移

    
    # 坐标转换，ue4_y对应mavros_x，ue4_x对应mavros_y
    local_pos_bia.header = msg.header
    local_pos_bia.pose.position.x = msg.pose.position.x + bias_ue4_y
    local_pos_bia.pose.position.y = msg.pose.position.y + bias_ue4_x
    local_pos_bia.pose.position.z = msg.pose.position.z
    local_pos_bia.pose.orientation = msg.pose.orientation
    local_pos_pub.publish(local_pos_bia)


if __name__ == '__main__':
    # ROS初始化，获取参数
    rospy.init_node('pose_cor', anonymous=True)
    param_id = rospy.get_param("~drone_id")
    param_num = rospy.get_param("~drone_num")

    local_pos_pub = rospy.Publisher('/mavros/local_position/pose_cor', PoseStamped, queue_size=10)  # 发布校正后的坐标
    local_pos_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, local_pos_cb)      # 订阅局部坐标
    rospy.spin()

    print("Finish")
