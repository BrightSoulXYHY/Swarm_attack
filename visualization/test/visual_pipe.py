#!/usr/bin/env python
# coding=utf-8

import rospy
import numpy as np
from swarm_msgs.msg import Pipeline, Pipeunit
from geometry_msgs.msg import PoseStamped, TwistStamped, Point32
from rflysim_ros_pkg.msg import Obj
import math

idex = 1
kp_x = 1
kp_y = 1
kp_z = 1.5
rate_to_degree = 180/math.pi
obj_type = 27
obj_yaw = 0
obj_pitch = 0
obj_x = 0
obj_y = 0
obj_z = 0

def pipeline_cb(msg):
    global obj_msg, idex, kp_x, kp_y, kp_z, rate_to_degree, obj_type, obj_yaw, obj_x,obj_y, obj_z, obj_pitch
    piple = msg.units

    for i,unit in enumerate(piple):
        # left = unit.left.y - 
        # right = unit.right.y
        size_weight = math.sqrt((unit.left.y - unit.right.y) * (unit.left.y - unit.right.y) + (unit.left.x - unit.right.x) * (unit.left.x - unit.right.x))
        # size_weight = abs(unit.left.z - unit.right.z)
        bottom = unit.bottom.z
        up = unit.up.z
        middle = [unit.middle.x, unit.middle.y, unit.middle.z]
        front = unit.middle.x
        weight = unit.middle.y
        height = unit.middle.z
        if i != 0:
            idex = idex + 1
            if idex == 5:
                idex = 2
            
            obj_x = kp_x * size_weight
            obj_y = obj_x
            obj_z = kp_z * math.sqrt((front - last_front) * (front - last_front) + 
                                     (weight - last_weight) * (weight - last_weight) + 
                                     (height - last_height) * (height - last_height))
            obj_yaw = math.atan2(weight - last_weight, front - last_front)
            obj_pitch = math.atan2(height, last_height)

        obj_msg.id = idex
        obj_msg.type = obj_type
        obj_msg.position.x = middle[0]
        obj_msg.position.y = middle[1]
        obj_msg.position.z = middle[2]
        obj_msg.angule.x = obj_pitch
        
        obj_msg.angule.y = 0
        obj_msg.angule.z = -obj_yaw
        obj_msg.size.x = obj_x
        obj_msg.size.y = obj_x
        obj_msg.size.z = obj_z
        obj_pub.publish(obj_msg)
        print("idex is ok {}".format(idex))
        print("obj_z is ok {}".format(obj_z))
        print("obj_pitch is ok {}".format(obj_pitch))
        print("obj_yaw is ok {}".format(obj_yaw))
        # last_left = left
        # last_right = right
        last_bottom = bottom
        last_up = up
        last_front = front
        last_height = height
        last_weight = weight

if __name__ == '__main__':
    rospy.init_node('pipeline_vision', anonymous=True)

    global obj_pub, obj_msg
    obj_msg = Obj()
    obj_pub = rospy.Publisher("ue4_ros/obj", Obj, queue_size=10)
    rate = rospy.Rate(1)
    rospy.Subscriber("decision_info", Pipeline, pipeline_cb)
    # while not rospy.is_shutdown():
    #     obj_msg.id = 2
    #     obj_msg.type = 24#30
    #     obj_msg.position.x = 0
    #     obj_msg.position.y = 0
    #     obj_msg.position.z = 0
    #     obj_msg.angule.x = 0
    #     obj_msg.angule.y = 0
    #     obj_msg.angule.z = 0#0.25*math.pi#-30/2
    #     obj_msg.size.x = 1
    #     obj_msg.size.y = 1
    #     obj_msg.size.z = 1
    #     obj_pub.publish(obj_msg)
    #     rate.sleep()
    rospy.spin()

