#!/usr/bin/env python
# coding=utf-8

import rospy
import numpy as np
from swarm_msgs.msg import Pipeline, Pipeunit
from geometry_msgs.msg import PoseStamped, TwistStamped, Point32
from rflysim_ros_pkg.msg import Obj


def draw_unit(id, start, end):
    global obj_pub, param_type
    obj_msg = Obj()
    obj_msg.id = id
    obj_msg.type = param_type
    obj_msg.position.x = (start.x + end.x) / 2
    obj_msg.position.y = (start.y + end.y) / 2
    obj_msg.position.z = 0.3
    obj_msg.angule.z = np.arctan2(end.y - start.y, end.x - start.x)
    obj_msg.size.x = np.linalg.norm([end.y - start.y, end.x - start.x])
    obj_msg.size.y = 0.2
    obj_msg.size.z = 0.2
    obj_pub.publish(obj_msg)
    print(obj_msg)


def pipeline_cb(msg):
    global id, idex
    piple = msg.units

    for i, u in enumerate(piple):
        if i == 0:
            last_left = u.left
            last_right = u.right
            continue
        left = u.left
        draw_unit(id+idex, last_left, left)
        id = (id + 1) % 10
        right = u.right
        draw_unit(id+idex, last_right, right)
        id = (id + 1) % 10
        last_left = left
        last_right = right

if __name__ == '__main__':
    rospy.init_node('pipeline_vision', anonymous=True)
    param_id = rospy.get_param("~drone_id")
    param_type = int(rospy.get_param("~type"))
    drone_name = "drone_{}".format(param_id)

    idex = 100*param_id
    id = 0

    obj_pub = rospy.Publisher("ue4_ros/obj", Obj, queue_size=10)
    rospy.Subscriber("{}/decision_info".format(drone_name), Pipeline, pipeline_cb)
    rospy.spin()