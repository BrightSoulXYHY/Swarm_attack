#!/usr/bin/env python
# coding=utf-8

import rospy
import numpy as np
from swarm_msgs.msg import Pipeline, Pipeunit
from geometry_msgs.msg import PoseStamped, TwistStamped, Point32
from rflysim_ros_pkg.msg import Obj

idex = 100
id = 0


def draw_unit(id, start, end):
    global obj_pub
    obj_msg = Obj()
    obj_msg.id = id
    obj_msg.type = 27
    obj_msg.position.x = (start.x + end.x) / 2
    obj_msg.position.y = (start.y + end.y) / 2
    obj_msg.position.z = start.z
    obj_msg.angule.z = np.arctan2(end.y - start.y, end.x - start.x)
    obj_msg.size.x = np.linalg.norm([end.y - start.y, end.x - start.x])
    obj_msg.size.y = 0.1
    obj_msg.size.z = 2
    obj_pub.publish(obj_msg)
    print(obj_msg)


def pipeline_cb(msg):
    global id
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
    obj_pub = rospy.Publisher("ue4_ros/obj", Obj, queue_size=10)
    rospy.Subscriber("decision_info", Pipeline, pipeline_cb)
    rospy.spin()