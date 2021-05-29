#!/usr/bin/env python
# coding=utf-8

import rospy
import numpy as np
from swarm_msgs.msg import Pipeline, Pipeunit
from geometry_msgs.msg import Point32

class Decision:
    def __init__(self):
        rospy.init_node('decision_node', anonymous=True)
        self.pipe_pub = rospy.Publisher('decision_info', Pipeline , queue_size=10)
        self.rate = rospy.Rate(1)

    def worker(self):
        a = Pipeline()
        a.pipetype.data = 10
        u = Pipeunit()
        u.middle = Point32(10,20,30)
        u.left = Point32(0,20,30)
        u.right = Point32(20,20,30)
        while not rospy.is_shutdown():
            a.units.append(u)
            print(a)
            self.pipe_pub.publish(a)
            self.rate.sleep()


if __name__ == '__main__':
    dd = Decision()
    dd.worker()