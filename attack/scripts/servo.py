#!/usr/bin/env python
#!coding=utf-8

import rospy
import numpy as np
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cv2

# parameters
width = 720
height = 405
min_prop = 0.000001


def calc_centroid(img):
    """Get the centroid and area of green in the image"""

    hue_image = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    low_range = np.array([175,200,20])
    high_range = np.array([180,256,250])
    th = cv2.inRange(hue_image, low_range, high_range)
    cv2.imshow("th", th)
    cv2.waitKey(1)

    M = cv2.moments(th, binaryImage=True)
    if M["m00"] >= min_prop*width*height:
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        return [cx, cy, int(M["m00"])]
    else:
        return [-1, -1, -1]


def callback(msg):
    global case, pos_i
    if case != "task_attack":
        return

    np_arr = np.fromstring(msg.data, np.uint8)
    image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    cv2.imshow("frame" , image)
    cv2.waitKey(1)
    image = cv2.resize(image, (width, height))
    pos_i[0], pos_i[1], pos_i[2] = calc_centroid(image)

def arm():
    if armService(True):
        return True
    else:
        print("Vehicle arming failed!")
        return False

def offboard():
    if flightModeService(custom_mode='OFFBOARD'):
        return True
    else:
        print("Vechile Offboard failed")
        return False

if __name__ == "__main__":
    from mavros_msgs.msg import State
    from mavros_msgs.srv import CommandBool, SetMode
    from geometry_msgs.msg import PoseStamped, TwistStamped
    from sensor_msgs.msg import Imu, NavSatFix

    global case, pos_i
    case = "task_attack"
    pos_i = np.array([-1., -1., -1.])
    command = TwistStamped()
    rospy.init_node('attack_node', anonymous=True)

    # make a video_object and init the video object
    rospy.Subscriber('/camera/left/compressed', CompressedImage, callback)
    vel_pub = rospy.Publisher('mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
    armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
    flightModeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)
    rate = rospy.Rate(20)

    for i in range(10):
        vel_pub.publish(command)
        arm_state = arm()
        offboard_state = offboard()
        rate.sleep()


    while arm_state and offboard_state and (rospy.is_shutdown() is False):
        if pos_i[2] >= 0:
            command.twist.angular.z = 0.001*(width/2-pos_i[0])
            command.twist.linear.z = 0.0001*(height/2-pos_i[1])
        else:
            command.twist.angular.z = 0
            command.twist.linear.z = 0
        vel_pub.publish(command)
        rate.sleep()
    rospy.spin()