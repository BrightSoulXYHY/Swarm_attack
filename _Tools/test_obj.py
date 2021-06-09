# -*- coding: utf-8 -*-
import sys
import socket
import cv2
import numpy as np
import struct
import sys
import os
import win32gui, win32ui, win32con, win32api
from ctypes import windll
import time
import threading
import math

# 同目录下的文件
import RflyVisionAPI2_bs as RflyVisionAPI2
import ScreenCapApiV4_bs as sca


# #下面的127.0.0.1要改为目标计算机IP
# mav = RflyVisionAPI2.RflyVisionAPI('192.168.111.5',9999)

# # 人是30?
# mav.sendUE4Pos(100,30,0,[0,30,0],[0,0,math.pi])

udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

 # 球是152
# copterID,vehicleType,MotorRPMSMean=100,152,0
# PosE,AngEuler = [-177,0,-2],[0,0,math.pi/2]
# Scale=[.1,.1,.1]
# 
copterID,vehicleType,MotorRPMSMean=101,30,0
PosE,AngEuler = [-177,10,-0.2],[0,0,math.pi/2]
Scale=[1,1,1]


buf = struct.pack("3i10f",1234567890,copterID,vehicleType,MotorRPMSMean,*PosE,*AngEuler,*Scale)
udp_socket.sendto(buf, ("192.168.111.4", 20010))