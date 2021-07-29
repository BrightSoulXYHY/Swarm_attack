# -*- coding: utf-8 -*-

import socket
import struct
import math

def send(buf):
    for i in range(5):
        udp_socket.sendto(buf, ("127.0.0.1", 20010+i))

udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


# 人是30?
copterID,vehicleType,MotorRPMSMean=101,30,0
PosE,AngEuler = [-177,10,-0.2],[0,0,math.pi/2]
Scale=[1,1,1]
buf = struct.pack("3i10f",1234567890,copterID,vehicleType,MotorRPMSMean,*PosE,*AngEuler,*Scale)
send(buf)

# 球是152
copterID,vehicleType,MotorRPMSMean=100,152,0
PosE,AngEuler = [-177,0,-2],[0,0,math.pi/2]
Scale=[.1,.1,.1]
buf = struct.pack("3i10f",1234567890,copterID,vehicleType,MotorRPMSMean,*PosE,*AngEuler,*Scale)
send(buf)


