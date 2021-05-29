#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Date    : 2020-02-02 21:01:46
# @Author  : BrightSoul (653538096@qq.com)


import os
import time
from pymavlink import mavutil
import serial.tools.list_ports



if __name__ == '__main__':
    infoL = []

    port_list = serial.tools.list_ports.comports()
    for i in port_list:
        print("---"*10)
        print(f"start reboot on {i}")
        the_connection = mavutil.mavlink_connection(i.name,baud=115200)
        # 返回一个HB
        haveHB = the_connection.wait_heartbeat(timeout=2)
        if not haveHB:
            print(f"{i} is not a px4")
            continue
        
        print("Heartbeat from system (system %u component %u)"% 
            (the_connection.target_system, the_connection.target_component))

        the_connection.mav.command_long_send(
            the_connection.target_system, the_connection.target_component,
            mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN , 0,
            1, 0, 0, 0, 0, 0, 0 )
        while True:
            # 接arducopter_3.6.9_Fuck模式加了控制接口收数据:
            msg = the_connection.recv_match(blocking=True)
                
            if msg.get_type() == "COMMAND_ACK":
                print("command {} is {}".format(msg.command, msg.result))
                break
        print(f"{i} reboot successful")

        infoL.append({"com":i.name,"sys_id":the_connection.target_system})


    print("all ports reboot successful, wait 5s to restart")
    print("---"*10)

    for i in infoL:
        print("{sys_id} at {com}".format(**i))
    time.sleep(5)
