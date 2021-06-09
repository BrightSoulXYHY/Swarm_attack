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

udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


def sendImgUDP(img,ip,port):
    img_encode = cv2.imencode('.jpg', img)[1]
    data_encode = np.array(img_encode)
    data = data_encode.tostring()
    #定义文件头，打包成结构体
    imgFlag = 1234567890
    imgLen = len(data)
    imgPackUnit = 60000
    imgpackNum = imgLen//imgPackUnit+1
    fhead = struct.pack('4i',imgFlag,imgLen,imgPackUnit,imgpackNum)
    # 发送文件头:
    udp_socket.sendto(fhead,(ip, port))
    #循环发送图片码流
    for i in range(imgpackNum):
        if imgPackUnit*(i+1)>len(data):
            udp_socket.sendto(data[imgPackUnit*i:], (ip, port))
        else:
            udp_socket.sendto(data[imgPackUnit*i:imgPackUnit*(i+1)], (ip, port))

# sending image thread function
def img_send_thrd(imginfo,ip,port):
    # print(imginfo.title,ip,port)
    # print("Satrt Pub Img")
    lastTime = time.time()
    while True:
        lastTime = lastTime + 1/30.0
        sleepTime = lastTime - time.time()
        if sleepTime > 0:
            time.sleep(sleepTime)
        else:
            lastTime = time.time()
        #以下代码以30Hz运行
        img=sca.getCVImg(imginfo)
        sendImgUDP(img,ip,port)

if __name__ == '__main__':
    mav_num = 2
    
    # 1280x720为基准，否则会有黑边
    # width,height = 640,360
    width,height = 1280,720
    
    client_ip = "127.0.0.1"
    mav = RflyVisionAPI2.RflyVisionAPI(client_ip,9999)

    
    # 设置Rflysim的参数和属性
    for i in range(mav_num):
        mav.sendUE4Cmd(f'r.setres {width}x{height}w'.encode(),i)
        time.sleep(1) 
        mav.sendUE4Cmd('RflyChangeViewKeyCmd B {}'.format(i+1).encode(),i)
        time.sleep(0.1) 
        mav.sendUE4Cmd(b'RflyChangeViewKeyCmd V 1',i)
        time.sleep(0.1) 



    # Get handles of all UE4/RflySim3D windows
    window_hwnds = sca.getWndHandls()
    time.sleep(2)

    socketL = []
    infoL = []
    ipL = ["192.168.111.142","192.168.111.137"]

    # 根据窗口的ID确定创建发送图片的Socket
    # for hwd in window_hwnds:
    for i in range(mav_num):
        hwd = window_hwnds[i]
        info = sca.getHwndInfo(hwd)
        infoL.append(info)
        wdID = int(info.title.split("-")[-1])
        # socket2nuc = (f"192.168.111.{10+wdID+1}",9999)
        socket2nuc = (ipL[wdID],9999)
        socketL.append(socket2nuc)
        
    for i in range(mav_num):
        threading.Thread(target=img_send_thrd, args=(infoL[i],*socketL[i],)).start()
    print('Start Transfer Img')

