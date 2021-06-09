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


# sending image thread function
def img_send_thrd(imginfo,mav):
    print("Satrt Pub Img")
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
        mav.sendImgUDP(img)

if __name__ == '__main__':
    wdID = 0
    width,height = 640,480
    client_ip = "127.0.0.1"
    if len(sys.argv)>1:
        client_ip = sys.argv[1]



    print(f"Connect to {client_ip}")
    mav = RflyVisionAPI2.RflyVisionAPI(client_ip,9999)


    mav.sendUE4Cmd(b'RflyChangeViewKeyCmd V 1',0)
    time.sleep(0.5)
    mav.sendUE4Cmd(b'RflyCameraPosAng 0.3 0 0 0 0 0',0) 
    time.sleep(0.5)
 

    # Get handles of all UE4/RflySim3D windows
    window_hwnds = sca.getWndHandls()

    mav.sendUE4Cmd(f'r.setres {width}x{height}w'.encode(),0)    
    time.sleep(2)    


    for hwd in window_hwnds:
        info = sca.getHwndInfo(hwd)
        if int(info.title.split("-")[-1]) == wdID:
            ImgInfo = info
            break

    t_recimg1 = threading.Thread(target=img_send_thrd, args=(ImgInfo,mav,))
    t_recimg1.start()

    print('Start Transfer Img')
