# -*- coding: utf-8 -*-
import socket
import cv2
import numpy as np
import struct
import sys
import RflyVisionAPI
import os
import win32gui, win32ui, win32con, win32api
from ctypes import windll
import time
import threading
import math
import ScreenCapApiV4 as sca


#下面的127.0.0.1要改为目标计算机IP
mav = RflyVisionAPI.RflyVisionAPI('192.168.199.238',9999)

# sending image thread function
def img_send_thrd(imginfo,mavl):
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
        mavl.sendImgUDP(img)


mav.sendUE4Cmd(b'RflyChangeMapbyName OldFactory',0)
# mav.sendUE4Cmd(b'RflyChangeMapbyName OldFactory',1)
time.sleep(0.5)    
# create a ball, set its position and altitude, use the default red color
mav.sendUE4Pos(100,152,0,[-177,10,-2],[0,0,0])
time.sleep(0.5)
mav.sendUE4Cmd(b'RflyChangeViewKeyCmd V 1',0)
time.sleep(0.5)
mav.sendUE4Cmd(b'RflyCameraPosAng 0.3 0 0 0 0 0',0)


"""
# Get handles of all UE4/RflySim3D windows
window_hwnds = sca.getWndHandls()

mav.sendUE4Cmd(b'r.setres 720x405w',0)    
time.sleep(2)    

# find the window with length 720 as the first window
Wd01 = window_hwnds[0]
hasFoundWd = False
for hwd in window_hwnds:
    info = sca.getHwndInfo(hwd)
    if info.width == 720:
        Wd01 = hwd
        hasFoundWd = True
        window_hwnds.remove(hwd)
        break

# if no window is found, throw an error
if not hasFoundWd:
    print("The first RflySim3D window does not response the command, please close all apps and try again.")
    mav.stopRun()
    sys.exit(1)    
else:
    print("The first RflySim3D window is found with desired size.")
    

ImgInfo1 = sca.getHwndInfo(Wd01)

t_recimg1 = threading.Thread(target=img_send_thrd, args=(ImgInfo1,mav,))
t_recimg1.start()

print('Start Transfer Img')

"""