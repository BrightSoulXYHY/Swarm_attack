# -*- coding: utf-8 -*-
# import required libraries
import numpy as np
import cv2
import time
import threading

# import RflySim APIs
import PX4MavCtrlV4 as PX4MavCtrl
import ScreenCapApiV4 as sca


import socket
import struct
import sys

udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

'''

'''
max_mav = 10
ipList = [
    f"192.168.111.{i+20+1}" for i in range(max_mav)
]
# width,height = 1280,720
width,height = 640,360

def img_send_thrd(imginfo,ip,port=9999):
    lastTime = time.time()
    while True:
        lastTime = lastTime + 1/30.0
        sleepTime = lastTime - time.time()
        if sleepTime > 0:
            time.sleep(sleepTime)
        else:
            lastTime = time.time()
        img = sca.getCVImg(imginfo)
        sendImgUDP(img,ip,port)


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





if __name__ == '__main__':
    ip_start = 1
    if len(sys.argv)>1:
        ip_start = int(sys.argv[1])
    ip_start -= 1
    print(f"start with id {ip_start+1} at {ipList[ip_start]}")


    mav = PX4MavCtrl.PX4MavCtrler()


    window_hwnds = sca.getWndHandls()

    wd_num = len(window_hwnds)


    for i in range(wd_num):
        mav.sendUE4Cmd(f'RflyChangeViewKeyCmd B {i+1+ip_start}'.encode(),i)
        time.sleep(0.5)
        mav.sendUE4Cmd(b'RflyChangeViewKeyCmd V 1',i)
        time.sleep(0.5)
        mav.sendUE4Cmd(f'r.setres {width}x{height}w'.encode(),i)    
        time.sleep(0.5)
    time.sleep(2)    


    for wd_id,hwd in enumerate(window_hwnds):
        if ip_start+wd_id > max_mav-1:
            break

        ImgInfo = sca.getHwndInfo(hwd)
        threading.Thread(target=img_send_thrd, args=(ImgInfo,ipList[ip_start+wd_id],)).start()

    print('Start Transfer Img')
