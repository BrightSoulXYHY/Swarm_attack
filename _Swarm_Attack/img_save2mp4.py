# -*- coding: utf-8 -*-
# import required libraries
import numpy as np
import cv2
import time
import threading

# import RflySim APIs
import PX4MavCtrlV4 as PX4MavCtrl
import ScreenCapApiV4 as sca



import sys
import os


'''

'''
max_mav = 10
# width,height = 1280,720
width,height = 640,360
fps = 30

    
fileName = time.strftime("%Y%m%d_%H%M%S", time.localtime())

SAVING_RUN = True


def img_save_thrd(imginfo,name):
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    video = cv2.VideoWriter(f"{fileName}/{name}.mp4", fourcc, fps, (width,height))
    
    
    lastTime = time.time()
    while SAVING_RUN:
        lastTime = lastTime + 1/fps
        sleepTime = lastTime - time.time()
        if sleepTime > 0:
            time.sleep(sleepTime)
        else:
            lastTime = time.time()
        img = sca.getCVImg(imginfo)
        video.write(img)
    video.release()
    print(f"{name} save done")
        # sendImgUDP(img,ip,port)





if __name__ == '__main__':
    if not os.path.exists(fileName):
        os.mkdir(fileName) 

    ip_start = 1
    if len(sys.argv)>1:
        ip_start = int(sys.argv[1])
    ip_start -= 1
    


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
        # threading.Thread(target=img_send_thrd, args=(ImgInfo,HOST_IP,portList[ip_start+wd_id],)).start()
        threading.Thread(target=img_save_thrd, args=(ImgInfo,"mav-{:02d}".format(ip_start+wd_id+1),)).start()

    print('Start Transfer Img')
    input("enter to break")


    SAVING_RUN = False
    time.sleep(4)
