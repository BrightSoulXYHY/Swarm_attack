# -*- coding: utf-8 -*-
# import required libraries
import numpy as np
import cv2
import time
import threading

# import RflySim APIs
import PX4MavCtrlV4 as PX4MavCtrl
import ScreenCapApiV4 as sca

'''
新版本的vision多了个排序
添加一个窗口向对应的NX发送指令
截取窗口通过opencv查看
'''

def show_img(imginfo,wd_name):
    lastTime = time.time()
    while True:
        lastTime = lastTime + 1/30.0
        sleepTime = lastTime - time.time()
        if sleepTime > 0:
            time.sleep(sleepTime)
        else:
            lastTime = time.time()
        img = sca.getCVImg(imginfo)
        cv2.imshow(wd_name, img)
        cv2.waitKey(1)






if __name__ == '__main__':
    mav = PX4MavCtrl.PX4MavCtrler()
    width,height = 1280,720

    window_hwnds = sca.getWndHandls()

    wd_num = len(window_hwnds)


    for i in range(wd_num):
        mav.sendUE4Cmd(b'RflyChangeViewKeyCmd V 1',i)
        time.sleep(0.5)
        mav.sendUE4Cmd(f'r.setres {width}x{height}w'.encode(),i)    
        time.sleep(0.5)
    time.sleep(2)    


    # ImgInfoList = []

    

    for wd_id,hwd in enumerate(window_hwnds):
        ImgInfo = sca.getHwndInfo(hwd)

        threading.Thread(target=show_img, args=(ImgInfo,f"mav-{wd_id}",)).start()

    print('Start Transfer Img')
