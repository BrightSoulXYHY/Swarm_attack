#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Date    : 2021-05-28 22:16:39
# @Author  : BrightSoul (653538096@qq.com)


import os
import subprocess
import time
from multiprocessing import Process,Lock
import threading
import ctypes,sys

STD_INPUT_HANDLE = -10
STD_OUTPUT_HANDLE = -11
STD_ERROR_HANDLE = -12
 
#字体颜色定义 text colors
FOREGROUND_BLUE = 0x09 # blue.
FOREGROUND_GREEN = 0x0a # green.
FOREGROUND_RED = 0x0c # red.
FOREGROUND_YELLOW = 0x0e # yellow.
 
# 背景颜色定义 background colors
BACKGROUND_YELLOW = 0xe0 # yellow.
 
# get handle
std_out_handle = ctypes.windll.kernel32.GetStdHandle(STD_OUTPUT_HANDLE)

def set_cmd_text_color(color, handle=std_out_handle):
    Bool = ctypes.windll.kernel32.SetConsoleTextAttribute(handle, color)
    return Bool


def priint_color(mess,color,endl='\n'):
    set_cmd_text_color(color)
    sys.stdout.write(mess + endl)
    resetColor()

def printBlue(mess,endl='\n'):
    set_cmd_text_color(FOREGROUND_BLUE)
    sys.stdout.write(mess + endl)
    resetColor()

def printYellow(mess,endl='\n'):
    set_cmd_text_color(FOREGROUND_YELLOW)
    sys.stdout.write(mess + endl)
    resetColor()

def printRed(mess,endl='\n'):
    set_cmd_text_color(FOREGROUND_RED)
    sys.stdout.write(mess + endl)
    resetColor()


def printGreen(mess,endl='\n'):
    set_cmd_text_color(FOREGROUND_GREEN)
    sys.stdout.write(mess + endl)
    resetColor()

def resetColor():
    set_cmd_text_color(FOREGROUND_RED | FOREGROUND_GREEN | FOREGROUND_BLUE)



'''
实现一个自动打包工作空间并上传解压编译的脚本

'''
start_time = time.time()
# SRC_DIR="_release"
SRC_DIR="src"
runme = f"wsl tar -cpf {SRC_DIR}.tar --exclude=.git --exclude=upload.bat {SRC_DIR}"
lock=threading.Lock()


path = "~/Swarm_ws"
user = "nvidia"
cmdL = [
    {
        "info":"",
        "cmd":"ssh {user}@192.168.111.{id} mkdir -p {path}/dd;rm -rf {path}/*;",
    },
    {
        "info":"start upload",
        # cmdL[1]["cmd"]
        "cmd":"scp {src}.tar {user}@192.168.111.{id}:{path};",
    },    
    {
        "info":"start build",
        # cmdL[2]["cmd"]
        "cmd":"ssh {user}@192.168.111.{id} "
            "cd {path};"
            "tar -xpf {src}.tar;"
            # "mv {src} src;"
            # "source ~/.${{SHELL:5}}rc;"
            # "source /opt/ros/melodic/setup.${{SHELL:5}};"
            # "catkin_make",
    },

]

# cmdL = [
#     {
#         "info":"",
#         "cmd":"ssh q@192.168.111.{id} mkdir ~/Swarm_ws/dd;rm -rf ~/Swarm_ws/*;",
#     },
#     {
#         "info":"",
#         # cmdL[1]["cmd"]
#         "cmd":"scp {src}.tar q@192.168.111.{id}:~/Swarm_ws;",
#     },    
#     {
#         "info":"start build",
#         # cmdL[2]["cmd"]
#         "cmd":"ssh q@192.168.111.{id} "
#             "cd ~/Swarm_ws;"
#             "tar -xpf {src}.tar;"
#             # "mv {src} src;"
#             # "source ~/.${{SHELL:5}}rc;"
#             # "source /opt/ros/melodic/setup.${{SHELL:5}};"
#             # "catkin_make",
#     },

# ]


# cmddL = [
#     "ssh q@192.168.111.{id} "
#     "echo ${{SHELL:5}}",

#     "ssh q@192.168.111.{id} "
#     "cd ~/{src};"
#     "source /opt/ros/melodic/setup.${{SHELL:5}};"
# ]

def upload_single(id):
    for i in cmdL:
        cmd = i["cmd"].format(id=id,src=SRC_DIR,path=path,user=user)
        if i["info"]:
            lock.acquire()
            print(
                "[{:.2f}]".format(time.time()-start_time),
                "{}@{}".format(i["info"],id)
                ) 
            lock.release()
        res = subprocess.run(cmd,capture_output=True)
        

        if res.returncode:
            lock.acquire()
            printRed("[{:.2f}]".format(time.time()-start_time)+f"error@{id}")
            printRed("len(res.stderr):{} res.returncode:{}".format(len(res.stderr),res.returncode))
            printRed("error cmd:",cmd)
            printRed("error info: \n{}".format(res.stderr.decode()))
            lock.release()
            return
        elif  len(res.stderr):
            lock.acquire()
            printYellow("[{:.2f}]".format(time.time()-start_time)+f"warning@{id}")
            printYellow("len(res.stderr):{} res.returncode:{}".format(len(res.stderr),res.returncode))
            # printYellow("warning cmd:",cmd)
            # printYellow("warning info: \n{}".format(res.stderr.decode()))
            lock.release()



def main():

    print("[{:.2f}]".format(time.time()-start_time)+"start multi upload")


    subprocess.run(runme)
    print(f"pack {SRC_DIR} done")
    
    upload_single(142)
    # upload_single(137)

    # for i in range(6):
    #     upload_single(11+i)

    # 对小文件而言省的时间差不多
    # 大文件的主要限制是带宽，也省不了多少
    # process = [Process(target=upload_single,args=(11+i,)) for i in range(6)]
    # for p in process:
    #     p.start()
    # for p in process:
    #     p.join()



if __name__ == '__main__':
    main()