import socket
import threading
import time
import cv2
import numpy as np
import struct

class RflyVisionAPI:

    def __init__(self, ip, port):
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # 创建套接字
        if ip == '255.255.255.255':
            self.udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)        
        self.ip = ip
        self.port = port
        self.img = 1
        self.hasData=False

    def initImgServ(self):
        self.udp_socket.setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,60000*10)
        self.udp_socket.bind(('0.0.0.0', self.port))
        self.t_recimg0 = threading.Thread(target=self.img_rec_thrd, args=())
        self.t_recimg0.start()

    def sendUE4Cmd(self,cmd,windowID=-1):
        if windowID<0:
            for i in range(5):
                buf = struct.pack("i52s",1234567890,cmd)
                self.udp_socket.sendto(buf, ('127.0.0.1', 20010+i))
        else:    
            buf = struct.pack("i52s",1234567890,cmd)
            self.udp_socket.sendto(buf, ('127.0.0.1', 20010+windowID))

    def sendUE4Pos(self,copterID,vehicleType,MotorRPMSMean,PosE,AngEuler,windowID=-1):
        if windowID<0:
            for i in range(5):
                buf = struct.pack("3i7f",1234567890,copterID,vehicleType,MotorRPMSMean,PosE[0],PosE[1],PosE[2],AngEuler[0],AngEuler[1],AngEuler[2])
                self.udp_socket.sendto(buf, ('127.0.0.1', 20010+i))
        else:    
            buf = struct.pack("3i7f",1234567890,copterID,vehicleType,MotorRPMSMean,PosE[0],PosE[1],PosE[2],AngEuler[0],AngEuler[1],AngEuler[2])
            self.udp_socket.sendto(buf, ('127.0.0.1', 20010+windowID)) 

    def sendImgUDP(self,img):
        img_encode = cv2.imencode('.jpg', img)[1]
        data_encode = np.array(img_encode)
        data = data_encode.tostring()
        #定义文件头，打包成结构体
        imgFlag = 1234567890
        imgLen = len(data)
        imgPackUnit = 60000
        imgpackNum = imgLen//imgPackUnit+1
        fhead = struct.pack('4i',imgFlag,imgLen,imgPackUnit,imgpackNum)
        #print((imgFlag,imgLen,imgPackUnit,imgpackNum))
        # 发送文件头:
        self.udp_socket.sendto(fhead,(self.ip, self.port))
        #循环发送图片码流
        for i in range(imgpackNum):
            if imgPackUnit*(i+1)>len(data):
                self.udp_socket.sendto(data[imgPackUnit*i:], (self.ip, self.port))
            else:
                self.udp_socket.sendto(data[imgPackUnit*i:imgPackUnit*(i+1)], (self.ip, self.port))

    def img_rec_thrd(self):
        imgFlag = 1234567890
        fhead_size = struct.calcsize('4i')
        StartFlag = False
        imgPackUnit = 60000
        recvd_size = 0
        data_total = b''
        while True:
            buf,addr = self.udp_socket.recvfrom(imgPackUnit)
            if len(buf)==fhead_size:
                dd = struct.unpack('4i',buf)
                chksm = dd[0]
                if chksm==imgFlag:
                    StartFlag=True
                    data_size = dd[1]
                    imgPackUnit= dd[2]
                    recvd_size = 0
                    data_total = b''
                    continue

            if StartFlag:
                if data_size -recvd_size >imgPackUnit:
                    recvd_size += len(buf)
                else:
                    recvd_size = data_size  
                data_total += buf
                    
                if recvd_size == data_size:
                    nparr = np.frombuffer(data_total, np.uint8)
                    self.img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                    self.hasData = True
                    StartFlag=False
                
                if recvd_size>data_size:
                    print('Wrong Data.')
                    StartFlag=False