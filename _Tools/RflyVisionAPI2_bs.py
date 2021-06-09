import socket
import threading
import time
import cv2
import numpy as np
import struct

class RflyVisionAPI:

    # 参数ip和port是发送给的一个IP

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
        # self.t_recimg0 = threading.Thread(target=self.img_rec_thrd, args=())
        # self.t_recimg0.start()

    def sendUE4Cmd(self,cmd,windowID=-1):
        if windowID<0:
            for i in range(5):
                buf = struct.pack("i52s",1234567890,cmd)
                self.udp_socket.sendto(buf, ('127.0.0.1', 20010+i))
        else:    
            buf = struct.pack("i52s",1234567890,cmd)
            self.udp_socket.sendto(buf, ('127.0.0.1', 20010+windowID))
    
    # def display(s):
    #     b = bytearray(s)
    #     for i in b:
    #         print "%02x" % i,
    #     print

    def sendUE4Pos(self,copterID,vehicleType,MotorRPMSMean,PosE,AngEuler,windowID=-1):
        if windowID<0:
            for i in range(5):
                buf = struct.pack("3i7f",1234567890,copterID,vehicleType,MotorRPMSMean,*PosE,*AngEuler)
                self.udp_socket.sendto(buf, ('127.0.0.1', 20010+i))
        else:    
            buf = struct.pack("3i7f",1234567890,copterID,vehicleType,MotorRPMSMean,PosE[0],PosE[1],PosE[2],
                              AngEuler[0],AngEuler[1],AngEuler[2])
            self.udp_socket.sendto(buf, ('127.0.0.1', 20010+windowID))

    def sendUE4PosScale(self,copterID,vehicleType,MotorRPMSMean,PosE,AngEuler,Scale=[1,1,1],windowID=-1):
        if windowID<0:
            for i in range(5):
                buf = struct.pack("3i10f",1234567890,copterID,vehicleType,MotorRPMSMean,PosE[0],PosE[1],PosE[2],AngEuler[0],AngEuler[1],AngEuler[2],Scale[0],Scale[1],Scale[2])
                self.udp_socket.sendto(buf, (self.ip, 20010+i))
        else:    
            buf = struct.pack("3i10f",1234567890,copterID,vehicleType,MotorRPMSMean,PosE[0],PosE[1],PosE[2],AngEuler[0],AngEuler[1],AngEuler[2],Scale[0],Scale[1],Scale[2])
            self.udp_socket.sendto(buf, (self.ip, 20010+windowID))         
               

    def sendUE4Obj(self,copterID,vehicleType,PosE,AngEuler,AngQuatern,windowID=-1):
        MotorRPMS = [0]*8
        VelE = [0]*3
        AccB = [0]*3
        RateB = [0]*3
        PosGPS = [0]*3
        runnedTime = 10
        if windowID<0:
            for i in range(5):
                buf = struct.pack("2id27f3d",copterID,vehicleType,runnedTime,*VelE,*PosE,*AngEuler,*AngQuatern,*MotorRPMS,*AccB,*RateB,*PosGPS)
                self.udp_socket.sendto(buf, ('127.0.0.1', 20010+i))
        else:    
            buf = struct.pack("2id27f3d",copterID,vehicleType,runnedTime,*VelE,*PosE,*AngEuler,*AngQuatern,*MotorRPMS,*AccB,*RateB,*PosGPS)
            # buf = struct.pack("3i11f",1234567890,copterID,vehicleType,runnedTime,PosE[0],PosE[1],PosE[2],
            #                   AngEuler[0],AngEuler[1],AngEuler[2],AngQuatern[0],AngQuatern[1],AngQuatern[2],AngQuatern[3])
            self.udp_socket.sendto(buf, ('127.0.0.1', 20010+windowID)) 

    def sendUE4_Objsend(self,copterID,vehicleType,PostionE,Angle,size,RPMMean,windowID=-1):
        IDs = [0]*100
        Types = [0]*100
        PosE = [0]*300
        AngEuler = [0]*300
        ScaleXYZ = [0]*300
        mean = [0]*100
        IDs[0] = copterID
        Types[0] = vehicleType
        PosE[0],PosE[1],PosE[2] = PostionE[0],PostionE[1],PostionE[2]
        AngEuler[0],AngEuler[1],AngEuler[2] = Angle[0],Angle[1],Angle[2]
        ScaleXYZ[0],ScaleXYZ[1],ScaleXYZ[2] = size[0],size[1],size[2]
        mean[0] = RPMMean
        print("pose is {}".format(PosE))
        print("Types is {}".format(Types))
        if windowID<0:
            for i in range(5):
                buf = struct.pack("200i1000f",*IDs,*Types,*PosE,*AngEuler,*ScaleXYZ,*mean)
                self.udp_socket.sendto(buf, ('127.0.0.1', 20010+i))
        else:    
            buf = struct.pack("200i1000f",*IDs,*Types,*PosE,*AngEuler,*ScaleXYZ,*mean)
            # buf = struct.pack("3i11f",1234567890,copterID,vehicleType,runnedTime,PosE[0],PosE[1],PosE[2],
            #                   AngEuler[0],AngEuler[1],AngEuler[2],AngQuatern[0],AngQuatern[1],AngQuatern[2],AngQuatern[3])
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