import socket
import threading
import time
from pymavlink import mavutil
from pymavlink.dialects.v20 import common as mavlink2
import struct

# PX4 main mode enumeration
class PX4_CUSTOM_MAIN_MODE:
    PX4_CUSTOM_MAIN_MODE_MANUAL = 1
    PX4_CUSTOM_MAIN_MODE_ALTCTL = 2
    PX4_CUSTOM_MAIN_MODE_POSCTL = 3
    PX4_CUSTOM_MAIN_MODE_AUTO = 4
    PX4_CUSTOM_MAIN_MODE_ACRO = 5
    PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6
    PX4_CUSTOM_MAIN_MODE_STABILIZED = 7
    PX4_CUSTOM_MAIN_MODE_RATTITUDE = 8
    PX4_CUSTOM_MAIN_MODE_SIMPLE = 9

# PX4 sub mode enumeration
class PX4_CUSTOM_SUB_MODE_AUTO:
    PX4_CUSTOM_SUB_MODE_AUTO_READY = 1
    PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF = 2
    PX4_CUSTOM_SUB_MODE_AUTO_LOITER = 3
    PX4_CUSTOM_SUB_MODE_AUTO_MISSION = 4
    PX4_CUSTOM_SUB_MODE_AUTO_RTL = 5
    PX4_CUSTOM_SUB_MODE_AUTO_LAND = 6
    PX4_CUSTOM_SUB_MODE_AUTO_RTGS = 7
    PX4_CUSTOM_SUB_MODE_AUTO_FOLLOW_TARGET = 8
    PX4_CUSTOM_SUB_MODE_AUTO_PRECLAND = 9

# define a class for MAVLink initialization
class fifo(object):
    def __init__(self):
        self.buf = []

    def write(self, data):
        self.buf += data
        return len(data)

    def read(self):
        return self.buf.pop(0)

# PX4 MAVLink listen and control API && RflySim3D control API
class PX4MavCtrler:

    # constructor function
    def __init__(self, port):
        self.f = fifo
        self.stopFlag = False
        self.mav0 = mavlink2.MAVLink(self.f)
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # Create socket
        self.ip = '127.0.0.1'  # IP address and port number to send data
        self.port = port
        self.uavAngEular = [0, 0, 0]
        self.uavAngRate = [0, 0, 0]
        self.uavPosNED = [0, 0, 0]
        self.uavVelNED = [0, 0, 0]
        self.EnList = [0,1,0,0,0,1]
        self.coordinate_frame = mavlink2.MAV_FRAME_BODY_NED
        self.pos=[0,0,0]
        self.vel = [0,0,0]
        self.acc = [0, 0, 0]
        self.yaw=0
        self.yawrate = 0
        self.isInOffboard = False
        self.isArmed = False
        self.hasSendDisableRTLRC = False
        print("Thread Started!")

    # Initialize MAVLink listen loop from CopterSim
    def InitMavLoop(self):
        self.the_connection = mavutil.mavlink_connection('udpin:0.0.0.0:'+str(self.port+1))    
        self.lastTime = 0            
        self.t1 = threading.Thread(target=self.getMavMsg, args=())
        self.t1.start()
        self.t2 = threading.Thread(target=self.OffboardSendMode, args=())
        self.startTime = time.time()
        self.lastTime2 = 0
        self.startTime2 = time.time()
        
    # saturation function
    def sat(self,inPwm,thres=1):
        outPwm= inPwm
        if inPwm>thres:
            outPwm = thres
        elif inPwm<-thres:
            outPwm = -thres
        return outPwm

    # send MAVLink command long message to Pixhawk (routed through CopterSim) 
    def SendMavCmdLong(self, command, param1=0, param2=0, param3=0, param4=0, param5=0, param6=0, param7=0):
        buf = self.mav0.command_long_encode(self.the_connection.target_system, self.the_connection.target_component,
                                            command, 0,
                                            param1, param2, param3, param4, param5, param6, param7).pack(self.mav0)
        self.udp_socket.sendto(buf, (self.ip, self.port))

    # send command to make Pixhawk enter Offboard mode
    def sendMavOffboardCmd(self,type_mask,coordinate_frame, x,  y,  z,  vx,  vy,  vz,  afx,  afy,  afz,  yaw, yaw_rate):
        time_boot_ms = int(time.time()*1000000)
        buf = self.mav0.set_position_target_local_ned_encode(time_boot_ms,self.the_connection.target_system,
                                                             self.the_connection.target_component,
                                                             coordinate_frame,type_mask,x,  y,  z,  vx,  vy,  vz,  afx,
                                                             afy,  afz,  yaw, yaw_rate).pack(self.mav0)
        self.udp_socket.sendto(buf, (self.ip, self.port))

    # set the control sigals for Offboard sending loop
    def sendMavOffboardAPI(self,EnList,coordinate_frame,pos,vel,acc,yaw,yawrate):
        enPos = EnList[0]
        enVel = EnList[1]
        enAcc = EnList[2]
        enForce = EnList[3]
        enYaw = EnList[4]
        EnYawrate= EnList[5]
        y=int(0)
        if not enPos:
            y = y | 7

        if not enVel:
            y = y | (7<<3)

        if not enAcc:
            y = y | (7<<6)

        if not enForce:
            y = y | (1<<9)

        if not enYaw:
            y = y | (1<<10)

        if not EnYawrate:
            y = y|(1<<11)
        type_mask = y
        time_boot_ms = (time.time()-self.startTime)*1000
        buf = self.mav0.set_position_target_local_ned_encode(int(time_boot_ms),self.the_connection.target_system,
                                                             self.the_connection.target_component,
                                                             coordinate_frame,type_mask,pos[0],  pos[1],  pos[2],
                                                             vel[0],  vel[1],  vel[2],  acc[0],
                                                             acc[1],  acc[2],  yaw, yawrate).pack(self.mav0)
        self.udp_socket.sendto(buf, (self.ip, self.port))

    # send velocity control signal in earth north-east-down (NED) frame to Pixhawk
    def SendVelNED(self,vx,vy,vz,yawrate):
        self.EnList = [0,1,0,0,0,1]
        self.coordinate_frame = mavlink2.MAV_FRAME_LOCAL_NED
        self.pos=[0,0,0]
        self.vel = [vx,vy,vz]
        self.acc = [0, 0, 0]
        self.yaw = 0
        self.yawrate = yawrate

    # send command to control the display style of RflySim3D
    def sendUE4Cmd(self,cmd,windowID=-1):
        if windowID<0:
            for i in range(5):
                buf = struct.pack("i52s",1234567890,cmd)
                self.udp_socket.sendto(buf, (self.ip, 20010+i))
        else:    
            buf = struct.pack("i52s",1234567890,cmd)
            self.udp_socket.sendto(buf, (self.ip, 20010+windowID))

    # send the position & angle information to RflySim3D to create a new 3D model or update the old model's states
    def sendUE4Pos(self,copterID,vehicleType,MotorRPMSMean,PosE,AngEuler,windowID=-1):
        if windowID<0:
            for i in range(5):
                buf = struct.pack("3i7f",1234567890,copterID,vehicleType,MotorRPMSMean,PosE[0],PosE[1],PosE[2],AngEuler[0],AngEuler[1],AngEuler[2])
                self.udp_socket.sendto(buf, (self.ip, 20010+i))
        else:    
            buf = struct.pack("3i7f",1234567890,copterID,vehicleType,MotorRPMSMean,PosE[0],PosE[1],PosE[2],AngEuler[0],AngEuler[1],AngEuler[2])
            self.udp_socket.sendto(buf, (self.ip, 20010+windowID)) 
                
    def sendUE4PosScale(self,copterID,vehicleType,MotorRPMSMean,PosE,AngEuler,Scale=[1,1,1],windowID=-1):
        if windowID<0:
            for i in range(5):
                buf = struct.pack("3i10f",1234567890,copterID,vehicleType,MotorRPMSMean,PosE[0],PosE[1],PosE[2],AngEuler[0],AngEuler[1],AngEuler[2],Scale[0],Scale[1],Scale[2])
                self.udp_socket.sendto(buf, (self.ip, 20010+i))
        else:    
            buf = struct.pack("3i10f",1234567890,copterID,vehicleType,MotorRPMSMean,PosE[0],PosE[1],PosE[2],AngEuler[0],AngEuler[1],AngEuler[2],Scale[0],Scale[1],Scale[2])
            self.udp_socket.sendto(buf, (self.ip, 20010+windowID))         
                
    # send velocity control signal in body front-right-down (FRD) frame    
    def SendVelFRD(self,vx,vy,vz,yawrate):
        self.EnList = [0,1,0,0,0,1]
        self.coordinate_frame = mavlink2.MAV_FRAME_BODY_NED
        self.pos=[0,0,0]
        self.vel = [vx,vy,vz]
        self.acc = [0, 0, 0]
        self.yaw = 0
        self.yawrate = yawrate

    # send target position in earth NED frame
    def SendPosNED(self,x,y,z,yaw):
        self.EnList = [1,0,0,0,1,0]
        self.coordinate_frame = mavlink2.MAV_FRAME_LOCAL_NED
        self.pos=[x,y,z]
        self.vel = [0,0,0]
        self.acc = [0, 0, 0]
        self.yawrate = 0
        self.yaw = yaw

    # initialize Offboard in Pixhawk and start sending data loop in Python
    def initOffboard(self):
        self.EnList = [0,1,0,0,0,1]
        self.coordinate_frame = mavlink2.MAV_FRAME_BODY_NED
        self.pos=[0,0,0]
        self.vel = [0,0,0]
        self.acc = [0, 0, 0]
        self.yaw=0
        self.yawrate = 0
        self.isInOffboard = True
        self.SendSetMode(PX4_CUSTOM_MAIN_MODE.PX4_CUSTOM_MAIN_MODE_AUTO,PX4_CUSTOM_SUB_MODE_AUTO.PX4_CUSTOM_SUB_MODE_AUTO_LOITER)
        time.sleep(0.5)
        self.t2.start()
        self.sendMavOffboardAPI(self.EnList, self.coordinate_frame, self.pos, self.vel, self.acc, self.yaw,
                                self.yawrate)
        self.sendMavOffboardAPI(self.EnList, self.coordinate_frame, self.pos, self.vel, self.acc, self.yaw,
                                self.yawrate)
        if not self.hasSendDisableRTLRC:
            self.sendMavSetParam('NAV_RCL_ACT'.encode(),0,mavlink2.MAV_PARAM_TYPE_INT32)
            self.sendMavSetParam('NAV_DLL_ACT'.encode(), 0, mavlink2.MAV_PARAM_TYPE_INT32)
            self.SendMavCmdLong(mavlink2.MAV_CMD_NAV_GUIDED_ENABLE, True)
            self.hasSendDisableRTLRC = True
        if not self.isArmed:
            self.SendMavCmdLong(mavlink2.MAV_CMD_COMPONENT_ARM_DISARM, 1)

    # stop Offboard mode
    def endOffboard(self):
        self.isInOffboard = False
        if self.hasSendDisableRTLRC:
            self.SendMavCmdLong(mavlink2.MAV_CMD_NAV_GUIDED_ENABLE, False)
            self.hasSendDisableRTLRC = False
        if self.isArmed:
            self.SendMavCmdLong(mavlink2.MAV_CMD_COMPONENT_ARM_DISARM, 0, 21196.0)
        self.t2.join()

    # send command pixhawk to modify its parameters
    def sendMavSetParam(self,param_id, param_value, param_type):
        buf = self.mav0.param_set_encode(self.the_connection.target_system,self.the_connection.target_component,param_id,param_value, param_type).pack(self.mav0)
        self.udp_socket.sendto(buf, (self.ip, self.port))

    # send hil_actuator_controls message to Pixhawk (for rfly_ctrl uORB message)
    def SendHILCtrlMsg(self,ctrls):
        controls = [1500,1500,1100,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500]
        for i in range(len(ctrls)): 
            if i<len(controls):
                controls[i]=ctrls[i]
        buf = self.mav0.hil_actuator_controls_encode(int(time.time()*1000000),controls,1,1).pack(self.mav0)
        self.udp_socket.sendto(buf, (self.ip, self.port))
        print("Msg Send.")

    # send debug_vect message to Pixhawk to update rfly_ctrl uORB message
    def SendHILCtrlMsg1(self):
        name = b'hello'
        buf = self.mav0.debug_vect_encode(name, int(time.time()*1000000), 1100, 1500, 1700).pack(self.mav0)
        self.udp_socket.sendto(buf, (self.ip, self.port))
        print("Msg1 Send.")

    # send MAVLink command to Pixhawk to Arm/Disarm the drone
    def SendMavArm(self, isArm):
        if (isArm):
            self.SendMavCmdLong(mavlink2.MAV_CMD_COMPONENT_ARM_DISARM, 1)
        else:
            self.SendMavCmdLong(mavlink2.MAV_CMD_COMPONENT_ARM_DISARM, 0, 21196.0)

    # send MAVLink rc_channels_override message to override the RC signals
    def SendRcOverride(self, ch1=1500, ch2=1500, ch3=1100, ch4=1500, ch5=1100, ch6=1100, ch7=1500, ch8=1500):
        buf = self.mav0.rc_channels_override_encode(self.the_connection.target_system,
                                                    self.the_connection.target_component, ch1, ch2,
                                                    ch3, ch4, ch5, ch6, ch7, ch8).pack(self.mav0)
        self.udp_socket.sendto(buf, (self.ip, self.port))

    # send MAVLink message manual_control to send normalized and calibrated RC sigals to pixhawk
    def sendMavManualCtrl(self, x,y,z,r):
        buf = self.mav0.manual_control_encode(self.the_connection.target_system, x,y,z,r,0).pack(self.mav0)
        self.udp_socket.sendto(buf, (self.ip, self.port))

    # send MAVLink command to change current flight mode
    def SendSetMode(self,mainmode,cusmode=0):
        basemode = mavlink2.MAV_MODE_FLAG_HIL_ENABLED | mavlink2.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        self.SendMavCmdLong(mavlink2.MAV_CMD_DO_SET_MODE, basemode, mainmode, cusmode)

    # Stop MAVLink data listening loop
    def stopRun(self):
        self.stopFlag=True
        time.sleep(0.5)
        self.t1.join()
        if(self.isInOffboard):
            self.endOffboard()
        self.the_connection.close()

    # Update Pixhawk states from MAVLink for 100Hz
    def getMavMsg(self):
        self.lastTime = time.time()
        while True:
            if self.stopFlag:
                break
            self.lastTime = self.lastTime + 0.01
            sleepTime = self.lastTime - time.time()
            if sleepTime > 0:
                time.sleep(sleepTime)
            else:
                self.lastTime = time.time()
            while True:
                if self.stopFlag:
                    break
                # message IDs to be listened
                msg = self.the_connection.recv_match(
                    type=['ATTITUDE', 'LOCAL_POSITION_NED','HEARTBEAT'],
                    blocking=False)
                if msg is not None:
                    if msg.get_type() == "ATTITUDE":
                        self.uavAngEular[0] = msg.roll
                        self.uavAngEular[1] = msg.pitch
                        self.uavAngEular[2] = msg.yaw
                        self.uavAngRate[0] = msg.rollspeed
                        self.uavAngRate[1] = msg.pitchspeed
                        self.uavAngRate[2] = msg.yawspeed
                    if msg.get_type() == "LOCAL_POSITION_NED":
                        self.uavPosNED[0] = msg.x
                        self.uavPosNED[1] = msg.y
                        self.uavPosNED[2] = msg.z
                        self.uavVelNED[0] = msg.vx
                        self.uavVelNED[1] = msg.vy
                        self.uavVelNED[2] = msg.vz
                    if msg.get_type() == "HEARTBEAT":
                        isArmed = msg.base_mode & mavlink2.MAV_MODE_FLAG_SAFETY_ARMED
                        if not self.isArmed and isArmed:
                            print("PX4 Armed!")
                        if self.isArmed and not isArmed:
                            print("PX4 DisArmed!")
                        self.isArmed = isArmed
                else:
                    break
        print("Mavlink Stoped.")

    # Offboard message sending loop, 100Hz
    def OffboardSendMode(self):
        self.lastTime2 = time.time()
        while True:
            if not self.isInOffboard:
                break
            self.lastTime2 = self.lastTime2 + 0.01
            sleepTime = self.lastTime2 - time.time()
            if sleepTime > 0:
                time.sleep(sleepTime)
            else:
                self.lastTime2 = time.time()
            if self.isInOffboard:
                self.sendMavOffboardAPI(self.EnList, self.coordinate_frame, self.pos, self.vel, self.acc, self.yaw, self.yawrate)
        print("Offboard Stoped.")