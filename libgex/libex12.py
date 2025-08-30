import time
from .dynamixel_sdk import PortHandler, PacketHandler
from .motor import Motor
import sys
import numpy as np
from .config import BAUDRATE, PROTOCOL_VERSION
from .ex12 import kinematics

class Glove:
    def __init__(self, port, vis=False) -> None:
        self.is_connected = False
        self.port = port
        self.name = 'EX12'
        self.kin = kinematics.KinEX12(vis)

    def connect(self, goal_pwm=300, init=True):
        """
        连接Hand，并且使能每个电机为默认的力控位置模式
        """

        portHandler = PortHandler(self.port)
        packetHandler = PacketHandler(PROTOCOL_VERSION)

        if portHandler.openPort() and portHandler.setBaudRate(BAUDRATE):
            print(f'Open {self.port} Success...')
            self.is_connected = True
        else:
            print(f'Failed...')
            self.is_connected = False
            sys.exit(0)
        
        self.portHandler = portHandler
        self.packetHandler = packetHandler

        self.motors = [Motor(i+1, portHandler, packetHandler) for i in range(12)]

        if init:
            for m in self.motors:
                m.init_config(goal_pwm=goal_pwm)
            
            print(f'{self.name} init done...')


    def off(self):
        """
        失能所有电机
        """
        for m in self.motors:
            m.torq_off()
    
    def on(self):
        """
        使能所有电机
        """
        for m in self.motors:
            m.torq_on()


    def home(self):
        """
        GX11会到原点
        """
        motors = self.motors
        for m in motors:
            m.set_pos(0)
        time.sleep(1)

    def getj(self):
        """
        获取GX11关节角度，单位度
        """
        js = [m.get_pos() for m in self.motors]

        js_rect = []
        for j in js:
            if j > 180:
                j -= 360
            if j < -180:
                j += 360
            js_rect.append(j)


        return js_rect
    
    def setj(self, js):
        """
        设置GX11关节角度，单位度
        """
        for m, j in zip(self.motors, js):
            m.set_pos(j)
    
    def getj_finger1(self):
        """
        获取GX11指1关节角度，单位度
        """
        js = [m.get_pos() for m in self.motors[0:4]]
        js_rect = []
        for j in js:
            if j > 180:
                j -= 360
            if j < -180:
                j += 360
            js_rect.append(j)


        return js_rect
    
    def setj_finger1(self, js):
        """
        设置GX11指1关节角度，单位度
        """
        for m, j in zip(self.motors[0:4], js):
            m.set_pos(j)

    def getj_finger2(self):
        """
        获取GX11指2关节角度，单位度
        """
        js = [m.get_pos() for m in self.motors[4:8]]
        js_rect = []
        for j in js:
            if j > 180:
                j -= 360
            if j < -180:
                j += 360
            js_rect.append(j)


        return js_rect
    
    def setj_finger2(self, js):
        """
        设置GX11指2关节角度，单位度
        """
        for m, j in zip(self.motors[4:8], js):
            m.set_pos(j)

    def getj_finger3(self):
        """
        获取GX11指3关节角度，单位度
        """
        js = [m.get_pos() for m in self.motors[8:12]]
        js_rect = []
        for j in js:
            if j > 180:
                j -= 360
            if j < -180:
                j += 360
            js_rect.append(j)


        return js_rect
    
    def setj_finger3(self, js):
        """
        设置GX11指3关节角度，单位度
        """
        for m, j in zip(self.motors[8:12], js):
            m.set_pos(j)
    
    def set_zero_whole_hand(self):
        """
        GX11手掌设置编码器以当前位置归0，慎用
        """
        for m in self.motors[4:12]:
            m.set_zero()
    

    def fk_finger1(self):
        """获取finger1的正运动学，单位m"""
        xyz = self.kin.fk_finger1(self.getj_finger1())

        return xyz
    
    def ik_finger1(self, xyz):
        """获取finger1的逆运动学，单位m"""
        q = self.kin.ik_finger1(xyz)

        return q


    def fk_finger2(self):
        """获取finger2的正运动学，单位m"""
        xyz = self.kin.fk_finger2(self.getj_finger2())

        return xyz
    
    def ik_finger2(self, xyz):
        """获取finger2的逆运动学，单位m"""
        q = self.kin.ik_finger2(xyz)

        return q
    
    def fk_finger3(self):
        """获取finger3的正运动学，单位m"""
        xyz = self.kin.fk_finger3(self.getj_finger3())

        return xyz
    
    def ik_finger3(self, xyz):
        """获取finger3的逆运动学，单位m"""
        q = self.kin.ik_finger3(xyz)

        return q