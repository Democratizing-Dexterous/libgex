from .dynamixel_sdk import * 
import os
import time
import numpy as np

class Motor:
    def __init__(self, id, portHandler, packetHandler) -> None:
        self.id = id
        self.unit_scale = 0.087891 # degree
        self.port_handler = portHandler
        self.packet_handler = packetHandler

        self.addr_torq_enable = 64
        self.addr_led = 65
        self.addr_present_pos = 132
        self.addr_present_current = 126
        self.addr_goal_pos = 116
        self.addr_curr_limit = 38
        self.addr_pos_p = 84
        self.addr_pos_d = 80
        self.addr_vel_limit = 44
       
        self.addr_home_off = 20
        self.addr_goal_curr = 102
        self.addr_goal_pwm = 100

        # 运行模式
        self.addr_operating_mode = 11
        self.curr_operating_mode = 0 # 电流模式
        self.vel_operating_mode = 1
        self.pos_operating_mode = 3
        self.extpos_operating_mode = 4
        self.currpos_operating_mode = 5 # 推荐使用，位置力控模式
        self.pwm_operating_mode = 16

        self.curr_limit = 1200
        self.curr_max = 1200
    
    def led_on(self):
        """
        开启LED
        """
        self.packet_handler.write1ByteTxRx(self.port_handler, self.id, self.addr_led, 1) 

    def led_off(self):
        """
        关闭LED
        """
        self.packet_handler.write1ByteTxRx(self.port_handler, self.id, self.addr_led, 0) 
    
    def torq_on(self):
        """
        打开电机使能
        """
        self.packet_handler.write1ByteTxRx(self.port_handler, self.id, self.addr_torq_enable, 1) 
    
    def torq_off(self):
        """
        关闭电机使能
        """
        self.packet_handler.write1ByteTxRx(self.port_handler, self.id, self.addr_torq_enable, 0) 

    def get_pos(self):
        """
        获取电机位置，单位度
        """
        pos = self.packet_handler.read4ByteTxRx(self.port_handler, self.id, self.addr_present_pos)
        pos = pos[0]
        if pos > 2**31:
            pos = pos - 2**32
        else:
            pos = pos
        return pos*self.unit_scale
    
    def set_zero(self):
        """
        设置电机归0
        """
        self.packet_handler.write4ByteTxRx(self.port_handler, self.id, self.addr_home_off, 0) 
        pos = self.get_pos()
        self.packet_handler.write4ByteTxRx(self.port_handler, self.id, self.addr_home_off, -int(pos/self.unit_scale)) 
    
    def set_curr_limit(self, curr=800):
        # 设置最大电流，单位mA
        self.packet_handler.write2ByteTxRx(self.port_handler, self.id, self.addr_curr_limit, curr)
        if curr > self.curr_max:
            curr = self.curr_max
        self.curr_limit = curr

    def pos_force_mode(self, goal_current=600, goal_pwm=200):
        """
        设置力控位置模式
        """
        self.torq_off()
        # 力控位置模式
        self.packet_handler.write1ByteTxRx(self.port_handler, self.id, self.addr_operating_mode, self.currpos_operating_mode)

        # https://emanual.robotis.com/docs/en/dxl/x/xl330-m288/
        if goal_current > self.curr_limit:
            goal_current = self.curr_limit
        # 设置goal current
        self.packet_handler.write2ByteTxRx(self.port_handler, self.id, self.addr_goal_curr, goal_current)

        if goal_pwm > 885:
            goal_pwm = 885
        
        # 设置goal pwm
        self.packet_handler.write2ByteTxRx(self.port_handler, self.id, self.addr_goal_pwm, goal_pwm) # 885 Max
        self.torq_on()

    def force_mode(self):
        """
        设置电流模式
        """
        self.torq_off()
        # 力控模式
        self.packet_handler.write1ByteTxRx(self.port_handler, self.id, self.addr_operating_mode, self.curr_operating_mode)

        
    def init_config(self, curr_limit=800, goal_current=300, goal_pwm=800):
        """
        电机初始配置，LED闪烁，设置为力控位置模式，并使能电机，goal_current影响力的大小，goal_pwm影响速度
        """
        for i in range(2):
            self.led_on()
            time.sleep(0.1)
            self.led_off()
            time.sleep(0.1)
        self.set_curr_limit(curr_limit)
        self.pos_force_mode(goal_current, goal_pwm)
         

    def set_pos(self, pos):
        """
        设置目标角度，单位度
        """
        self.packet_handler.write4ByteTxRx(self.port_handler, self.id, self.addr_goal_pos, int(pos/self.unit_scale)) 

    def set_curr(self, cur):
        """
        设置目标电流，单位mA
        """
        self.packet_handler.write2ByteTxRx(self.port_handler, self.id, self.addr_goal_curr, cur)


    
