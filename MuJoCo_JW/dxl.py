import os
import time
from math import pi
from dynamixel_sdk import *  

class Dynamixel:
    def __init__(self, id):
        # 통신 설정
        self.DEVICENAME = '/dev/ttyUSB0'  
        self.BAUDRATE = 1000000
        self.PROTOCOL_VERSION = 2.0       # Dynamixel 프로토콜 버전

        # 다이나믹셀 설정
        self.OPERATING_MODE = 11           # Pos, vel, torq control 
        self.CURRENT_CONTROL = 0
        self.POSITION_CONTROL = 3
        self.CURRENT_BASED_POSITION_CONTROL = 5
        self.DXL_ID = id                   # 제어할 다이나믹셀의 ID
        self.ADDR_TORQUE_ENABLE = 64       # Torque Enable 주소
        self.ADDR_GOAL_CURRENT = 102       # 목표 torque 주소
        self.ADDR_GOAL_POSITION = 116      # 목표 위치 주소
        self.ADDR_PRESENT_POSITION = 132   # 현재 위치 주소
        self.ADDR_PRESENT_VELOCITY = 128   # 현재 속도 주소
        self.TORQUE_ENABLE = 1             # 토크 활성화
        self.TORQUE_DISABLE = 0            # 토크 비활성화
        self.DXL_MINIMUM_POSITION_VALUE = 0     # 최소 위치 (예시)
        self.DXL_MAXIMUM_POSITION_VALUE = 4095  # 최대 위치 (예시)
        
        # 포트 핸들러와 패킷 핸들러 초기화
        self.portHandler = PortHandler(self.DEVICENAME)
        self.packetHandler = Protocol2PacketHandler()

        # 다이나믹셀 연결 및 위치 초기화
        self.connect_motor()
        self.get_init_state()
        # self.change_mode(self.CURRENT_CONTROL)

    def connect_motor(self):
        # 포트 열기
        if self.portHandler.openPort():
            print("포트를 성공적으로 열었습니다.")
        else:
            print("포트 열기에 실패했습니다.")
            quit()

        # 통신 속도 설정
        if self.portHandler.setBaudRate(self.BAUDRATE):
            print("통신 속도 설정 성공.")
        else:
            print("통신 속도 설정 실패.")
            quit()

        # 다이나믹셀의 토크 활성화
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print(f"토크 활성화 실패: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
            quit()

        print("다이나믹셀 토크 활성화 완료.")

    # def change_mode(self, num):
    #     # Operating Mode 변경
    #     dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, self.OPERATING_MODE, num)
    #     if dxl_comm_result != COMM_SUCCESS:
    #         print(f"Operating Mode 변경 실패: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
    #         quit()

    #     print(f"Operating Mode가 {num}로 변경되었습니다.")

    def get_init_state(self):
        self.init_state, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_PRESENT_POSITION)

        if self.init_state > 4000000:
            self.init_state = self.init_state - 4294965248 - 2048
        
        self.init_state = self.init_state / 2048 * pi

    
    def control(self, goal):
        if goal < 0:
            goal *= 1.5
        goal_torque = int(goal)
        
        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_GOAL_CURRENT, goal_torque)

    def get_qpos(self):
        dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_PRESENT_POSITION)

        if dxl_present_position > 4000000:
            dxl_present_position = dxl_present_position - 4294965248 - 2048

        return dxl_present_position / 2048 * pi #- self.init_state
    
    def get_qvel(self):
        dxl_present_velocity, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_PRESENT_VELOCITY)
        
        if dxl_present_velocity > 400000:
            dxl_present_velocity = dxl_present_velocity - 4294967296

        return dxl_present_velocity * 0.229 / 60

    def close_port(self):
        self.portHandler.closePort()


# import numpy as np

# m1 = Dynamixel(14)
 
# while 1:
#     torque = 0

#     m1.control(torque)
#     print("torque : ", torque, "\t", "qpos : ", m1.get_qpos(), "\t", "qvel : ", m1.get_qvel())

# m1.close_port()