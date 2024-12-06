from math import pi
from dynamixel_sdk import *  
import numpy as np

# ========================================================= MAIN MOTOR ==========================================================================
class MainDynamixel:
    def __init__(self, ids):
        # 통신 설정
        self.DEVICENAME = '/dev/ttyUSB0'  
        self.BAUDRATE = 3000000
        self.PROTOCOL_VERSION = 2.0       # Dynamixel 프로토콜 버전

        # 다이나믹셀 설정
        self.OPERATING_MODE = 11           # Pos, vel, torq control 
        self.CURRENT_CONTROL = 0
        self.POSITION_CONTROL = 4
        self.CURRENT_BASED_POSITION_CONTROL = 5
        self.DXL_IDs = ids                 # 제어할 다이나믹셀의 ID
        self.ADDR_TORQUE_ENABLE = 64       # Torque Enable 주소
        self.ADDR_GOAL_CURRENT = 102       # 목표 torque 주소
        self.ADDR_GOAL_POSITION = 116      # 목표 위치 주소
        self.ADDR_PRESENT_POSITION = 132   # 현재 위치 주소
        self.ADDR_PRESENT_VELOCITY = 128   # 현재 속도 주소
        self.TORQUE_ENABLE = 1             # 토크 활성화
        self.TORQUE_DISABLE = 0            # 토크 비활성화
        self.DXL_MINIMUM_POSITION_VALUE = 0     # 최소 위치 (예시)
        self.DXL_MAXIMUM_POSITION_VALUE = 4095  # 최대 위치 (예시)
        self.PROFILE_VELOCITY = 112
        
        # 포트 핸들러와 패킷 핸들러 초기화
        self.portHandler = PortHandler(self.DEVICENAME)
        self.packetHandler = Protocol2PacketHandler()

        self.groupSyncWriteTorque = GroupSyncWrite(self.portHandler, self.packetHandler, self.ADDR_GOAL_CURRENT, 2)
        self.groupSyncWritePosition = GroupSyncWrite(self.portHandler, self.packetHandler, self.ADDR_GOAL_POSITION, 4)
        self.groupSyncReadPosition = GroupSyncRead(self.portHandler,  self.packetHandler, self.ADDR_PRESENT_POSITION, 4)
        self.groupSyncReadVelocity = GroupSyncRead(self.portHandler,  self.packetHandler, self.ADDR_PRESENT_VELOCITY, 4)
        
        # 다이나믹셀 연결 및 위치 초기화
        self.connect_motor()
        self.change_mode(self.POSITION_CONTROL)
        self.enable_torque()
        self.limit_velocity(20)
        self.init_state = self.get_init_state()
    
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
    
    def enable_torque(self):
        # 다이나믹셀의 토크 활성화
        for dxl_id in self.DXL_IDs:
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)
            if dxl_comm_result != COMM_SUCCESS:
                print(f"토크 활성화 실패: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
                quit()

            print("다이나믹셀 토크 활성화 완료.")
    
    def disable_torque(self):
        # 다이나믹셀의 토크 활성화
        for dxl_id in self.DXL_IDs:
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, self.ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE)
            if dxl_comm_result != COMM_SUCCESS:
                print(f"토크 비활성화 실패: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
                quit()

            print("다이나믹셀 토크 비활성화 완료.")

    def change_mode(self, mode):
        # Operating Mode 변경
        for dxl_id in self.DXL_IDs:
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, self.OPERATING_MODE, mode)
            if dxl_comm_result != COMM_SUCCESS:
                print(f"Operating Mode 변경 실패: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
                quit()

        print(f"Operating Mode가 {mode}로 변경되었습니다.")

    def limit_velocity(self, num):
        for dxl_id in self.DXL_IDs:
            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, dxl_id, self.PROFILE_VELOCITY, num)
            if dxl_comm_result != COMM_SUCCESS:
                print(f"Profile velocity 변경 실패: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
                quit()
        
        print(f"Profile velocity가 {num}로 변경되었습니다.")

    def get_init_state(self):
        self.control_pos([0,0,0,0,0,0])
        init_states = {}
        for dxl_id in self.DXL_IDs:
            init_state, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, dxl_id, self.ADDR_PRESENT_POSITION)
            if init_state > 4000000:
                init_state = init_state - 4294965248 - 2048
            init_states[dxl_id] = init_state / 2048 * pi

        return init_states

    
    def control_torque(self, torque_values):
        self.groupSyncWriteTorque.clearParam()
        
        for dxl_id, goal in zip(self.DXL_IDs[:-1], torque_values):
            if goal < 0:
                goal *= 2
            goal_torque = int(goal)
            param_goal_torque = [DXL_LOBYTE(goal_torque), DXL_HIBYTE(goal_torque)]
            self.groupSyncWriteTorque.addParam(dxl_id, param_goal_torque)

        self.groupSyncWriteTorque.txPacket()
        
    def control_pos(self, goal_position):
        self.groupSyncWritePosition.clearParam()
        for dxl_id, goal in zip(self.DXL_IDs[:-1], goal_position):
            goal = goal / pi * 2048 #+ init_state / pi * 2048
            goal_pos = int(goal)
            param_goal_pos = [DXL_LOBYTE(DXL_LOWORD(goal_pos)),
                              DXL_HIBYTE(DXL_LOWORD(goal_pos)),
                              DXL_LOBYTE(DXL_HIWORD(goal_pos)),
                              DXL_HIBYTE(DXL_HIWORD(goal_pos))]
            self.groupSyncWritePosition.addParam(dxl_id, param_goal_pos)
        
        self.groupSyncWritePosition.txPacket()
  
    def control_by_remote(self, goal_position):
        self.groupSyncWritePosition.clearParam()
        for dxl_id, goal in zip(self.DXL_IDs, goal_position):
            goal = goal / pi * 2048 #+ init_state / pi * 2048
            goal_pos = int(goal)
            param_goal_pos = [DXL_LOBYTE(DXL_LOWORD(goal_pos)),
                              DXL_HIBYTE(DXL_LOWORD(goal_pos)),
                              DXL_LOBYTE(DXL_HIWORD(goal_pos)),
                              DXL_HIBYTE(DXL_HIWORD(goal_pos))]
            self.groupSyncWritePosition.addParam(dxl_id, param_goal_pos)
        
        self.groupSyncWritePosition.txPacket()
    
    def control_gripper(self, goal_position):
        goal_position = int(goal_position)
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL_IDs[-1], self.ADDR_GOAL_POSITION, goal_position)

    def get_qpos(self):
        self.groupSyncReadPosition.clearParam()
        results = []

        for num, dxl_id in enumerate(self.DXL_IDs):
            if not self.groupSyncReadPosition.addParam(dxl_id):
                print(f"Failed to add parameter for ID {dxl_id}")
                continue

        dxl_comm_result = self.groupSyncReadPosition.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print(f"Communication failed with result: {dxl_comm_result}")
            return dxl_comm_result

        for dxl_id in self.DXL_IDs:
            position = self.groupSyncReadPosition.getData(dxl_id, self.ADDR_PRESENT_POSITION, 4)
            if position > 4000000:
                position = position - 4294965248 - 2048
            position = position / 2048 * pi
            results.append(position)

        self.groupSyncReadPosition.clearParam()
        return results

    
    def get_qvel(self):
        self.groupSyncReadVelocity.clearParam()
        results = []

        for num, dxl_id in enumerate(self.DXL_IDs):
            if not self.groupSyncReadVelocity.addParam(dxl_id):
                print(f"Failed to add parameter for ID {dxl_id}")
                continue

        dxl_comm_result = self.groupSyncReadVelocity.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print(f"Communication failed with result: {dxl_comm_result}")
            return dxl_comm_result

        for dxl_id in self.DXL_IDs:
            velocity = self.groupSyncReadVelocity.getData(dxl_id, self.ADDR_PRESENT_VELOCITY, 4)
            if velocity > 400000:
                velocity = velocity - 4294967296
            velocity = velocity * 0.229 / 60
            results.append(velocity)

        self.groupSyncReadVelocity.clearParam()
        return results

    def close_port(self):
        self.portHandler.closePort()

# ========================================================= REMOTE MOTOR ==========================================================================
class RemoteDynamixel:
    def __init__(self, ids):
        # 통신 설정
        self.DEVICENAME = '/dev/ttyUSB0'  
        self.BAUDRATE = 3000000
        self.PROTOCOL_VERSION = 2.0       # Dynamixel 프로토콜 버전

        # 다이나믹셀 설정
        self.OPERATING_MODE = 11           # Pos, vel, torq control 
        self.CURRENT_CONTROL = 0
        self.POSITION_CONTROL = 4
        self.CURRENT_BASED_POSITION_CONTROL = 5
        self.DXL_IDs = ids                 # 제어할 다이나믹셀의 ID
        self.ADDR_TORQUE_ENABLE = 64       # Torque Enable 주소
        self.ADDR_GOAL_CURRENT = 102       # 목표 torque 주소
        self.ADDR_GOAL_POSITION = 116      # 목표 위치 주소
        self.ADDR_PRESENT_POSITION = 132   # 현재 위치 주소
        self.ADDR_PRESENT_VELOCITY = 128   # 현재 속도 주소
        self.TORQUE_ENABLE = 1             # 토크 활성화
        self.TORQUE_DISABLE = 0            # 토크 비활성화
        self.DXL_MINIMUM_POSITION_VALUE = 0     # 최소 위치 (예시)
        self.DXL_MAXIMUM_POSITION_VALUE = 4095  # 최대 위치 (예시)
        self.PROFILE_VELOCITY = 112
        
        # 포트 핸들러와 패킷 핸들러 초기화
        self.portHandler = PortHandler(self.DEVICENAME)
        self.packetHandler = Protocol2PacketHandler()

        self.groupSyncWriteTorque = GroupSyncWrite(self.portHandler, self.packetHandler, self.ADDR_GOAL_CURRENT, 2)
        self.groupSyncWritePosition = GroupSyncWrite(self.portHandler, self.packetHandler, self.ADDR_GOAL_POSITION, 4)
        self.groupSyncReadPosition = GroupSyncRead(self.portHandler,  self.packetHandler, self.ADDR_PRESENT_POSITION, 4)
        self.groupSyncReadVelocity = GroupSyncRead(self.portHandler,  self.packetHandler, self.ADDR_PRESENT_VELOCITY, 4)
        
        # 다이나믹셀 연결 및 위치 초기화
        self.connect_motor()

        self.change_mode(self.POSITION_CONTROL)
        self.enable_torque()
        self.limit_velocity(60)
        self.init_state = self.get_init_state()
        self.disable_torque()

        self.change_mode(self.CURRENT_CONTROL)
        self.enable_torque()
        self.limit_velocity(60)

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
    
    def enable_torque(self):
        # 다이나믹셀의 토크 활성화
        for dxl_id in self.DXL_IDs:
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)
            if dxl_comm_result != COMM_SUCCESS:
                print(f"토크 활성화 실패: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
                quit()

            print("다이나믹셀 토크 활성화 완료.")
    
    def disable_torque(self):
        # 다이나믹셀의 토크 활성화
        for dxl_id in self.DXL_IDs:
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, self.ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE)
            if dxl_comm_result != COMM_SUCCESS:
                print(f"토크 비활성화 실패: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
                quit()

            print("다이나믹셀 토크 비활성화 완료.")

    def change_mode(self, mode):
        # Operating Mode 변경
        for dxl_id in self.DXL_IDs:
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, self.OPERATING_MODE, mode)
            if dxl_comm_result != COMM_SUCCESS:
                print(f"Operating Mode 변경 실패: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
                quit()

        print(f"Operating Mode가 {mode}로 변경되었습니다.")

    def limit_velocity(self, num):
        for dxl_id in self.DXL_IDs:
            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, dxl_id, self.PROFILE_VELOCITY, num)
            if dxl_comm_result != COMM_SUCCESS:
                print(f"Profile velocity 변경 실패: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
                quit()
        
        print(f"Profile velocity가 {num}로 변경되었습니다.")

    def get_init_state(self):
        self.control_pos([0,0,0,0,0,0])
        init_states = {}
        for dxl_id in self.DXL_IDs:
            init_state, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, dxl_id, self.ADDR_PRESENT_POSITION)
            if init_state > 4000000:
                init_state = init_state - 4294965248 - 2048
            init_states[dxl_id] = init_state / 2048 * pi
        return init_states

    
    def control_torque(self, torque_values):
        self.groupSyncWriteTorque.clearParam()
        
        for dxl_id, goal in zip(self.DXL_IDs, torque_values):
            if goal < 0:
                goal *= 2
            goal_torque = int(goal)
            param_goal_torque = [DXL_LOBYTE(goal_torque), DXL_HIBYTE(goal_torque)]
            self.groupSyncWriteTorque.addParam(dxl_id, param_goal_torque)

        self.groupSyncWriteTorque.txPacket()
        
    def control_pos(self, goal_position):
        self.groupSyncWritePosition.clearParam()
        for dxl_id, goal in zip(self.DXL_IDs, goal_position):
            goal = goal / pi * 2048 #+ init_state / pi * 2048
            goal_pos = int(goal)
            param_goal_pos = [DXL_LOBYTE(DXL_LOWORD(goal_pos)),
                              DXL_HIBYTE(DXL_LOWORD(goal_pos)),
                              DXL_LOBYTE(DXL_HIWORD(goal_pos)),
                              DXL_HIBYTE(DXL_HIWORD(goal_pos))]
            self.groupSyncWritePosition.addParam(dxl_id, param_goal_pos)
        
        self.groupSyncWritePosition.txPacket()

    def get_qpos(self):
        self.groupSyncReadPosition.clearParam()
        results = []

        for num, dxl_id in enumerate(self.DXL_IDs):
            if not self.groupSyncReadPosition.addParam(dxl_id):
                print(f"Failed to add parameter for ID {dxl_id}")
                continue

        dxl_comm_result = self.groupSyncReadPosition.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print(f"Communication failed with result: {dxl_comm_result}")
            return dxl_comm_result

        for dxl_id in self.DXL_IDs:
            position = self.groupSyncReadPosition.getData(dxl_id, self.ADDR_PRESENT_POSITION, 4)
            if position > 4000000:
                position = position - 4294965248 - 2048
            position = position / 2048 * pi
            results.append(position)

        self.groupSyncReadPosition.clearParam()
        return results
    
    def get_qvel(self):
        self.groupSyncReadVelocity.clearParam()
        results = []

        for num, dxl_id in enumerate(self.DXL_IDs):
            if not self.groupSyncReadVelocity.addParam(dxl_id):
                print(f"Failed to add parameter for ID {dxl_id}")
                continue

        dxl_comm_result = self.groupSyncReadVelocity.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print(f"Communication failed with result: {dxl_comm_result}")
            return dxl_comm_result

        for dxl_id in self.DXL_IDs:
            velocity = self.groupSyncReadVelocity.getData(dxl_id, self.ADDR_PRESENT_VELOCITY, 4)
            if velocity > 400000:
                velocity = velocity - 4294967296
            velocity = velocity * 0.229 / 60
            results.append(velocity)

        self.groupSyncReadVelocity.clearParam()
        return results

    def close_port(self):
        self.portHandler.closePort()

# import numpy as np
 
# m1 = RemoteDynamixel([5,6,7,8,9])
# while 1:
#     print("pos : ",m1.get_qpos())
#     print("vel : ",m1.get_qvel())

 

# for i in np.linspace(0, 4, 100):
#     torque = -pi * i / 8
#     m1.control_pos(torque)
#     print("torque : ", torque, "\t", "qpos : ", m1.get_qpos(), "\t", "qvel : ", m1.get_qvel())

# m1.close_port()