#!/usr/bin/env python
import os, sys
import rospy
from JW_control import *
import mujoco_py
from std_msgs.msg import Float64MultiArray
from dxl import *
# cur_dir = os.getcwd()
# target_dir = os.path.join(cur_dir, '/comprehensive_design_project/src/ROS_remote/')
# print(target_dir)
# sys.path.append(target_dir)

# from position_sub import cur_pos


class MuJoCo:
    def __init__(self):
        self.position_sub = rospy.Subscriber('positions', Float64MultiArray, self.position_callback)
        self.xml_path = './MuJoCo_JW/model/scene.xml'
        self.m = mujoco_py.load_model_from_path(self.xml_path)
        self.sim = mujoco_py.MjSim(self.m)
        self.d = self.sim.data
        self.viewer = mujoco_py.MjViewer(self.sim)        
        self.robot = JWControl(self.sim, self.m, self.d)
        self.cur_pos = []

    def position_callback(self, msg):
        self.cur_pos = msg.data
        # print(msg.data)
# class Remote_contorl:
#     def __init__(self):
#         rospy.init_node('remote_sub_node2')
#         self.position_sub = rospy.Subscriber('positions', Float64MultiArray, self.position_callback)
#         self.qpos_d = [0, 0, 0, 0, 0]
    
#     def position_callback(self, msg):
#         print(msg.data)
#         self.qpos_d = msg.data
    
def main():
    mujoco = MuJoCo()
    # rate = rospy.Rate(200)
    # rospy.init_node('main_node')
    
    # AT = Remote_contorl()
    # dxl = MainDynamixel([11, 12, 13, 14, 15])
    while 1: # 반복 제어랑 원격제어랑 if로 아예 공간 나눠야함
        # dxl.control_pos([0,0,0,0,0])
        print(mujoco.cur_pos)
        
        joint_torq = mujoco.robot.move_fixed_traj()

        # if d.time >= 5:
        #     joint_torq = robot.go_init_point()

        for i in range(mujoco.m.nu):
            mujoco.d.ctrl[i] = joint_torq[i]

        mujoco.sim.step()
        mujoco.viewer.render()
        # rate.sleep()
        

if __name__=='__main__':
    main()
