import numpy as np
from kinematic import *
from trajectory import Trajectory
from dxl import MainDynamixel, RemoteDynamixel

class JWControl:
    def __init__(self, sim, model, data):
        # rospy.init_node('remote_sub_node2')
        # self.position_sub = rospy.Subscriber('positions', Float64MultiArray, self.position_callback)
        self.m = model
        self.sim = sim
        self.d = data
        self.qpos_d = []

        self.initialize_params()

        # main_dxl_ids = []
        # self.main_dxl = MainDynamixel(main_dxl_ids)
        # remote_dxl_ids = [5,6,7,8,9]
        # self.remote_dxl = RemoteDynamixel(remote_dxl_ids)

    # def position_callback(self, msg):
    #     self.qpos_d = msg.data

    def initialize_params(self):
        self.traj_time = [0, 4, 8, 12, 16]
        self.cnt_init = 0
        self.cnt_remote = 0
        self.linear_d = [None] * len(self.traj_time)
        self.angular_d = [None] * len(self.traj_time)
        for i in range(len(self.traj_time) - 1):
            self.linear_d[i] = Trajectory(self.traj_time[i], self.traj_time[i + 1])
            self.angular_d[i] = Trajectory(self.traj_time[i], self.traj_time[i + 1])
        
        self.init_linear_d = None
        self.init_angular_d = None
        self.remote_d = None
        
        self.qpos = [None] * self.m.njnt
        self.qpos = np.reshape(self.qpos, (self.m.njnt,))
        self.qvel = [None] * self.m.njnt
        self.qvel = np.reshape(self.qvel, (self.m.njnt,))
        self.qpos_d = 0
        self.quat_e_pre = [1, 0 ,0 ,0]
        
        self.K = Kinematic(self.sim)

    def kinematics(self, qpos):
        P_EE,R_EE,P_lnk,R_lnk = self.K.forward_kinematics(qpos)
        # print(qpos)
        jnt_axes = self.K.get_jnt_axis()
        J_p, J_r = self.K.get_jacobian(P_lnk, jnt_axes, P_EE)
        J_pr = np.vstack((J_p, J_r))

        quat_e = Rot2Quat(R_EE)
        if np.dot(quat_e, self.quat_e_pre) < 0:
            quat_e = -quat_e
        self.quat_e_pre = quat_e

        return P_EE,R_EE,P_lnk,R_lnk,J_pr,quat_e

    def move_fixed_traj(self, remote_qpos, remote_qvel):
        # ========================================Kinematics: START===============================================
        P_EE,R_EE,P_lnk,R_lnk,J_pr,quat_e = self.kinematics(self.d.qpos)

        P_EE_mj = self.d.body_xpos[-1]
        quat_mj = self.d.body_xquat[-1]
        R_EE_mj = np.reshape(self.d.body_xmat[-1], (3,3))
        # ==========================================Kinematics: END===============================================
        pos_d, vel_d = P_EE, np.zeros((3,)) 
        quat_d, quatdot_d = quat_e, np.zeros((4,))
    
        # ========================================Trajectory: START===============================================
        init_state = P_EE
        final_state = [0.375, 0, 0.2525]
        self.linear_d[0].get_coeff(init_state, final_state)

        init_state = self.linear_d[0].final_state
        final_state = [0.127, 0, 0.4175]
        self.linear_d[1].get_coeff(init_state, final_state)

        init_state = self.linear_d[1].final_state
        final_state = [0.275, 0.275, 0.245]
        self.linear_d[2].get_coeff(init_state, final_state)

        init_state = self.linear_d[2].final_state
        final_state = [-0.287, 0.287, 0.2175]
        self.linear_d[3].get_coeff(init_state, final_state)
        # ========================================================================================================
        init_state = quat_e
        final_rot = Rot_y(90)
        final_state = Rot2Quat(final_rot)
        self.angular_d[0].get_coeff_quat(init_state, final_state)

        init_state = self.angular_d[0].final_state
        final_rot = final_rot # local coordinate
        final_state = Rot2Quat(final_rot)
        self.angular_d[1].get_coeff_quat(init_state, final_state)    

        init_state = self.angular_d[1].final_state
        final_rot = final_rot @ Rot_x(-45) # local coordinate
        final_state = Rot2Quat(final_rot)
        self.angular_d[2].get_coeff_quat(init_state, final_state)  

        init_state = self.angular_d[2].final_state
        final_rot = final_rot @ Rot_x(-90) # local coordinate
        final_state = Rot2Quat(final_rot)
        self.angular_d[3].get_coeff_quat(init_state, final_state)  
        # ========================================================================================================

        if self.d.time <= self.traj_time[1]:
            pos_d, vel_d, acc_d = self.linear_d[0].calculate_pva(self.d.time)
            quat_d, quatdot_d, quatddot_d = self.angular_d[0].calculate_pva_quat(self.d.time)
        elif self.d.time <= self.traj_time[2]:
            pos_d, vel_d, acc_d = self.linear_d[1].calculate_pva(self.d.time)
            quat_d, quatdot_d, quatddot_d = self.angular_d[1].calculate_pva_quat(self.d.time)
        elif self.d.time <= self.traj_time[3]:
            pos_d, vel_d, acc_d = self.linear_d[2].calculate_pva(self.d.time)
            quat_d, quatdot_d, quatddot_d = self.angular_d[2].calculate_pva_quat(self.d.time)
        elif self.d.time <= self.traj_time[4]:
            pos_d, vel_d, acc_d = self.linear_d[3].calculate_pva(self.d.time)
            quat_d, quatdot_d, quatddot_d = self.angular_d[3].calculate_pva_quat(self.d.time)

        # ==========================================Trajectory: END===============================================
        vel_CLIK = vel_d + 50 * (pos_d - P_EE)
        quatdot_CLIK = quatdot_d + 50 * (quat_d - quat_e)
        
        del_quat = mul_quat(quat_d, inverse_quat(quat_e))
        angle_error = 2 * np.arctan2(np.linalg.norm(del_quat[1:]), del_quat[0])  # 각도
        axis_error = del_quat[1:] / (np.linalg.norm(del_quat[1:]) + 10 ** (-5))  # 축
        omega_error = axis_error * angle_error
        
        omega = Quat2Omega(quat_d, quatdot_CLIK)
        total = np.hstack((vel_CLIK,omega))
        null = np.eye(5) - np.linalg.pinv(J_pr) @ J_pr
        
        qvel_d = SVD_DLS_inverse(J_pr) @ np.reshape(total, (6,)) + null @ self.d.qvel

        # dxl.control_pos(d.qpos)
        # self.qpos_d = self.qpos_d + qvel_d * self.m.opt.timestep

        # joint_torq = 2000 * (qpos_d - d.qpos) + 1300 * (qvel_d - d.qvel)
        # joint_torq = 50 * (self.qpos_d - self.d.qpos) + 30 * (qvel_d - self.d.qvel)  # 200Hz  50, 50
        # joint_torq = 10 * (qpos_d - d.qpos) + 20 * (qvel_d - d.qvel)  # 100Hz  100, 100
        # joint_torq = 3 * (qpos_d - d.qpos) + 20 * (qvel_d - d.qvel) # 50Hz  3, 20 40 50 끝까지 흔들리긴함

        # joint_torq = 50 * (float_list - self.d.qpos)
        joint_torq = 50 * (remote_qpos - self.d.qpos) + 30 * (remote_qvel - self.d.qvel)
        # self.remote_dxl.control_pos(self.d.qpos)

        return joint_torq

    def remote_control(self, qpos_ros):
        qpos_d, qvel_d = self.d.qpos, np.zeors((5,))

        if self.cnt_remote == 0:
            init_time = self.d.time
            final_time = init_time + 3
            self.remote_d = Trajectory(init_time, final_time)
            self.remote_d.get_coeff_qpos(self.main_dxl.get_qpos(), qpos_ros)
            self.cnt_remote += 1
        
        qpos_d, qvel_d, qacc_d = self.remote_d.calculate_pva_qpos(self.d.time)

        joint_torq = 50 * (qpos_d - self.d.qpos) + 30 * (qvel_d - self.d.qvel)

        if self.d.time <= final_time:
            self.main_dxl.control_pos(self.d.qpos)
        else:
            self.main_dxl.control_pos(qpos_ros)

    def go_init_point(self):
        init_pos = [0.1765, 0, 0.2265]

        P_EE,R_EE,P_lnk,R_lnk,J_pr,quat_e = self.kinematics(self.d.qpos)

        pos_d, vel_d = P_EE, np.zeros((3,)) 
        quat_d, quatdot_d = quat_e, np.zeros((4,))

        if self.cnt_init == 0:
            init_time = self.d.time
            final_time = init_time + 3
            self.init_linear_d = Trajectory(init_time, final_time)
            self.init_linear_d.get_coeff(P_EE, init_pos)

            self.init_angular_d = Trajectory(init_time, final_time)
            final_rot = Rot_y(90)
            final_state = Rot2Quat(final_rot)
            self.init_angular_d.get_coeff_quat(quat_e, final_state)
            
            self.qpos_d = 0
            self.cnt_init += 1
        

        pos_d, vel_d, acc_d = self.init_linear_d.calculate_pva(self.d.time)
        quat_d, quatdot_d, quatddot_d = self.init_angular_d.calculate_pva_quat(self.d.time)

        vel_CLIK = vel_d + 50 * (pos_d - P_EE)
        quatdot_CLIK = quatdot_d + 50 * (quat_d - quat_e)

        omega = Quat2Omega(quat_d, quatdot_CLIK)
        total = np.hstack((vel_CLIK,omega))
        
        qvel_d = SVD_DLS_inverse(J_pr) @ np.reshape(total, (6,))
        self.qpos_d = self.qpos_d + qvel_d * self.m.opt.timestep
        joint_torq = 60 * (self.qpos_d - self.d.qpos) + 30 * (qvel_d - self.d.qvel)

        return joint_torq
