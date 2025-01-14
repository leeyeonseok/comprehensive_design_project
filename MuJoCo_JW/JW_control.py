from kinematic import *
from trajectory import Trajectory
from dxl import MainDynamixel, RemoteDynamixel

class JWControl:
    def __init__(self, sim, model, data, start_time=0):
        self.m = model
        self.sim = sim
        self.d = data
        self.start_time = start_time
        
        self.initialize_params()

    def initialize_params(self):
        self.traj_time = []
        for i in range(18 + 1): # +1은 시작시간
            self.traj_time.append(2 * i + self.start_time)
        
        self.linear_d = [None] * (len(self.traj_time))
        self.angular_d = [None] * (len(self.traj_time))
        for i in range(len(self.traj_time) - 1):
            self.linear_d[i] = Trajectory(self.traj_time[i], self.traj_time[i + 1])
            self.angular_d[i] = Trajectory(self.traj_time[i], self.traj_time[i + 1])
        
        self.init_qpos_d = None
        self.remote_d = None

        self.points = None

        self.cnt = 0
        self.cnt_init = 0
        self.cnt_remote = 0

        self.complete_a_cycle = False
        
        self.qpos_d = 0
        self.quat_e_pre = [1, 0 ,0 ,0]
        
        self.K = Kinematic(self.sim)
        self.step = None
        self.define_points()


    def define_points(self): # 17s without start point  [-0.0940, -0.3152,  0.2300] / [ 0.1800,  0.2950,  0.1200] / [ 0.3270, -0.3021,  0.0650]
        self.points = np.array([[ 0.1942,  0.0000,  0.1842],   # 1
                                [ 0.0000, -0.1950,  0.1860],   # 2
                                [-0.0520, -0.2410,  0.2240],   # 3
                                [-0.0940, -0.3152,  0.2300],   # 4
                                [-0.0940, -0.3152,  0.2300],   # 5
                                [ 0.1280, -0.1280,  0.1725],   # 6
                                [ 0.1280,  0.1280,  0.1425],   # 7
                                [ 0.1530,  0.2507,  0.1425],   # 8
                                [ 0.1800,  0.2950,  0.1530],   # 9
                                [ 0.1800,  0.2950,  0.1530],   # 10
                                [ 0.1530,  0.2507,  0.1425],   # 11
                                [ 0.2480,  0.0000,  0.1425],   # 12
                                [ 0.2543, -0.0840,  0.1410],   # 13
                                [ 0.3270, -0.3021,  0.0750],   # 14
                                [ 0.3270, -0.3021,  0.0750],   # 15
                                [ 0.2670, -0.2470,  0.1243],   # 16
                                [ 0.2043,  0.0000,  0.2010],   # 17
                                [ 0.0910,  0.0000,  0.1015]])  # 18
                                #[ 0.1050,  0.0000,  0.2015]])  # 17
        

    def kinematics(self, qpos):
        P_EE,R_EE,P_lnk,R_lnk = self.K.forward_kinematics(qpos)
        jnt_axes = self.K.get_jnt_axis()

        J_p, J_r = self.K.get_jacobian(self.d.body_xpos[1:], jnt_axes, P_EE)
        J_pr = np.vstack((J_p, J_r))

        quat_e = Rot2Quat(R_EE)
        quat_e /= (np.linalg.norm(quat_e) + 10 ** (-8))

        if np.dot(quat_e, self.quat_e_pre) < 0:
            quat_e = -quat_e
        self.quat_e_pre = quat_e
        
        return P_EE,R_EE,P_lnk,R_lnk,J_pr,quat_e
    

    def generate_traj(self, start_point, start_orien, points):
        if isinstance(points, list):
            points = np.array(points)

        if points.ndim == 1:
            points = points.reshape(1,-1)

        for i in range(len(points)):
            if i == 0:
                init_state = start_point
                final_state = points[i]
                self.linear_d[i].get_coeff(init_state, final_state)

                init_state = start_orien
                rot = get_XYdeg(final_state)
                final_rot = Rot_y(90) @ Rot_x(-rot) # local coordinate
                final_state = Rot2Quat(final_rot)
                self.angular_d[i].get_coeff_quat(init_state, final_state)
            else:
                init_state = self.linear_d[i - 1].final_state
                final_state = points[i]
                self.linear_d[i].get_coeff(init_state, final_state)

                init_state = self.angular_d[i - 1].final_state
                deg = get_XYdeg(points[i])
                final_rot = Rot_y(90) @ Rot_x(-deg) # local coordinate
                final_state = Rot2Quat(final_rot)
                self.angular_d[i].get_coeff_quat(init_state, final_state)

    def move_fixed_traj(self, points):
        # ========================================Kinematics: START===============================================
        P_EE, R_EE, P_lnk, R_lnk, J_pr, quat_e = self.kinematics(self.d.qpos)

        P_EE_mj = self.d.body_xpos[-1]
        quat_mj = self.d.body_xquat[-1]
        R_EE_mj = np.reshape(self.d.body_xmat[-1], (3,3))
        # ==========================================Kinematics: END===============================================
        pos_d, vel_d = P_EE, np.zeros((3,)) 
        quat_d, quatdot_d = quat_e, np.zeros((4,))
        # ========================================Trajectory: START===============================================
        if self.cnt == 0:
            self.generate_traj(P_EE, quat_e, points)
            self.cnt = 1
        
        for i in range(len(self.traj_time) - 1):
            if self.d.time <= self.traj_time[i + 1] and self.d.time > self.traj_time[i]:
                pos_d, vel_d, acc_d = self.linear_d[i].calculate_pva(self.d.time)
                quat_d, quatdot_d, quatddot_d = self.angular_d[i].calculate_pva_quat(self.d.time)
                self.step = i
        
        quat_d /= (np.linalg.norm(quat_d) + 10 ** (-8))

        # ==========================================Trajectory: END===============================================

        vel_CLIK = vel_d + 50 * (pos_d - P_EE)
        quatdot_CLIK = quatdot_d + 50 * (quat_d - quat_e)
        
        omega = Quat2Omega(quat_d, quatdot_CLIK)
        total = np.hstack((vel_CLIK, omega))
        null = np.eye(self.m.njnt) - np.linalg.pinv(J_pr) @ J_pr
        
        qvel_d = DLS_inverse(J_pr) @ np.reshape(total, (6,)) + null @ self.d.qvel
        
        # dxl.control_pos(d.qpos)
        self.qpos_d = self.qpos_d + qvel_d * self.m.opt.timestep

        # joint_torq = 2000 * (qpos_d - d.qpos) + 1300 * (qvel_d - d.qvel)
        joint_torq = 60 * (self.qpos_d - self.d.qpos) + 30 * (qvel_d - self.d.qvel)  # 200Hz  50, 50
        # joint_torq = 10 * (qpos_d - d.qpos) + 20 * (qvel_d - d.qvel)  # 100Hz  100, 100
        # joint_torq = 3 * (qpos_d - d.qpos) + 20 * (qvel_d - d.qvel) # 50Hz  3, 20 40 50 끝까지 흔들리긴함

        return joint_torq

    def remote_control(self, remote_qpos, remote_qvel):
        
        joint_torq = 60 * (remote_qpos - self.d.qpos) + 30 * (remote_qvel - self.d.qvel)
        if np.all(self.d.qpos < 2e-3):
            self.complete_a_cycle = True
        return joint_torq

    def go_init_point(self):
        init_pos = [0, 0, 0, 0, 0]
        init_pos[2] = 1e-3

        if self.cnt_init == 0:
            init_time = self.d.time
            final_time = init_time + 3
            self.init_qpos_d = Trajectory(init_time, final_time)
            self.init_qpos_d.get_coeff_qpos(self.d.qpos, init_pos)
            self.cnt_init += 1
        
        qpos_d, qvel_d, qacc_d = self.init_qpos_d.calculate_pva_qpos(self.d.time)

        joint_torq = 50 * (qpos_d - self.d.qpos) + 30 * (qvel_d - self.d.qvel)

        if np.all(self.d.qpos < 2e-3):
            self.complete_a_cycle = True

        return joint_torq
