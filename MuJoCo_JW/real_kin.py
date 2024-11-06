import mujoco_py
from functions import *
from scipy.linalg import expm
from math import pi

class Kinematic:    
    def get_first_wv(self):
        """
        실제 로봇의 w,v 를 넣으면 됨
        """
        w,q = [], []
        # Simulation
        # for i in range(self.model.nbody - 2): # except gripper
        #     w.append(self.data.xaxis[i])
        
        # for i in range(1, self.model.nbody - 1):
        #     q.append(self.data.body_xpos[i])
        
        #==================Open4DOF========================
        w = np.array([[0, 0, 1],
                      [0,-1, 0],
                      [0,-1, 0],
                      [0,-1, 0]])
        
        q = np.array([np.array([0, 0, 0]).T,
                      np.array([0, 0, 0.0765]).T,
                      np.array([0, 0, 0.0765]).T + Rot_x(90) @ Rot_z(90) @ np.array([0.024, 0.128, 0]).T,
                      np.array([0, 0, 0.0765]).T + Rot_x(90) @ Rot_z(90) @ np.array([0.024, 0.128, 0]).T + Rot_x(90) @ Rot_z(90) @ Rot_z(-180) @ np.array([0, 0.124, 0]).T])

        #==================Open5DOF========================
        # w = np.array([[ 0, 0, 1],
        #               [ 0,-1, 0],
        #               [ 0,-1, 0],
        #               [ 0,-1, 0],
        #               [ 0, 0, 1]])
        
        # q = np.array([np.array([0, 0, 0]).T,
        #               np.array([0, 0, 0.306]).T,
        #               np.array([0, 0, 0.306]).T + Rot_x(90) @ Rot_z(40) @ np.array([0.096, 0.512, 0]).T,
        #               np.array([0, 0, 0.306]).T + Rot_x(90) @ Rot_z(40) @ np.array([0.096, 0.512, 0]).T + Rot_x(90) @ Rot_z(40) @ Rot_z(-40) @ np.array([0, 0.496, 0]).T,
        #               np.array([0, 0, 0.306]).T + Rot_x(90) @ Rot_z(40) @ np.array([0.096, 0.512, 0]).T + Rot_x(90) @ Rot_z(40) @ Rot_z(-40) @ np.array([0, 0.496, 0]).T + Rot_x(90) @ Rot_z(40) @ Rot_z(-40) @ Rot_z(-40) @ np.array([0, 0.12, 0]).T])
        
        v = -np.cross(w, q)
        return w,v

    def exp_twist(self, i, xi, theta):
        omega = xi[:3]
        v = xi[3:]
        omega_norm = np.linalg.norm(omega)


        omega_hat = skew(omega)
        R = (np.eye(3) +
                np.sin(omega_norm * theta) / omega_norm * omega_hat +
                (1 - np.cos(omega_norm * theta)) / (omega_norm ** 2) * np.dot(omega_hat, omega_hat))
        
        p = (np.eye(3) * theta +
                (1 - np.cos(omega_norm * theta)) / (omega_norm ** 2) * omega_hat +
                (theta - np.sin(omega_norm * theta) / omega_norm) / (omega_norm ** 2) * np.dot(omega_hat, omega_hat)).dot(v)

        return np.vstack((np.hstack((R, np.reshape(p, (3, 1)))), [0, 0, 0, 1]))

    def forward_kinematics(self, q):
        """
        q: 관절 각도 배열
        """
        
        #=================Open4DOF=========================
        T_ = []
        T_.append(np.eye(4))

        T_.append(np.eye(4))
        T_[1][2,3] = 0.0765
        T_[1][:3,:3] = Rot_x(90) @ Rot_z(90) 

        T_.append(np.eye(4))
        z = np.array([[0.024,0.128,0]])
        T_[2][:3, 3] = T_[1][:3, 3] + T_[1][:3,:3] @ np.reshape(z,(3,))
        T_[2][:3,:3] = T_[1][:3,:3] @ Rot_z(-180)

        T_.append(np.eye(4))
        z = np.array([[0,0.124,0]])
        T_[3][:3,3] = T_[2][:3, 3] + T_[2][:3,:3] @ np.reshape(z,(3,))
        T_[3][:3,:3] = T_[2][:3,:3]

        T_.append(np.eye(4))
        z = np.array([[0,0.1066,0]])
        T_[4][:3,3] = T_[3][:3, 3] + T_[3][:3,:3] @ np.reshape(z,(3,))
        T_[4][:3,:3] = T_[3][:3,:3] @ Rot_x(-90)
        
        #=================Open5DOF=========================
        # T_ = []
        # T_.append(np.eye(4))

        # T_.append(np.eye(4))
        # T_[1][2,3] = 0.306
        # T_[1][:3,:3] = Rot_x(90) @ Rot_z(40) 

        # T_.append(np.eye(4))
        # z = np.array([[0.096,0.512,0]])
        # T_[2][:3, 3] = T_[1][:3, 3] + T_[1][:3,:3] @ np.reshape(z,(3,))
        # T_[2][:3,:3] = T_[1][:3,:3] @ Rot_z(-40)

        # T_.append(np.eye(4))
        # z = np.array([[0,0.496,0]])
        # T_[3][:3,3] = T_[2][:3, 3] + T_[2][:3,:3] @ np.reshape(z,(3,))
        # T_[3][:3,:3] = T_[2][:3,:3] @ Rot_z(-40)

        # T_.append(np.eye(4))
        # z = np.array([[0,0.12,0]])
        # T_[4][:3,3] = T_[3][:3, 3] + T_[3][:3,:3] @ np.reshape(z,(3,))
        # T_[4][:3,:3] = T_[3][:3,:3] @ Rot_x(-90)

        # T_.append(np.eye(4))
        # z = np.array([[0,0,0.2096]])
        # T_[5][:3,3] = T_[4][:3, 3] + T_[4][:3,:3] @ np.reshape(z,(3,))
        # T_[5][:3,:3] = T_[4][:3,:3]
        
        w,v = self.get_first_wv()

        # 각 joint의 twist 벡터를 정의 (실제 로봇 돌릴 때, 여기 수정)
        twists = np.hstack((w,v))

        # POE 방식에 따른 forward kinematics 계산
        T = np.eye(4)
        for i, xi in enumerate(twists):
            T = np.dot(T, self.exp_twist(i, xi, q[i]))
            T_[i + 1] = np.dot(T,T_[i + 1])

        self.R_lnk = []
        self.P_lnk = []
        for i in range(4 + 1):
            self.R_lnk.append(T_[i][:3,:3])
            self.P_lnk.append(T_[i][:3,3])

        self.P_EE = self.P_lnk[-1]
        self.R_EE = self.R_lnk[-1]

        return self.P_EE, self.R_EE, self.P_lnk, self.R_lnk
    
    def get_jnt_axis(self):
        w,v = self.get_first_wv()

        joint_axis = np.zeros((4, 3))

        for i in range(4):
            joint_axis[i,:] = self.R_lnk[i] @ np.reshape(w[0],(3,))

        return joint_axis

    def get_jacobian(self, joint_positions, joint_axes, end_effector_pos):
        """
        자코비안 계산 함수
        joint_positions: 각 관절의 위치 리스트 (Nx3 배열) -> P_lnk
        joint_axes: 각 관절의 회전축 리스트 (Nx3 배열) -> R_lnk*w
        end_effector_pos: 엔드 이펙터 위치 (3x1 벡터) -> P_EE
        joint_types: 관절 타입 리스트 (0=프리스매틱, 1=회전) -> m.jnt_type
        """
        num_joints = 4
        
        # 선형 및 각 자코비안 초기화 (6 x N 자코비안)
        J_p = np.zeros((3, num_joints))
        J_r = np.zeros((3, num_joints))
        
        for i in range(num_joints):
            joint_pos = joint_positions[i]
            joint_axis = joint_axes[i]

            J_p[:, i] = np.cross(joint_axis, (end_effector_pos - joint_pos))
            J_r[:, i] = joint_axis
        
        return J_p, J_r
    


