import mujoco_py
import os
from functions import *
from kinematic import Kinematic
from trajectory import Trajectory
import matplotlib.pyplot as plt
from dxl import Dynamixel
from scipy.spatial.transform import Rotation as R
import copy

xml_path = './model/scene.xml'

# 모델 로드
m = mujoco_py.load_model_from_path(xml_path)
sim = mujoco_py.MjSim(m)
d = sim.data
viewer = mujoco_py.MjViewer(sim)
# Dynamixel load
# dxl = Dynamixel(id=2)

#=========================Define Parameters===========================
cnt = 0
traj_time = [4, 8, 12]
qpos_d = 0
linear_d = [None] * len(traj_time)
angular_d = [None] * len(traj_time)

#===========================Plot==================================
qp=[]
qv=[]
pd=[]
quatd=[]
quatv=[]
pv=[]
pe=[]
T=[]

while d.time < traj_time[-1] + 2:
    # ========================================Kinematics: START===============================================
    K = Kinematic(sim)
    T.append(d.time)
    P_EE,R_EE,P_lnk,R_lnk = K.forward_kinematics(d.qpos)
    jnt_axes = K.get_jnt_axis()
    J_p, J_r = K.get_jacobian(d.body_xpos[1:], d.xaxis, d.body_xpos[-1])
    J_pr = np.vstack((J_p, J_r))

    Jp_flat = d.get_body_jacp("fingers").reshape(3, m.nv)  # 선형 자코비안 (3 x nv)
    Jr_flat = d.get_body_jacr("fingers").reshape(3, m.nv)  # 각 자코비안 (3 x nv)

    euler_e = Rot2EulerZXY(R_EE)
    quat_e = Rot2Quat(R_EE)
    quat_e /= (np.linalg.norm(quat_e) + 10 ** (-8))
    quat_mj = d.body_xquat[-1]
    quat_mj /= (np.linalg.norm(quat_mj) + 10 ** (-8))
    R_EE_mj = np.reshape(d.body_xmat[-1], (3,3))
    euler_mj = Rot2EulerZXY(R_EE_mj)
    # ==========================================Kinematics: END===============================================
    
    # ========================================Trajectory: START===============================================
       

    if cnt == 0:
        linear_d[0] = Trajectory(0,traj_time[0])
        init_state = d.body_xpos[-1]
        final_state = [0.705, 0, 1.15]  # 보정값 필요
        linear_d[0].get_coeff(init_state, final_state)

        linear_d[1] = Trajectory(traj_time[0], traj_time[1])
        init_state = copy.deepcopy(final_state)
        final_state = [0, 0.705, 0.95]  # 보정값 필요
        linear_d[1].get_coeff(init_state, final_state)

        linear_d[2] = Trajectory(traj_time[1], traj_time[2])
        init_state = copy.deepcopy(final_state)
        final_state = [-0.605, 0, 1.05]  # 보정값 필요
        linear_d[2].get_coeff(init_state, final_state)

        angular_d[0] = Trajectory(0,traj_time[0])
        init_state = quat_mj
        final_state = np.array([np.cos(pi/4), 0, np.sin(pi/4), 0])
        final_rot = Rot_y(90)
        final_state = Rot2Quat(final_rot)
        angular_d[0].get_coeff_quat(init_state, final_state)

        angular_d[1] = Trajectory(traj_time[0],traj_time[1])
        init_state = copy.deepcopy(final_state)
        final_rot = Rot_y(90) @ Rot_x(-90) # local coordinate
        final_state = Rot2Quat(final_rot)
        angular_d[1].get_coeff_quat(init_state, final_state)    

        angular_d[2] = Trajectory(traj_time[1],traj_time[2])
        init_state = copy.deepcopy(final_state)
        final_rot = Rot_y(90) @ Rot_x(-179.9) # local coordinate
        final_state = Rot2Quat(final_rot)
        angular_d[2].get_coeff_quat(init_state, final_state)  

        cnt = 1

    if d.time < traj_time[0] :
        pos_d, vel_d, acc_d = linear_d[0].calculate_pva(d.time)
        quat_d, quatdot_d, quatddot_d = angular_d[0].calculate_pva_quat(d.time)
    elif d.time < traj_time[1]:
        pos_d, vel_d, acc_d = linear_d[1].calculate_pva(d.time)
        quat_d, quatdot_d, quatddot_d = angular_d[1].calculate_pva_quat(d.time)
    elif d.time < traj_time[2]:
        pos_d, vel_d, acc_d = linear_d[2].calculate_pva(d.time)
        quat_d, quatdot_d, quatddot_d = angular_d[2].calculate_pva_quat(d.time)

    
    quat_d /= (np.linalg.norm(quat_d) + 10 ** (-8))
    
    # if np.dot(quat_d, quat_mj) < 1:
    #     quat_d = -quat_d
    
    # ==========================================Trajectory: END===============================================
    vel_CLIK = vel_d + 100 * (pos_d - d.body_xpos[-1])
    quatdot_CLIK = quatdot_d + 100 * (quat_d - quat_mj)
    del_quat = mul_quat(quat_d, inverse_quat(quat_mj))
    angle_error = 2 * np.arctan2(np.linalg.norm(del_quat[1:]), del_quat[0])  # 각도
    axis_error = del_quat[1:] / (np.linalg.norm(del_quat[1:]) + 10 ** (-5))  # 축
    omega_error = axis_error * angle_error
    
    omega = Quat2Omega(quat_d, quatdot_CLIK)
    total = np.hstack((vel_CLIK,omega))
    null = np.eye(4) - np.linalg.pinv(J_pr) @ J_pr
    # qvel_null = np.full(m.njnt, 10)
    # qvel_null[3] = 0.1  
    
    qvel_d = DLS_inverse(J_pr) @ np.reshape(total, (6,)) + 2 * null @ d.qvel


    qpos_d = qpos_d + qvel_d * m.opt.timestep
    
    # joint_torq = 2000 * (qpos_d - d.qpos) + 1300 * (qvel_d - d.qvel)
    # joint_torq = 100 * (qpos_d - d.qpos) + 50 * (qvel_d - d.qvel)  # 200Hz  50, 50
    joint_torq = 20 * (qpos_d - d.qpos) + 30 * (qvel_d - d.qvel)  # 100Hz  100, 100
    # joint_torq = 3 * (qpos_d - d.qpos) + 20 * (qvel_d - d.qvel) # 50Hz  3, 20 40 50 끝까지 흔들리긴함

    #current = 100 * (qpos_d[2] - dxl.get_qpos())
    #dxl.control(current)

    pd.append(pos_d)
    pv.append(vel_d)
    qv.append(qvel_d)
    qp.append(qpos_d)
    pe.append(P_EE)
    quatd.append(quat_d)
    quatv.append(quatdot_d)


    print("==========================================")
    print(d.time, "\t", "joint_torque : ", joint_torq)
    print(d.time, "\t", "P_EE : ", d.body_xpos[-1] ,P_EE)
    print(d.time, "\t", "qpos_d - d.qpos : ", qpos_d - d.qpos)
    print(d.time, "\t", "pos_d - P_EE : ", pos_d - d.body_xpos[-1])
    print(d.time, "\t", "omega_error : ", omega_error)
    print(d.time, "\t", "quat_e : ", quat_e, "\t", quat_mj) 
    print(d.time, "\t", "Rot_EE : ", R_EE)
    print(d.time, "\t", "Rot_mj : ", R_EE_mj)
    print("==========================================")

    for i in range(m.nu):
        d.ctrl[i] = joint_torq[i]
    
    sim.step()
    viewer.render()

#dxl.close_port()

plt.subplot(321)
plt.plot(T,qp)
plt.title("qpos")
plt.grid()

plt.subplot(322)
plt.plot(T,qv)
plt.title("qvel")
plt.grid()

plt.subplot(323)
plt.plot(T,pd)
plt.title("pos_d")
plt.grid()

plt.subplot(324)
plt.plot(T,pv)
plt.title("vel_d")
plt.grid()

plt.subplot(325)
plt.plot(T,quatd)
plt.title("quat_d")
plt.grid()

plt.subplot(326)
plt.plot(T,quatv)
plt.title("quatdot_d")
plt.grid()

plt.show()

