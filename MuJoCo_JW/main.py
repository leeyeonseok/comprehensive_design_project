import mujoco_py
import os
from kinematic import *
from trajectory import Trajectory
import matplotlib.pyplot as plt
from dxl import Dynamixel
import time

xml_path = './MuJoCo_JW/model/scene.xml'

# 모델 로드
m = mujoco_py.load_model_from_path(xml_path)
sim = mujoco_py.MjSim(m)
d = sim.data
viewer = mujoco_py.MjViewer(sim)

#========================= Dynamixel load ==============================
# ids = [11, 12, 13, 14]
# dxl = Dynamixel(ids)
#========================= Define Parameters ===========================
traj_time = [4, 8, 12, 16]
cnt = 0
linear_d = [None] * len(traj_time)
angular_d = [None] * len(traj_time)
qpos = [None] * m.njnt
qvel = [None] * m.njnt
qpos = np.reshape(qpos, (m.njnt,))
qvel = np.reshape(qvel, (m.njnt,))
qpos_d = 0

#=========================== Plot ==================================
T, pd, pvd, qvd, qpd, pe, quatd, quatv = [], [], [], [], [], [], [], []


while d.time < traj_time[-1] + 2:
    # ========================================Kinematics: START===============================================
    K = Kinematic(sim)
    T.append(d.time)

    P_EE,R_EE,P_lnk,R_lnk = K.forward_kinematics(d.qpos)
    jnt_axes = K.get_jnt_axis()
    J_p, J_r = K.get_jacobian(P_lnk, jnt_axes, P_EE)
    J_pr = np.vstack((J_p, J_r))

    quat_e = Rot2Quat(R_EE)
    quat_e /= (np.linalg.norm(quat_e) + 10 ** (-8))
    
    quat_mj = d.body_xquat[-1]
    R_EE_mj = np.reshape(d.body_xmat[-1], (3,3))
    # ==========================================Kinematics: END===============================================
    
    # ========================================Trajectory: START===============================================
    if cnt == 0:
        linear_d[0] = Trajectory(0,traj_time[0])
        init_state = P_EE
        final_state = [0.168, 0, 0.1525]
        linear_d[0].get_coeff(init_state, final_state)

        linear_d[1] = Trajectory(traj_time[0], traj_time[1])
        init_state = linear_d[0].final_state
        final_state = [-0.202, 0.202, 0.19]
        linear_d[1].get_coeff(init_state, final_state)

        linear_d[2] = Trajectory(traj_time[1], traj_time[2])
        init_state = linear_d[1].final_state
        final_state = [0.191, 0.191, 0.18]
        linear_d[2].get_coeff(init_state, final_state)

        linear_d[3] = Trajectory(traj_time[2], traj_time[3])
        init_state = linear_d[2].final_state
        final_state = [-0.212, 0.212, 0.161]
        linear_d[3].get_coeff(init_state, final_state)
    # ========================================================================================================
        angular_d[0] = Trajectory(0,traj_time[0])
        init_state = quat_e
        final_rot = Rot_y(90)
        final_state = Rot2Quat(final_rot)
        angular_d[0].get_coeff_quat(init_state, final_state)

        angular_d[1] = Trajectory(traj_time[0],traj_time[1])
        init_state = angular_d[0].final_state
        final_rot = final_rot @ Rot_x(-135) # local coordinate
        final_state = Rot2Quat(final_rot)
        angular_d[1].get_coeff_quat(init_state, final_state)    

        angular_d[2] = Trajectory(traj_time[1],traj_time[2])
        init_state = angular_d[1].final_state
        final_rot = final_rot @ Rot_x(89) # local coordinate
        final_state = Rot2Quat(final_rot)
        angular_d[2].get_coeff_quat(init_state, final_state)  

        angular_d[3] = Trajectory(traj_time[2],traj_time[3])
        init_state = angular_d[2].final_state
        final_rot = final_rot @ Rot_x(-89) # local coordinate
        final_state = Rot2Quat(final_rot)
        angular_d[3].get_coeff_quat(init_state, final_state)  

        cnt = 1
    # ========================================================================================================

    if d.time <= traj_time[0]:
        pos_d, vel_d, acc_d = linear_d[0].calculate_pva(d.time)
        quat_d, quatdot_d, quatddot_d = angular_d[0].calculate_pva_quat(d.time)
    elif d.time <= traj_time[1]:
        pos_d, vel_d, acc_d = linear_d[1].calculate_pva(d.time)
        quat_d, quatdot_d, quatddot_d = angular_d[1].calculate_pva_quat(d.time)
    elif d.time <= traj_time[2]:
        pos_d, vel_d, acc_d = linear_d[2].calculate_pva(d.time)
        quat_d, quatdot_d, quatddot_d = angular_d[2].calculate_pva_quat(d.time)
    elif d.time <= traj_time[3]:
        pos_d, vel_d, acc_d = linear_d[3].calculate_pva(d.time)
        quat_d, quatdot_d, quatddot_d = angular_d[3].calculate_pva_quat(d.time)
    
    quat_d /= (np.linalg.norm(quat_d) + 10 ** (-8))
    
    # if np.dot(quat_d, quat_mj) < 1:
    #      quat_d = -quat_d
    
    # ==========================================Trajectory: END===============================================
    vel_CLIK = vel_d + 50 * (pos_d - P_EE)
    quatdot_CLIK = quatdot_d + 50 * (quat_d - quat_e)
    
    del_quat = mul_quat(quat_d, inverse_quat(quat_e))
    angle_error = 2 * np.arctan2(np.linalg.norm(del_quat[1:]), del_quat[0])  # 각도
    axis_error = del_quat[1:] / (np.linalg.norm(del_quat[1:]) + 10 ** (-5))  # 축
    omega_error = axis_error * angle_error
    
    omega = Quat2Omega(quat_d, quatdot_CLIK)
    total = np.hstack((vel_CLIK,omega))
    null = np.eye(4) - np.linalg.pinv(J_pr) @ J_pr
    
    qvel_d = DLS_inverse(J_pr) @ np.reshape(total, (6,)) + null @ d.qvel


    qpos_d = qpos_d + qvel_d * m.opt.timestep
    
    # joint_torq = 2000 * (qpos_d - d.qpos) + 1300 * (qvel_d - d.qvel)
    # joint_torq = 20 * (qpos_d - d.qpos) + 10 * (qvel_d - d.qvel)  # 200Hz  50, 50
    joint_torq = 20 * (qpos_d - d.qpos) + 10 * (qvel_d - d.qvel)  # 10Hz  100, 100
    # joint_torq = 10 * (qpos_d - d.qpos) + 20 * (qvel_d - d.qvel)  # 100Hz  100, 100
    # joint_torq = 3 * (qpos_d - d.qpos) + 20 * (qvel_d - d.qvel) # 50Hz  3, 20 40 50 끝까지 흔들리긴함

    pd.append(pos_d)
    pvd.append(vel_d)
    qvd.append(qvel_d)
    qpd.append(qpos_d)
    pe.append(d.body_xpos[-1])
    quatd.append(quat_d)
    # tq0.append(joint_torq[0])
    # tq1.append(joint_torq[1])
    # tq2.append(joint_torq[2])
    # tq3.append(joint_torq[3])
    quatv.append(quatdot_d)

    print("===============================================================================================================================")
    print(d.time, "\t", "joint_torque : ", joint_torq)
    print(d.time, "\t", "P_EE : ", d.body_xpos[-1], "\t", P_EE)   
    print(d.time, "\t", "qpos_d : ", qpos_d)
    print(d.time, "\t", "pos_d - P_EE : ", pos_d - d.body_xpos [-1])
    print(d.time, "\t", "quat_e : ", quat_mj, "\t", quat_e) 
    print(d.time, "\t", "Rot_EE : ", R_EE)
    print(d.time, "\t", "Rot_mj : ", R_EE_mj, linear_d[1].excuted_once)
    print(d.time, "\t", "quat : ", Rot2Quat(Rot_y(90)), Rot2Quat(Rot_y(90) @ Rot_x(-135)), Rot2Quat(Rot_y(90) @ Rot_x(-135) @ Rot_x(-90)))
    # print(d.time, "\t", "current : ", current)
    # print(d.time, "\t", "qpos_dxl : ", dxl[3].get_qpos(), d.qpos, dxl[3].get_qvel(), d.qvel[3])
    print("===============================================================================================================================")

    for i in range(m.nu):
        d.ctrl[i] = joint_torq[i]

    sim.step()
    viewer.render()

# for i in range(4):
#     dxl[i].close_port()

plt.subplot(321)
plt.plot(T,pe)
plt.title("P_EE")
plt.grid()

plt.subplot(322)
plt.plot(T,qvd)
plt.title("qvel_d")
plt.grid()

plt.subplot(323)
plt.plot(T,qpd)
plt.title("qpos_d")
plt.grid()

plt.subplot(324)
plt.plot(T,pvd)
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

# plt.plot(T,tq0, label="J1")
# plt.plot(T,tq1, label="J2")
# plt.plot(T,tq2, label="J3")
# plt.plot(T,tq3, label="J4")
# plt.title("joint torq")
# plt.legend(loc='best')

plt.show()

