from dxl import Dynamixel
from trajectory import Trajectory
from functions import *
from real_kin import Kinematic
import mujoco_py
import time
import matplotlib.pyplot as plt
# import rospy
# from std_msgs.msg import Float32MultiArray

xml_path = './MuJoCo_JW/model/scene.xml'
m = mujoco_py.load_model_from_path(xml_path)
sim = mujoco_py.MjSim(m)
d = sim.data

jnt = 4
dxl_ids = [11, 12, 13, 14]
dxl = Dynamixel(dxl_ids)
joint_torq = np.zeros(jnt)

traj_time = [4, 8, 12, 16]
linear_d = [None] * len(traj_time)
angular_d = [None] * len(traj_time)
qpos = [0] * jnt
qvel = [0] * jnt

T, pd, pvd, qvd, qpd, pe, quatd, quatv = [], [], [], [], [], [], [], []

qpos_d = np.zeros(jnt)
cnt, t = 0, 0
timestep = 0.005

K = Kinematic()

# rospy.init_node('dynamixel_controller', anonymous=True)

# def position_callback(msg):
#     global qpos
#     qpos = msg.data

# torque_pub = rospy.Publisher('/dynamixel/torque', Float32MultiArray, queue_size=10)
# position_sub = rospy.Subscriber('/dynamixel/position', Float32MultiArray, position_callback)

while t < traj_time[-1] + 2:
    start_time = time.time()
    
    # qpos = dxl.get_qpos()
    # qvel = dxl.get_qvel()

    P_EE,R_EE,P_lnk,R_lnk = K.forward_kinematics(d.qpos)
    jnt_axes = K.get_jnt_axis()
    J_p, J_r = K.get_jacobian(P_lnk, jnt_axes, P_EE)
    J_pr = np.vstack((J_p, J_r))

    quat_e = Rot2Quat(R_EE)
    quat_e /= (np.linalg.norm(quat_e) + 1e-8)

    if cnt == 0:
        linear_d[0] = Trajectory(0,traj_time[0]-1)
        init_state = P_EE
        final_state = [0.248, 0, 0.1725]
        linear_d[0].get_coeff(init_state, final_state)

        linear_d[1] = Trajectory(traj_time[0], traj_time[1]-1)
        init_state = linear_d[0].final_state
        final_state = [-0.182, 0.182, 0.19]
        linear_d[1].get_coeff(init_state, final_state)

        linear_d[2] = Trajectory(traj_time[1], traj_time[2]-1)
        init_state = linear_d[1].final_state
        final_state = [0.191, 0.191, 0.18]
        linear_d[2].get_coeff(init_state, final_state)

        linear_d[3] = Trajectory(traj_time[2], traj_time[3]-1)
        init_state = linear_d[2].final_state
        final_state = [-0.232, 0.232, 0.161]
        linear_d[3].get_coeff(init_state, final_state)
    # ========================================================================================================
        angular_d[0] = Trajectory(0,traj_time[0]-1)
        init_state = quat_e
        final_rot = Rot_y(90)
        final_state = Rot2Quat(final_rot)
        angular_d[0].get_coeff_quat(init_state, final_state)

        angular_d[1] = Trajectory(traj_time[0],traj_time[1]-1)
        init_state = angular_d[0].final_state
        final_rot = final_rot @ Rot_x(-135) # local coordinate
        final_state = Rot2Quat(final_rot)
        angular_d[1].get_coeff_quat(init_state, final_state)    

        angular_d[2] = Trajectory(traj_time[1],traj_time[2]-1)
        init_state = angular_d[1].final_state
        final_rot = final_rot @ Rot_x(89) # local coordinate
        final_state = Rot2Quat(final_rot)
        angular_d[2].get_coeff_quat(init_state, final_state)  

        angular_d[3] = Trajectory(traj_time[2],traj_time[3]-1)
        init_state = angular_d[2].final_state
        final_rot = final_rot @ Rot_x(-89) # local coordinate
        final_state = Rot2Quat(final_rot)
        angular_d[3].get_coeff_quat(init_state, final_state) 

        cnt = 1

    if t <= traj_time[0]:
        pos_d, vel_d, acc_d = linear_d[0].calculate_pva(t)
        quat_d, quatdot_d, quatddot_d = angular_d[0].calculate_pva_quat(t)
    elif t <= traj_time[1]:
        pos_d, vel_d, acc_d = linear_d[1].calculate_pva(t)
        quat_d, quatdot_d, quatddot_d = angular_d[1].calculate_pva_quat(t)
    elif t <= traj_time[2]:
        pos_d, vel_d, acc_d = linear_d[2].calculate_pva(t)
        quat_d, quatdot_d, quatddot_d = angular_d[2].calculate_pva_quat(t)
    elif t <= traj_time[3]:
        pos_d, vel_d, acc_d = linear_d[3].calculate_pva(t)
        quat_d, quatdot_d, quatddot_d = angular_d[3].calculate_pva_quat(t)
    
    quat_d /= (np.linalg.norm(quat_d) + 1e-8)

    vel_CLIK = vel_d + 50 * (pos_d - P_EE)
    quatdot_CLIK = quatdot_d + 50 * (quat_d - quat_e)

    omega = Quat2Omega(quat_d, quatdot_CLIK)
    total = np.hstack((vel_CLIK,omega))
    null = np.eye(jnt) - np.linalg.pinv(J_pr) @ J_pr
    
    qvel_d = DLS_inverse(J_pr) @ np.reshape(total, (6,)) + null @ qvel
    qpos_d = qpos_d + qvel_d * m.opt.timestep

    joint_torq = 10 * (qpos_d - d.qpos) + 5 * (qvel_d - d.qvel)
    # joint_torq = [10 * (qpos_d[i] - qpos[i]) + 5 * (qvel_d[i] - qvel[i]) for i in range(jnt)]
    # torque_pub.publish(Float32MultiArray(data=joint_torq))

    if abs(qpos_d - qpos).any() > pi/3 and t > 1:
        break

    # dxl.control_pos(1 * qpos_d)
    dxl.control_torque(joint_torq)

    T.append(t)
    pd.append(pos_d)
    pvd.append(vel_d)
    qvd.append(qvel_d)
    qpd.append(qpos_d)
    pe.append(P_EE)
    quatd.append(quat_d)
    quatv.append(quatdot_d)

    print("===============================================================================================================================")
    print("time : ",t, "\t", "joint_torq : ", joint_torq)
    print("time : ",t, "\t", "P_EE  : ", P_EE)
    print("time : ",t, "\t", "pos_d : ", pos_d)
    print("time : ",t, "\t", "qpos   : ", qpos)
    print("time : ",t, "\t", "qpos_d : ", qpos_d)
    print("===============================================================================================================================")

    t += timestep
    print('time: ', time.time() - start_time)
    for i in range(m.nu):
        d.ctrl[i] = joint_torq[i]
    
    sim.step()
    

dxl.close_port()

plt.subplot(321)
plt.plot(T,pe)
plt.title("P_EE")
plt.grid()

plt.subplot(322)
plt.plot(T,qvd)
plt.title("qvel_d")
plt.grid()

plt.subplot(323)
plt.plot(T,pd)
plt.title("pos_d")
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

plt.show()