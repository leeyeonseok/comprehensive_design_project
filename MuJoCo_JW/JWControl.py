from functions import *
from kinematic import Kinematic
from trajectory import Trajectory

cnt = 0

class JWControl:
    def __init__(self, sim, time):
        self.sim = sim
        self.time = time
        self.Control()

    def Control(self):
        d = self.sim.data
        m = self.sim.model
        P_EE, R_EE, P_lnk, R_lnk, J_p, J_r = self.cal_forward(m,d)
        
        Jp_flat = d.get_body_jacp("fingers").reshape(3, m.nv)  # 선형 자코비안 (3 x nv)
        Jr_flat = d.get_body_jacr("fingers").reshape(3, m.nv)  # 각 자코비안 (3 x nv)

        linear_d = Trajectory(4)
        init_state = P_EE
        final_state = [1.5, 0, 2.1]
        if self.time == 0.005:
            linear_d.get_coeff(init_state, final_state)

        pos_d, vel_d, acc_d = linear_d.calculate_pva(self.time)
        vel_CLIK = vel_d + 10 * (pos_d - P_EE)
    
        qvel_d = np.linalg.pinv(J_p) @ vel_CLIK
        qpos_d += qvel_d * self.time


        print("==========================================")
        print("time : ",self.sim.option.timestep)
        print("P_EE = ",d.site_xpos)
        print("JW P_EE = ", P_EE)
        print("R_EE = ",quat2rot(d.body_xquat[-1]))
        print("JW R_EE = ", R_EE)
        print("Linear_Jacobian_JW : ",J_p)
        print("Linear_Jacobian_MJ : ",Jp_flat)
        print("Angular_Jacobian_JW : ",J_r)
        print("Angular_Jacobian_MJ : ",Jr_flat)
        for i in range(3):
            print("JW P_lnk : ",P_lnk[i+1])
            print("real P_lnk : ",d.body_xpos[i+2])
            print("JW R_lnk : ",R_lnk[i+1])
            print("real R_lnk : ", quat2rot(d.body_xquat[i+2]))
        print("==========================================")

        d.ctrl[:] = 0
        d.ctrl[1:2] = 10*(pi/6 - d.qpos[3])
    
    def cal_forward(self, m ,d):
        K = Kinematic(self.sim)

        P_EE,R_EE,P_lnk,R_lnk = K.forward_kinematics(d.qpos)
        jnt_axes = K.get_jnt_axis()
        J_p, J_r = K.get_jacobian(P_lnk, jnt_axes, P_EE)

        return P_EE, R_EE, P_lnk, R_lnk, J_p, J_r