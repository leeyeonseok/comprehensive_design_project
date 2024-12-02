import numpy as np
from scipy.spatial.transform import Rotation as R
from functions import *

class Trajectory:
    def __init__(self, init_time, time_to_reach_goal):
        self.final_time = time_to_reach_goal
        self.init_time = init_time
        self.excuted_once = False

    def time_mat(self, current_time):
        tmat = np.array([[   current_time**5,    current_time**4,   current_time**3,   current_time**2, current_time**1, 1],
                         [ 5*current_time**4,  4*current_time**3, 3*current_time**2, 2*current_time**1,               1, 0],
                         [20*current_time**3, 12*current_time**2, 6*current_time**1,                 2,               0, 0]])
        
        return tmat
    
    def get_coeff(self, init_state, final_state):
        if self.excuted_once == False:
            self.final_state = final_state
            mat = np.array([[    self.init_time**5,     self.init_time**4,    self.init_time**3,    self.init_time**2,  self.init_time**1, 1],
                            [  5*self.init_time**4,   4*self.init_time**3,  3*self.init_time**2,  2*self.init_time**1,                  1, 0],
                            [ 20*self.init_time**3,  12*self.init_time**2,  6*self.init_time**1,                    2,                  0, 0],
                            [   self.final_time**5,    self.final_time**4,   self.final_time**3,   self.final_time**2, self.final_time**1, 1],
                            [ 5*self.final_time**4,  4*self.final_time**3, 3*self.final_time**2, 2*self.final_time**1,                  1, 0],
                            [20*self.final_time**3, 12*self.final_time**2, 6*self.final_time**1,                    2,                  0, 0]])
            
            target = np.array([[      init_state[0],       init_state[1],       init_state[2]],
                               [                  0,                   0,                   0],
                               [                  0,                   0,                   0],
                               [self.final_state[0], self.final_state[1], self.final_state[2]],
                               [                  0,                   0,                   0],
                               [                  0,                   0,                   0]])
            
            self.coeff = np.linalg.inv(mat) @ target    # 6x3 matrix
            self.excuted_once = True


    def calculate_pva(self, current_time):   # calculate pose, velocity, acceleration       
        if current_time <= self.final_time:
            pva = self.time_mat(current_time) @ self.coeff    # 3x3 matrix
        else:
            pva = self.time_mat(self.final_time) @ self.coeff
        
        pos = pva[0,:]
        vel = pva[1,:]
        acc = pva[2,:]
            
        return pos, vel, acc
    
    def get_coeff_quat(self, init_state, final_state):
        if self.excuted_once == False:
            self.angle_diff = mul_quat(inverse_quat(init_state), final_state)
            self.final_state = final_state

            if np.dot(init_state, self.final_state) < 0:
                self.final_state = -final_state
                self.angle_diff = mul_quat(inverse_quat(init_state), self.final_state)
            
            mat = np.array([[    self.init_time**5,     self.init_time**4,    self.init_time**3,    self.init_time**2,  self.init_time**1, 1],
                            [  5*self.init_time**4,   4*self.init_time**3,  3*self.init_time**2,  2*self.init_time**1,                  1, 0],
                            [ 20*self.init_time**3,  12*self.init_time**2,  6*self.init_time**1,                    2,                  0, 0],
                            [   self.final_time**5,    self.final_time**4,   self.final_time**3,   self.final_time**2, self.final_time**1, 1],
                            [ 5*self.final_time**4,  4*self.final_time**3, 3*self.final_time**2, 2*self.final_time**1,                  1, 0],
                            [20*self.final_time**3, 12*self.final_time**2, 6*self.final_time**1,                    2,                  0, 0]])
            
            target = np.array([[      init_state[0],       init_state[1],       init_state[2],       init_state[3]],
                               [                  0,                   0,                   0,                   0],
                               [                  0,                   0,                   0,                   0],
                               [self.final_state[0], self.final_state[1], self.final_state[2], self.final_state[3]],
                               [                  0,                   0,                   0,                   0],
                               [                  0,                   0,                   0,                   0]])
            
            self.coeff_quat = np.linalg.inv(mat) @ target    # 6x4 matrix
            self.excuted_once = True

    def calculate_pva_quat(self, current_time):   # calculate pose, velocity, acceleration
        if current_time <= self.final_time:
            pva = self.time_mat(current_time) @ self.coeff_quat    # 3x4 matrix 
        else:
            pva = self.time_mat(self.final_time) @ self.coeff_quat

        pos = pva[0,:]
        vel = pva[1,:]
        acc = pva[2,:]
        
        return pos, vel, acc
    
    def get_coeff_qpos(self, init_state, final_state):
        if self.excuted_once == False:
            self.final_state = final_state
            mat = np.array([[    self.init_time**5,     self.init_time**4,    self.init_time**3,    self.init_time**2,  self.init_time**1, 1],
                            [  5*self.init_time**4,   4*self.init_time**3,  3*self.init_time**2,  2*self.init_time**1,                  1, 0],
                            [ 20*self.init_time**3,  12*self.init_time**2,  6*self.init_time**1,                    2,                  0, 0],
                            [   self.final_time**5,    self.final_time**4,   self.final_time**3,   self.final_time**2, self.final_time**1, 1],
                            [ 5*self.final_time**4,  4*self.final_time**3, 3*self.final_time**2, 2*self.final_time**1,                  1, 0],
                            [20*self.final_time**3, 12*self.final_time**2, 6*self.final_time**1,                    2,                  0, 0]])
            
            target = np.array([[      init_state[0],       init_state[1],       init_state[2],       init_state[0],       init_state[1]],
                               [                  0,                   0,                   0,                   0,                   0],
                               [                  0,                   0,                   0,                   0,                   0],
                               [self.final_state[0], self.final_state[1], self.final_state[2], self.final_state[0], self.final_state[1]],
                               [                  0,                   0,                   0,                   0,                   0],
                               [                  0,                   0,                   0,                   0,                   0]])
            
            self.coeff_qpos = np.linalg.inv(mat) @ target    # 6x5 matrix
            self.excuted_once = True
        
    def calculate_pva_qpos(self, current_time):   # calculate pose, velocity, acceleration
        if current_time <= self.final_time:
            pva = self.time_mat(current_time) @ self.coeff_qpos    # 3x4 matrix 
        else:
            pva = self.time_mat(self.final_time) @ self.coeff_qpos

        pos = pva[0,:]
        vel = pva[1,:]
        acc = pva[2,:]
        
        return pos, vel, acc


'''     발표용
traj = Trajectory(4)
init_state = [0,0,0]
final_state = [4,5,2]
traj.get_coeff(init_state,final_state)
t = np.linspace(0,4,4000)
pos = []
vel = []
acc = []
for i in t:
    p,v,a = traj.calculate_pva(i)
    pos.append(p)
    vel.append(v)
    acc.append(a)

plt.subplot(311)
plt.plot(t,pos)
plt.title("pos")
plt.grid()

plt.subplot(312)
plt.plot(t,vel)
plt.title("vel")
plt.grid()

plt.subplot(313)
plt.plot(t,acc)
plt.title("acc")
plt.grid()

plt.show()
'''