import numpy as np
from math import pi
from scipy.spatial.transform import Rotation as R

def Rot_x(q):
    q = np.deg2rad(q)
    r = np.array([[1,           0,          0],
                  [0,   np.cos(q), -np.sin(q)],
                  [0,   np.sin(q),  np.cos(q)]])
    return r

def Rot_y(q):
    q = np.deg2rad(q)
    r = np.array([[ np.cos(q),      0,  np.sin(q)],
                  [         0,      1,          0],
                  [-np.sin(q),      0,  np.cos(q)]])
    return r

def Rot_z(q):
    q = np.deg2rad(q)
    r = np.array([[np.cos(q),  -np.sin(q),          0],
                  [np.sin(q),   np.cos(q),          0],
                  [        0,           0,          1]])
    return r


def skew(vec):     
    skew = np.array([[0, -vec[2], vec[1]],
                     [vec[2], 0, -vec[0]],
                     [-vec[1], vec[0], 0]])
    return skew


def Rot2EulerZYZ(rot):
    '''
    Euler Angle Formulas
    @article{eberly2008euler,
    title = { Euler angle formulas },
    author = { Eberly, David },
    journal = { Geometric Tools, LLC, Technical Report },
    pages = { 1--18 },
    year = { 2008 }
    }
    '''
    euler = np.zeros(shape=(3,))
    if rot[2,2] < 1:
        if rot[2,2] > -1:
            euler[0] = np.atan2(rot[1,2], rot[0,2])
            euler[1] = np.acos(rot[2,2])
            euler[2] = np.atan2(rot[2,1], -rot[2,0])
        else:
            euler[0] = -np.atan2(rot[1,0], rot[1,1])
            euler[1] = pi
            euler[2] = 0
    else:
        euler[0] = np.atan2(rot[1,0], rot[1,1])
        euler[1] = 0
        euler[2] = 0
        
    return euler.T

def Rot2EulerZXY(rot):
    '''
    Euler Angle Formulas
    @article{eberly2008euler,
    title = { Euler angle formulas },
    author = { Eberly, David },
    journal = { Geometric Tools, LLC, Technical Report },
    pages = { 1--18 },
    year = { 2008 }
    }
    '''
    euler = np.zeros((3,))
    if rot[2,1] < 1:
        if rot[2,1] > -1:
            euler[0] = np.atan2(-rot[0,1], rot[1,1])
            euler[1] = np.asin(rot[2,1])
            euler[2] = np.atan2(-rot[2,0], rot[2,2])
        else:
            euler[0] = -np.atan2(rot[0,2], rot[0,0])
            euler[1] = -pi/2
            euler[2] = 0
    else:
        euler[0] = np.atan2(rot[0,2], rot[0,0])
        euler[1] = pi/2
        euler[2] = 0
    
    return euler.T

def Rot2Quat(Rot):
    # Calculate the trace of the matrix
    trace = np.trace(Rot)
    
    # Initialize quaternion array
    q = np.zeros(4)
    
    if trace > 0:
        s = 2.0 * np.sqrt(trace + 1.0)
        q[0] = 0.25 * s
        q[1] = (Rot[2, 1] - Rot[1, 2]) / s
        q[2] = (Rot[0, 2] - Rot[2, 0]) / s
        q[3] = (Rot[1, 0] - Rot[0, 1]) / s
    elif (Rot[0, 0] > Rot[1, 1]) and (Rot[0, 0] > Rot[2, 2]):
        s = 2.0 * np.sqrt(1.0 + Rot[0, 0] - Rot[1, 1] - Rot[2, 2])
        q[0] = (Rot[2, 1] - Rot[1, 2]) / s
        q[1] = 0.25 * s
        q[2] = (Rot[0, 1] + Rot[1, 0]) / s
        q[3] = (Rot[0, 2] + Rot[2, 0]) / s
    elif Rot[1, 1] > Rot[2, 2]:
        s = 2.0 * np.sqrt(1.0 + Rot[1, 1] - Rot[0, 0] - Rot[2, 2])
        q[0] = (Rot[0, 2] - Rot[2, 0]) / s
        q[1] = (Rot[0, 1] + Rot[1, 0]) / s
        q[2] = 0.25 * s
        q[3] = (Rot[1, 2] + Rot[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + Rot[2, 2] - Rot[0, 0] - Rot[1, 1])
        q[0] = (Rot[1, 0] - Rot[0, 1]) / s
        q[1] = (Rot[0, 2] + Rot[2, 0]) / s
        q[2] = (Rot[1, 2] + Rot[2, 1]) / s
        q[3] = 0.25 * s

    return q

def Quat2Rot(quat):
    q_w, q_x, q_y, q_z = quat[0], quat[1], quat[2], quat[3]

    # 회전 행렬 계산
    R = np.array([
        [1 - 2*(q_y**2 + q_z**2), 2*(q_x*q_y - q_z*q_w), 2*(q_x*q_z + q_y*q_w)],
        [2*(q_x*q_y + q_z*q_w), 1 - 2*(q_x**2 + q_z**2), 2*(q_y*q_z - q_x*q_w)],
        [2*(q_x*q_z - q_y*q_w), 2*(q_y*q_z + q_x*q_w), 1 - 2*(q_x**2 + q_y**2)]
    ])

    return R

def conjugation(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])

def inverse_quat(q):
    q_norm = np.linalg.norm(q)
    q_inverse = conjugation(q) / q_norm ** 2
    return q_inverse

def mul_quat(q1,q2):
    w1, x1, y1, z1 = q1[0], q1[1], q1[2], q1[3]
    w2, x2, y2, z2 = q2[0], q2[1], q2[2], q2[3]

    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2

    mul = np.array([w, x, y, z])
    
    return mul

def Quat2Omega(quat, quatdot):
    trans = 2 * mul_quat(quatdot, inverse_quat(quat))
    Omega = np.array([trans[1], trans[2], trans[3]])
    return Omega

def SVD_DLS_inverse(J, lambda_factor=0.1):
    U, S, Vt = np.linalg.svd(J, full_matrices=False)
    
    S_damped = np.diag([s / (s**2 + lambda_factor**2) for s in S])
    
    J_damped_inverse = Vt.T @ S_damped @ U.T
    
    return J_damped_inverse

def DLS_inverse(J, init_lambda=0.01, threshold=0.01):
    cond_number = np.linalg.cond(J)
    
    if cond_number < threshold:
        lambda_factor = 0
    else:
        lambda_factor = init_lambda * (cond_number - threshold)

    J_inverse = np.linalg.inv(J.T @ J + lambda_factor**2 * np.eye(J.shape[1])) @ J.T
    
    return J_inverse