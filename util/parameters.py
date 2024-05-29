from scipy.spatial.transform import Rotation as Rot
from math import pi, atan2, sin, cos
import numpy as np

NUM_ROBOTS=4

init_X = [[0,6,0], [1,3,0], [0,0,0], [1,-3,0]]

DELTA_T = 0.1
# Q = np.diag(np.tile([.7, .05], NUM_ROBOTS)) # coefficient of variables
Q = np.diag(np.tile([0, 0], NUM_ROBOTS)) # coefficient of variables

# Q = np.diag([0, 0])
# Q_B = np.tile([0, 0], NUM_ROBOTS)
Q_B = np.tile([0.7, 0.3], NUM_ROBOTS)

R_1 = np.diag([0.05, 0.05, (pi/180)])
R_0 = np.diag([0.08, (2*pi/180)])
P_INIT = np.diag([0.05**2, 0.05**2, (pi/180)**2])

R_ALL_1 = np.diag(np.tile([0.05, 0.05, (pi/180)], NUM_ROBOTS))
R_all_0 = np.diag(np.tile([0.08, (2*pi/180)], NUM_ROBOTS))
P_ALL_INIT = np.diag(np.tile([0.05**2,0.05**2, (pi/180)**2],NUM_ROBOTS))

MAESUREMENT_RANGE_BOUND = 5 # [m]
observation_bearing_bound = pi # ±[Rad]

# Assume that the input follows the uniform distribution
# V_MAX = 0.2 # [m]
# V_MIN = 0 # [m]
# E_V = (V_MAX+V_MIN)/2

# OMEGA_MAX = pi/18 # [Rad]
# OMEGA_MIN = -pi/18 # [Rad]
# E_OMEGA = (OMEGA_MAX+OMEGA_MIN)/2

# # X~N(Y, (sigma*Y)^2)
# SIGMA_V2 = (V_MAX-V_MIN)**2/12 + Q[0, 0]**2 * ((V_MAX**2+V_MIN**2+V_MAX*V_MIN)/3) + \
#     2*Q[0, 0]*Q_B[0]*E_V + Q_B[0]**2
# SIGMA_OMEGA2 = (OMEGA_MAX - OMEGA_MIN)**2/12 + Q[1, 1]**2 * ((OMEGA_MAX**2+OMEGA_MIN**2+OMEGA_MAX*OMEGA_MIN)/3) + \
#     2*Q[1, 1]*Q_B[1]*E_OMEGA + Q_B[1]**2

# SIGMA0_V2 = SIGMA_V2 + (E_V)**2 # uniform distribution's variance and pick 0
# SIGMA0_OMEGA2 = SIGMA_OMEGA2 + (E_OMEGA)**2

E_V_ADD = np.zeros(NUM_ROBOTS)
E_OMEGA_ADD = np.zeros(NUM_ROBOTS)

# Assume that the input follows the normal distribution
V_MAX = 0.2 # [m]
V_MIN = 0 # [m]

OMEGA_MAX = 1 # pi/18 # [Rad]
OMEGA_MIN = -1 # -pi/18 # [Rad]

E_V = (V_MAX+V_MIN)/2 + E_V_ADD
E_OMEGA = (OMEGA_MAX+OMEGA_MIN)/2 + E_OMEGA_ADD

SIGMA_V_INPUT = (V_MAX-E_V)/3
SIGMA_OMEGA_INPUT = (OMEGA_MAX-E_OMEGA)/3

SIGMA_V2, SIGMA_OMEGA2 = np.zeros(NUM_ROBOTS), np.zeros(NUM_ROBOTS)
for r in range(NUM_ROBOTS):
    SIGMA_V2[r] = (SIGMA_V_INPUT[r])**2 + Q[2*r+0, 2*r+0]**2 * ((SIGMA_V_INPUT[r])**2 + E_V[r]**2) + \
        2*Q[2*r+0, 2*r+0]*Q_B[2*r+0]*E_V[r] + Q_B[2*r+0]**2
    SIGMA_OMEGA2[r] = (SIGMA_OMEGA_INPUT[r])**2 + Q[2*r+1, 2*r+1]**2 * ((SIGMA_OMEGA_INPUT[r])**2 + E_OMEGA[r]**2) + \
        2*Q[2*r+1, 2*r+1]*Q_B[2*r+1]*E_OMEGA[r] + Q_B[2*r+1]**2

# Try as UTIAS
# E_V = 0.114 + E_V_ADD
# E_OMEGA = 0 + E_OMEGA_ADD # 0

# SIGMA_V_INPUT = (V_MAX-E_V)/3
# SIGMA_OMEGA_INPUT = (OMEGA_MAX-E_OMEGA)/3

# SIGMA_V2, SIGMA_OMEGA2 = np.zeros(NUM_ROBOTS), np.zeros(NUM_ROBOTS)
# for r in range(NUM_ROBOTS):
#     SIGMA_V2[r] = 5.325e-3 + Q[2*r+0, 2*r+0]**2 * (E_V[r]**2 + 5.325e-3) + \
#         2*Q[2*r+0, 2*r+0]*Q_B[2*r+0]*E_V[r] + Q_B[2*r+0]**2
#     SIGMA_OMEGA2[r] = 0.544 + Q[1, 1]**2 * (E_OMEGA[r]**2 + 0.544) + \
#         2*Q[2*r+1, 2*r+1]*Q_B[2*r+1]*E_OMEGA[r] + Q_B[2*r+1]**2

SIGMA0_V2 = SIGMA_V2 + E_V**2 # uniform distribution's variance and pick 0
SIGMA0_OMEGA2 = SIGMA_OMEGA2 + E_OMEGA**2


R_ALL_1_NT = np.diag(np.tile([0.05, 0.05], NUM_ROBOTS)) # no relative angle


Rv = .6 # .1
MU_RANGE_BEARING = Rv * np.array([2.4, 2*pi/180])
# MU_POSE = Rv * np.array([0.2, 0.2, 4*pi/180])
MU_POSE = Rv * np.array([1.5, 1.5, 3*pi/180])
BIAS_SIGMA_RANGE_BEARING = np.array([0.08, 2*pi/180])
BIAS_SIGMA_POSE = np.array([0.05, 0.05, 1*pi/180])

meas_bia_prob = 0 # Ra
comm_fail_prob = 0.1

LANDMARK_POS = [] # [[11.5, 10.5, 0], [11.5, 4.5, 0], [11.5, -1.5, 0]]
LANDMARK_NUM = len(LANDMARK_POS)

R_ALL_0_LANDMARK = np.diag(np.tile([0.08, 2*(pi/180)], LANDMARK_NUM))
R_ALL_1_LANDMARK = np.diag(np.tile([0.05, 0.05, (pi/180)], LANDMARK_NUM))
R_ALL_1_NT_LANDMARK = np.diag(np.tile([0.05, 0.05], LANDMARK_NUM))


def rot_mat_2d(angle):
    '''
    return a matrix:
        [ cos(angle), sin(angle), 0]
        [-sin(angle), cos(angle), 0]
        [          0,          0, 1]
    '''
    return Rot.from_euler('z', angle).as_matrix().T

def normalize_angle(angle):
    # Inspired by codes from "masinjila_ruslan_2016_thesis"
    angle = angle % (2*pi)
    if angle > pi:
        angle = angle - 2*pi
    return angle

def measurement(X2,X1):
    
    '''
    Calculate observation variables

    Parameters
    ----------
    X2: 机器人2 被观测
    X1: 机器人1 观测

    Returns
    ----------
    _range, bearing, relative pose
    '''
    
    # R2 = rot_mat_2d(X2[2])
    # R2 = np.matrix([[cos(X2[2]), -sin(X2[2]), 0], [sin(X2[2]), cos(X2[2]), 0], [0,0,1]])
    # X1_ = np.dot(R2, X1-X2)

    R1 = rot_mat_2d(X1[2])
    # R1 = np.matrix([[cos(X1[2]), sin(X1[2]), 0], [-sin(X1[2]), cos(X1[2]), 0], [0,0,1]])
    X2_ = np.dot(R1, X2-X1)

    return np.linalg.norm(X1[0:2]-X2[0:2]), atan2(X2[1]-X1[1], X2[0]-X1[0]) - X1[2], X2_.tolist()

def is_pos_def(P):
    '''
    Judge whether the input matrix P is positive definite
    '''
    # print(np.linalg.eigvals(P))
    return np.all(np.linalg.eigvals(P) > 0) 

def plot_ellipse(ax, v, A, Color = 'k'):
    eigval, eigvec = np.linalg.eig(A)
    t=np.arange(0, 2*pi+0.1, 0.1)
    # if eigval[0] >= eigval[1]:
    #     bigind = 0
    #     smallind = 1
    # else:
    #     bigind = 1
    #     smallind = 0
    
    
    # a = (eigval[bigind]*5.991)**0.5
    # b = (eigval[smallind]*5.991)**0.5
    # x = [a*cos(it) for it in t]
    # y = [b*sin(it) for it in t]
    # angle = atan2(eigvec[1,bigind], eigvec[0,bigind])
    # fx = parameters.rot_mat_2d(angle)[0:2,0:2].T @ (np.array([x, y]))

    xy = np.array([[cos(it), sin(it)] for it in t]).T
    fx = eigvec[:2,:2] @ np.diag(np.sqrt(eigval[:2]*4.605)) @ xy
            
    px = np.array(fx[0, :] + v[0]).flatten()
    py = np.array(fx[1, :] + v[1]).flatten()
    
    ax.plot(px, py, Color)
    ax.plot(0, 0, '*', color=Color, markersize=6)