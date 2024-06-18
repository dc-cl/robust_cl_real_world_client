from scipy.spatial.transform import Rotation as Rot
from math import pi, atan2, sin, cos
import numpy as np

numbers = 10
DELTA_T = 0.1
NUM_ROBOTS = 1
# init_X = [[0, 6, 0], [0, 0, 0]]
# init_v = [[.1, .1], [.2, .2]]
init_X = [[0, 6, 0]]
init_v = [[.1, .1]]

# types为一个数组，不同数字表示不同的算法类型
types = [28]
'''
    flag: int, observation model
        1 range-bearing and transform to relative position
        0 range-bearing
        -1 relative pose
    types: int
        -1 Dead Reckon(DR)
        0 Decentralized EKF(BDA)
        2 Decentralized EKF with CU(DCL-CU)
        6 Decentralized EKF(DMV)        
        12 CI-CU
        28 multi-centralized + M-estimation
'''

Q = np.diag(np.tile([.7, .3], NUM_ROBOTS))  # 式（4）sigma_v和sigma_w
Q_B = np.zeros(3* NUM_ROBOTS)   # 用于调节噪声
R_1 = np.array([0.05, 0.05, (pi / 180)])
R_ALL = np.diag(np.tile([0.05, 0.05, (pi / 180)], NUM_ROBOTS))  # 量测模型噪声的协方差
P_INIT = np.diag(np.tile([0.05 ** 2, 0.05 ** 2, (pi / 180) ** 2], NUM_ROBOTS)) # 初始化状态协方差
R_0 = np.diag([0.08])
# 量测更新的范围要求
MAESUREMENT_RANGE_BOUND = 5  # [m]
MAESUREMENT_BEARING_BOUND = pi  # ±[Rad]

# 根据效果考虑做的补偿
E_V_ADD = np.zeros(NUM_ROBOTS)
E_OMEGA_ADD = np.zeros(NUM_ROBOTS)

# Assume that the input follows the normal distribution
V_MAX = 0.2  # [m]
V_MIN = 0  # [m]
OMEGA_MAX = 1  # pi/18 # [Rad]
OMEGA_MIN = -1  # -pi/18 # [Rad]

# 式（32）（33）
E_V = (V_MAX + V_MIN) / 2 + E_V_ADD
E_OMEGA = (OMEGA_MAX + OMEGA_MIN) / 2 + E_OMEGA_ADD
SIGMA_V_INPUT = (V_MAX - E_V) / 3
SIGMA_OMEGA_INPUT = (OMEGA_MAX - E_OMEGA) / 3
SIGMA_V2, SIGMA_OMEGA2 = np.zeros(NUM_ROBOTS), np.zeros(NUM_ROBOTS)

Rv = .6
Bias = Rv * np.array([1.5, 1.5, 3 * pi / 180])  # 第二个乘数：量测模型的噪声均值，本来这个噪声是符合零均值正态分布的，但是因为偏置，导致其零均值变化为1.5，1.5...
BIAS_SIGMA_POSE = np.array([0.05, 0.05, 1 * pi / 180])  # 量测模型初始噪声，符合零均值分布，即sigma_zij,t

meas_bia_prob = 0.1
comm_fail_prob = 0.5
COMM_RATE = 10


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
    # 该函数用于确保角度在-pi到pi之间
    angle = angle % (2 * pi)
    if angle > pi:
        angle = angle - 2 * pi
    return angle


def measurement(X2, X1):
    '''
    Calculate observation variables
    输真实位置得到的量测，不含噪声，仿真用
    Parameters
    ----------
    X2: 机器人2 被观测
    X1: 机器人1 观测

    Returns
    ----------
    _range, bearing, relative pose
    '''

    R1 = rot_mat_2d(X1[2])
    X2_ = np.dot(R1, X2 - X1)  # 式（8）

    # 计算了两个点之间的距离、方向角度差，并将 X2_ 数组转换为列表。
    return np.linalg.norm(X1[0:2] - X2[0:2]), atan2(X2[1] - X1[1], X2[0] - X1[0]) - X1[2], X2_.tolist()


def is_pos_def(P):
    '''
    Judge whether the input matrix P is positive definite
    '''
    return np.all(np.linalg.eigvals(P) > 0)
