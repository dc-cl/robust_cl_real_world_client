#!/usr/bin/env python
import sys
sys.path.append('../')
# from pathlib import Path

import numpy as np
from math import pi
import time
import rospy
import socket
import threading
import pyrealsense2 as rs
import apriltag
import cv2
from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


# import matplotlib.pyplot as plt
# import scienceplots

import parameters as para

from algorithms.DR import Robot, Robot_true
from algorithms.BDA import Robot_BDA_EKF, Robot_BDA_EKF_KLD, \
                            Robot_BDA_IEKF, Robot_BDA_IEKF_KLD
from algorithms.DMV import Robot_DMV, Robot_PECMV
from algorithms.CU import Robot_BDA_EKF_CU, Robot_CI_CU, Robot_BDA_IEKF_CU
from algorithms.CCL import Robots_CCL
from algorithms.CI import Robot_CI
from algorithms.Split_CI import Robot_SCI
from algorithms.DCL_GS import Robot_GS_early_paper, Robot_GS_EKF, \
                                Robot_GS_EKF_FDE_NP_05, Robot_GS_EKF_FDE_Youden, \
                                Robot_GS_EKF_FDE_Chi_Youden, \
                                Robot_GS_IEKF, \
                                Robot_GS_IEKF_FDE_Youden, Robot_GS_IEKF_FDE_Chi_Youden, \
                                Robot_GS_LRHKF, Robot_GS_FDE_LRHKF, \
                                Robot_GS_CKF, Robot_GS_FDE_CKF, Robot_GS_CLRHKF, Robot_GS_FDE_CLRHKF, \
                                Robot_GS_LRH_GMKF, Robot_GS_FDE_LRH_GMKF, Robot_GS_CLRH_GMKF, Robot_GS_FDE_CLRH_GMKF\

# id
_id = 0

# total simulation time from parameters TODO
numbers = para.numbers
# time interval from parameters TODO
DELTA_T = para.DELTA_T

total_time = numbers * DELTA_T

SIGMA_V_INPUT, SIGMA_OMEGA_INPUT = para.SIGMA_V_INPUT, para.SIGMA_OMEGA_INPUT
sigma_v_input_, sigma_omega_input_ = SIGMA_V_INPUT[_id], SIGMA_OMEGA_INPUT[_id]
E_V, E_OMEGA = para.E_V, para.E_OMEGA
E_v_, E_omega_ = E_V[_id], E_OMEGA[_id]
NUM_ROBOTS = para.NUM_ROBOTS
types = para.types
init_X = para.init_X

alg_count = len(types)
if -1 in types: alg_count -= 1

s = None
pipe, CamIn, CDistortCoe = None, None, None

# Lock of 'v_all' and 'v_count
v_all_lock = threading.lock()
# int, index about where velocity updates
v_count = -1
# array, list all the velocities
v_all = np.zeros((numbers, 2))

# Lock of 'state_alg', 'cov_alg' and *_count
state_lock = threading.lock()
# dict, list all the states updated by algorithms
state_alg = {}
cov_alg = {}
# motion_count, meas_count, comm_count = 0, 0, 0
state_count = [0, 0, -1]
# int: After comm: index before 'back_need' should reupdate
back_need = -1

state_cond = threading.Condition(lock=state_lock)


# class of algorithms
algs_motion, algs_meas, algs_comm = {}, {}, {}

# broadcast the communication history and the init information
str_broad = '/broadcast_comm_his_GS'
# A lock about 'broadcast_comm_his_GS'
str_broad_lock = '/broadcast_lock'

# Float: timestamp of start
start_time = None

rospy.init_node('client'+str(_id), anonymous=False) # 

def init():
    '''
    Generates the results for the specified MRCLAM dataset and algorithm.

    Parameters
    ----------
    init_X: list
            All robots' initial states
    types: int
            20 Resilient-no theta
            24 Resilient-with theta and Chi2 detection with Youden index
    falg: int
        which measuement model
    
    
    '''
    ###### Software init Start######
    global start_time, state_alg, cov_alg, algs_motion, alg_meas, algs_comm

    meas_bia_prob, comm_fail_prob = para.meas_bia_prob, para.comm_fail_prob
    if not meas_bias_prob is None:
        meas_bia_prob = meas_bias_prob

    np.random.seed(_id)
    print(f"meas_bia_prob = {meas_bia_prob}, comm_fail_prob = {comm_fail_prob}")

    
    LANDMARK_NUM = para.LANDMARK_NUM
    LANDMARK_POS = para.LANDMARK_POS

    n2 = len(types) # the number of algorithms

    
    # Initialize
    if n2: 
        for type in types:
            if type == 20:
                algs_motion[20] = Robot_GS_early_paper(initial_s=init_X, _id=_id, NUM_ROBOTS=NUM_ROBOTS, flag=flag, LANDMARK_POS=LANDMARK_POS)
                algs_meas[20] = Robot_GS_early_paper(initial_s=init_X, _id=_id, NUM_ROBOTS=NUM_ROBOTS, flag=flag, LANDMARK_POS=LANDMARK_POS)
                algs_comm[20] = Robot_GS_early_paper(initial_s=init_X, _id=_id, NUM_ROBOTS=NUM_ROBOTS, flag=flag, LANDMARK_POS=LANDMARK_POS)
            elif type == 28:    
                algs_motion[28] = Robot_GS_LRHKF(initial_s=init_X, _id=_id, NUM_ROBOTS=NUM_ROBOTS, flag=flag, LANDMARK_POS=LANDMARK_POS)
                algs_meas[28] = Robot_GS_LRHKF(initial_s=init_X, _id=_id, NUM_ROBOTS=NUM_ROBOTS, flag=flag, LANDMARK_POS=LANDMARK_POS)
                algs_comm[28] = Robot_GS_LRHKF(initial_s=init_X, _id=_id, NUM_ROBOTS=NUM_ROBOTS, flag=flag, LANDMARK_POS=LANDMARK_POS)
            elif type == -1:
                algs_motion[-1] = Robot(X=init_X[_id], _id=_id, NUM_ROBOTS=NUM_ROBOTS)
            
            if type >= 20:
                state_alg[type] = np.zeros((numbers, 3*NUM_ROBOTS))
                cov_alg[type] = np.zeros((numbers, 3*NUM_ROBOTS, 3*NUM_ROBOTS))
                cov_alg[type][0] = algs_motion[type].P_GS.copy()
                state_alg[type][0,:3*NUM_ROBOTS] = np.array(init_X)
            else:
                state_alg[type] = np.zeros((numbers, 3))
                cov_alg[type] = np.zeros((numbers, 3, 3))
                cov_alg[type][0] = algs_motion[type].P.copy()
                state_alg[type][0,:3] = np.array(init_X[_id])
    
    ######## hardware init starts ##########
    # 1. RoboMaster
    global s
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect(("192.168.42.2", int(40923)))
    msg_robo = "command;"
    s.send(msg_robo.encode('utf-8'))
    buf_robo = s.recv(1024)

    # 2. D435i
    global pipe, CamIn, CDistortCoe
    Wid = 1920
    Hei = 1080
    pipe = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, Wid, Hei, rs.format.bgr8, 30)
    try:
        cfg = pipe.start(config)
    except Exception:
        str_warn = "blaster fire;"
        s.send(str_warn.encode('utf-8'))
        buf_robo = s.recv(1024)
        time.sleep(2)
    
    CProfile = cfg.get_stream(rs.stream.color)
    CIntrin = CProfile.as_video_stream_profile().get_intrinsics()
    CamIn = np.array([[CIntrin.fx, 0.0, CIntrin.ppx], [0.0, CIntrin.fy, CIntrin.ppy], [0.0, 0.0, 1.0]])
    CDistortCoe = np.array(CIntrin.coeffs)

    for _ in range(40):
        pipe.wait_for_frames()

    ######## Already initialize! ########
    # Register
    
    
    # start time
    str_start_time = '/start_time'
    # Bool: if 'broadcast_comm_his_GS' is created by this client, then True
    Create_broad = False
    # Bool: if 'start_time' is created by this client, then True
    Create_start_time = False

    if not rospy.has_param(str_broad):
        # 'broadcast_comm_his_GS' not be established in the Parameter Server
        broadcast_comm_his_GS = [0 for r2 in range(NUM_ROBOTS*NUM_ROBOTS)]
        broadcast_comm_his_GS[(NUM_ROBOTS + 1)*_id] = 1
        rospy.set_param(str_broad, broadcast_comm_his_GS)
        Create_broad = True

    if not Create_broad:
        # 'broadcast_comm_his_GS' has been established, need to update
        while rospy.has_param(str_broad_lock):
            rospy.sleep(0.1)
        else: rospy.set_param(str_broad_lock, True)
        broadcast_comm_his_GS = rospy.get_param(str_broad)
        broadcast_comm_his_GS[(NUM_ROBOTS + 1)*_id] = 1
        
        for r in range(NUM_ROBOTS):
            if broadcast_comm_his_GS[(NUM_ROBOTS + 1)*r] <= 0: break
        else:
            # all robots have initialized,
            start_time = time.time() + 5
            rospy.set_param(str_start_time, start_time)
            Create_start_time = True

        rospy.set_param(str_broad, broadcast_comm_his_GS)
        rospy.delete_param(str_broad_lock)
    
    if not Create_start_time:
        while rospy.has_param(str_start_time):
            # Not all robots have initialized
            rospy.sleep(0.1)
        
        start_time = rospy.get_param(str_start_time)

# ---------------------------------
    
def motion():
    global v_all, v_count, next_motion_time

    delay = 0
    while start_time is None:
        rospy.sleep(0.1)

    next_motion_time = start_time
    final_time = start_time + total_time
    velocity = [np.random.randn()*sigma_v_input_ + E_v_, np.random.randn()*sigma_omega_input_ + E_omega_]
    
    while next_motion_time < final_time:
        if time.time() + delay >= next_motion_time:
            msg_robo = f"chassis speed x {velocity[0]} y 0 z {-velocity[1]}"
            s.send(msg_robo.encode('utf-8'))
            buf_robo = s.recv(1024)

            with v_all_lock:
                v_count += 1
                v_all[v_count] = velocity
                

            velocity = [np.random.randn()*sigma_v_input_ + E_v_, np.random.randn()*sigma_omega_input_ + E_omega_]
            next_motion_time += DELTA_T

# -------------------

# TODO 当观测,通讯完成后,可能有些滞后,需要重新update(666)
def time_propagation():
    global state_alg, cov_alg, state_count, algs_motion
    while not rospy.is_shutdown():
        v_update = None
        with state_lock:
            v_count_local = state_count[0]
        with v_all_lock:
            if v_count > v_count_local:
                v_update = v_all[v_count_local].copy()

        if v_update is not None:       
            with state_lock:
                for type in types:
                    if state_count[1] >= v_count_local:
                        # time_propagation, the belief after measurement update can be obtained
                        # index of belief in algs_motion equals to v_count_local
                        if type >= 20:
                            algs_motion[type].X_GS[:,0] = state_alg[type][v_count_local].copy()
                            algs_motion[type].P_GS = cov_alg[type][v_count_local].copy()
                        # TODO ()other type of alg
                    
                    algs_motion[type].motion(v = v_all[v_count_local, 0], omega = v_all[v_count_local, 1])
                    
                    # Why v_count_local + 1? because this is prediction
                    if type >= 20:
                        state_alg[type][v_count_local + 1] = algs_motion[type].X_GS.copy()
                        cov_alg[type][v_count_local + 1] = algs_motion[type].P_GS.copy()
                    else:
                        state_alg[type][v_count_local + 1] = algs_motion[type].X.copy()
                        cov_alg[type][v_count_local + 1] = algs_motion[type].P.copy()
                
                state_count[0] += 1
            
            # Notify that the motion model has been updated
            with state_cond:
                state_cond.notify_all()
                        
# -----------------

cam2robo_rot = np.array([[1,0,0],[0,0,1],[0,-1,0]],dtype="double")
cam2robo_tra = np.array([[-154.5],[156.18],[87.5]])

# PnP algorithm
def PnPProcess(Points3D, Points0, CamIn, CDistortCoe):
    retval, RVec, TVec = cv2.solvePnP(Points3D, Points0, CamIn, CDistortCoe,
                                      flags=cv2.SOLVEPNP_ITERATIVE)  # useExtrinsicGuess = False,cv2.SOLVEPNP_EPNP
    RotWCam, Jacobian = cv2.Rodrigues(RVec)
    RotWCam = cam2robo_rot @ RotWCam
    TVec = cam2robo_rot @ TVec + cam2robo_tra # +bias from camera

    q0 = np.sqrt(np.trace(RotWCam)+1)/2.0
    q3 = (RotWCam[1,0]-RotWCam[0,1])/4/q0
    # RotCamW = RotWCam.T
    # CamW = np.dot(-RotCamW, TVec) # Another relative pose
    
    #return (str(CamW[0, 0]).ljust(20), str(CamW[1, 0]).ljust(20), str(CamW[2, 0]).ljust(20),
    #        str(RVec[0, 0]).ljust(20), str(RVec[1, 0]).ljust(20), str(RVec[2, 0]).ljust(20))
    # return [(CamW[0]**2 + CamW[1]**2)**(1/2), m.atan2(CamW[1], CamW[0]), np.linalg.norm(RVec)]
    # print(f"type0={type(CamW[1,0])}, type1={type(np.linalg.norm(RVec))}")

    # return [CamW[1, 0], CamW[0, 0], np.linalg.norm(RVec)]
    #print(TVec)
    return [TVec[1, 0], -TVec[0, 0], 2*m.asin(q3)] # project to z-axis

# Lock of 'mea_all' and 'mea_count'
mea_lock = threading.lock()
mea_rela_all = np.zeros((numbers, NUM_ROBOTS, 3))
mea_count = 0

robot_labels = para.Points3DAll.copy()
Options = apriltag.DetectorOptions(quad_decimate=2.0)
detections = apriltag.Detector(options=Options)

def Measurement():
    global mea_all, mea_count
    next_motion_time = start_time + DELTA_T
    while not rospy.is_shutdown():
        frame = pipe.wait_for_frames()
        measure_time = time.time()
        if measure_time >= next_motion_time:
            next_motion_time += DELTA_T
        else: continue
        # XXX 图像的获取与处理可以分成2个线程?
        Color = frame.get_color_frame()
        Color_np = np.asanyarray(Color.get_data())
        Gray_np = cv2.cvtColor(Color_np, cv2.COLOR_BGR2GRAY)
        tags = detections.detect(Gray_np)
        pnp_info = {}
        pnp_count = np.zeros(NUM_ROBOTS)
        see_landmark = False
        master_numb = -1
        if len(tags):
            #count = 0
            for tag in tags:
                ID = tag.tag_id
                #count = count + 1
                #print(ID)
                if ID >= 100:
                    # TODO ()process of landmarks
                    Point2DNp = np.array(tag.corners)
                    Point3D = robot_labels[8]
                    temp = PnPProcess(Point3D, Point2DNp, CamIn, CDistortCoe)
                    see_landmark = True
                elif ID <= MaxIndex:
                    master_numb = ID//8
                    Point2DNp = np.array(tag.corners)  ###########
                    # print(Point2DNp)Point3D = robot_labels[int(ID), 0:4, 0:3]
                    Point3D = robot_labels[int(ID%8)]

                    temp = PnPProcess(Point3D, Point2DNp, CamIn, CDistortCoe)
                    # n_temp = [(temp[0] ** 2 + temp[1] ** 2) ** (1 / 2), m.atan2(temp[1], temp[0]), temp[2]]
                    # n_temp = [(CamW[0] ** 2 + CamW[1] ** 2) ** (1 / 2), m.atan2(CamW[1], CamW[0]), np.linalg.norm(RVec)]
                if master_numb in pnp_info.keys():
                    pnp_info[master_numb] = pnp_info[master_numb] + np.array(temp)
                    pnp_count[master_numb] = pnp_count[master_numb] + 1
                else:
                    pnp_info[master_numb] = np.array(temp)
                    pnp_count[master_numb] = 1
            # pnp_result = pnp_result / count
            for id in pnp_info.keys():
                print(f"With id={id} {pnp_count[id]} detections")
                pnp_info[id] = pnp_info[id] / pnp_count[id]
                pnp_info[id][0:2] = pnp_info[id][0:2] / 1000 # mm -> m

            with mea_lock:
                mea_count += 1
                for id in pnp_info.keys():
                    mea_rela_all[mea_count, id, :] = pnp_info[id].copy()

# --------------------

def Mea_update():
    global state_count, state_alg, cov_alg, algs_meas
    Need_Update = False
    while not rospy.is_shutdown():
        # while True:
        #     with state_cond:
        #         if state_count[1] < state_count[0]:
        #             # motion has updated to the current, next the meas update
        #             # Comm update just now
        #             mea_count_local = state_count[1]
        #             break
        #         state_cond.wait()

        with state_cond:
            if state_count[1] >= state_count[0]:
                # motion has updated to the current, next the meas update
                # Comm update just now
                state_cond.wait()
            else: mea_count_local = state_count[1]

        # if Need_Update:
        #     with state_lock:
        #         state_count[1] += 1
        # else:

        with mea_lock:
            if mea_count > mea_count_local:
                Need_Update = True
                # mea_count_local += 1
                # mea_count_max_local = max(mea_count_max_local, mea_count_local)
                for type in types:
                    if type == -1: continue
                    for r in range(NUM_ROBOTS):
                        if not np.all(mea_rela_all[mea_count_local, r,:]) == 0:
                            algs_meas[type].measuring[r] = True
                            if type == 20:
                                algs_meas[type].Z[2*r:2*r+2] = mea_rela_all[mea_count_local, r, :2].copy()
                            elif type > 20:
                                algs_meas[type].Z[3*r:3*r+3] = mea_rela_all[mea_count_local, r, :].copy()
                            # TODO ()other type of algorithms
    
        if Need_Update:
            # Appears when measure faster than motion
            # while True:
            #     with state_cond:
            #         if mea_count_local <= state_count[0]:
            #             break
            #         state_cond.wait()
            with state_cond:
                if mea_count_local > state_count[0]:
                    state_cond.wait()

            with state_lock:
                for type in types:
                    if type >=20:
                        algs_motion[type].X_GS[:,0] = state_alg[type][mea_count_local].copy()
                        algs_motion[type].P_GS = cov_alg[type][mea_count_local].copy()
                    # TODO ()other type of algorithms
            
            for type in types:
                algs_meas[type].rela_meas_correct()
                algs_meas[type].reset_rela()
            
            with state_lock:
                state_count[1] += 1
                for type in types:
                    if type >= 20:
                        state_alg[type][state_count[1]] = algs_motion[type].X_GS[3*+id:3*_id+3,:].copy()
                    # TODO ()other type of algorithms
            
            Need_Update = False

# ------------

comm_lock = threading.Lock()
state_comm, cov_comm = None, None
comm_count = 1

COMM_RATE = para.COMM_RATE

def Comm_send():
    global state_comm, cov_comm, comm_count, back_need
    
    comm_times = 1
    comm_interval = DELTA_T * COMM_RATE
    comm_complete = False
    
    pub = rospy.Publisher('comm', Float64MultiArray, queue_size=10)
    while not rospy.is_shutdown():
        curr_comm_times = time.time() - start_time
        if not comm_complete and \
            (curr_comm_times > comm_interval * comm_times and curr_comm_times < comm_interval * (comm_times + 1)) \
            and curr_comm_times < numbers:
            
            while rospy.has_param(str_broad_lock):
                rospy.sleep(1e-3)
            else: rospy.set_param(str_broad_lock, True)
            broadcast_comm_his_GS = rospy.get_param(str_broad)
            if state_comm is None:
                broadcast_comm_his_GS[(NUM_ROBOTS+1)*_id] = comm_times
                for r in range(NUM_ROBOTS):
                    if broadcast_comm_his_GS[NUM_ROBOTS*_id + r] > numbers:
                        broadcast_comm_his_GS[NUM_ROBOTS*_id + r] -= numbers
                rospy.get_param(str_broad, broadcast_comm_his_GS)
            rospy.delete_param(str_broad_lock)
            
            if state_comm is None:
                # 1st: Obtain the belief for comm
                with state_cond:
                    # type[0]
                    if np.all(state_alg[types[0]][comm_times*COMM_RATE-1, :]) == 0:
                        state_cond.wait()
                    else:
                        state_comm, cov_comm = {}, {}
                        for type in types:
                            state_comm[type] = state_alg[type][comm_times*COMM_RATE-1].copy()
                            cov_comm[type] = cov_alg[type][comm_times*COMM_RATE-1].copy()
            
            # index = 0: id receiving; 1: id; 2: comm_times; 3: type of algorithms; last: sending state+covariance
            message = Float64MultiArray()
            message.data = [_id for ind in range(alg_count*NUM_ROBOTS*(NUM_ROBOTS+1)+3)]
            
            
            comm_1st_order = sorted(np.arange(NUM_ROBOTS), key = lambda i: broadcast_comm_his_GS[NUM_ROBOTS*_id + i])
            
            if broadcast_comm_his_GS[NUM_ROBOTS*_id + comm_1st_order[0]] == comm_times:
                comm_complete = True
            else:
                message.data[0] = comm_1st_order[0]
                # message[1] = _id
                message.data[2] = comm_times
                # Each algorithm needs NUM_ROBOTS*(NUM_ROBOTS+1)
                num = 0
                for type in types:
                    if type == -1: continue
                    message.data[3 + num*NUM_ROBOTS*(NUM_ROBOTS+1):3+NUM_ROBOTS + num*NUM_ROBOTS*(NUM_ROBOTS+1)] = state_comm[type].copy()
                    message.data[3+NUM_ROBOTS + num*NUM_ROBOTS*(NUM_ROBOTS+1):3 + (num+1)*NUM_ROBOTS*(NUM_ROBOTS+1)] = cov_comm[type].reshape(NUM_ROBOTS*NUM_ROBOTS,).copy()
                    num += 1
                pub.publish(message)
        else:
            with comm_lock:
                with state_lock:
                    state_alg[type][comm_times*COMM_RATE-1] = state_comm[type].copy()
                    cov_alg[type][comm_times*COMM_RATE-1] = cov_comm[type].copy()
                    state_count[0] = comm_times*COMM_RATE-1
                    state_count[1] = comm_times*COMM_RATE-1
                    back_need = comm_times*COMM_RATE-1
                
                state_comm, cov_comm = None, None
                comm_count += 1
            comm_times += 1
            comm_complete = False

# queue_recv = []
def Comm_recv_callback(recv):
    if int(recv.data[0]) != _id: return
    
    succ_recv = False
    state_recv, cov_recv = {}, {}
    with comm_lock:
        if int(recv.data[2]) == comm_count:
            succ_recv = True
            for type in types:
                if type == -1: continue
                if type >= 20: 
                    algs_comm[type].X_GS = state_comm[type].copy()
                    algs_comm[type].P_GS = cov_comm[type].copy()
                # TODO()other type of algorithms
    
    if succ_recv:
        succ_recv == False
        
        num = 0
        for type in types:
            if type == -1: continue
            state_recv[type] = recv.data[3 + num*NUM_ROBOTS*(NUM_ROBOTS+1):3+NUM_ROBOTS + num*NUM_ROBOTS*(NUM_ROBOTS+1)].copy()
            cov_recv[type] = np.array(recv.data[3+NUM_ROBOTS + num*NUM_ROBOTS*(NUM_ROBOTS+1):3 + (num+1)*NUM_ROBOTS*(NUM_ROBOTS+1)]).reshape(NUM_ROBOTS, NUM_ROBOTS).copy()
            num += 1
            can = algs_comm[type].communicate1_CI(state_recv[type], cov_recv[type], recv.data[1])
            
            while rospy.has_param(str_broad_lock):
                rospy.sleep(1e-3)
            else: rospy.set_param(str_broad_lock, True)
            broadcast_comm_his_GS = rospy.get_param(str_broad)
            
            if can:
                broadcast_comm_his_GS[NUM_ROBOTS*recv.data[1] + _id] = recv.data[2]
            else:
                broadcast_comm_his_GS[NUM_ROBOTS*recv.data[1] + _id] += numbers
            rospy.delete_param(str_broad_lock)

        with comm_lock:
            for type in types:
                state_comm[type] = algs_comm[type].X_GS.copy()
                cov_comm[type]=  algs_comm[type].P_GS.copy()




def Comm_recv():
    rospy.Subscriber('comm', Float64MultiArray, Comm_recv_callback)
    rospy.spin()


color_r = [1.0, ]
color_g = [0.0, ]
color_b = [0.0, ]

count = 0
markers = {}
for typ in types:
    markers[typ] = Marker()
    # marker.header.frame_id = "base_link"  # Set the frame ID
    # marker.header.stamp = rospy.Time.now()
    markers[typ].type = Marker.POINTS
    markers[typ].action = Marker.ADD
    markers[typ].pose.orientation.w = 1.0
    markers[typ].scale.x = 0.1  # Point size
    markers[typ].scale.y = 0.1
    markers[typ].color.r = 1.0  # Red color
    markers[typ].color.a = 1.0  # Full opacity

    count += 1

def Send2rviz():
    marker_publisher = {}
    for typ in types:
        marker_publisher[typ] = rospy.Publisher(str(_id) + 'tra' + str(type), Marker, queue_size=10)
    # time, time_max, mea, mea_max
    state_count_local = [-1, 0]
    while not rospy.is_shutdown():
        with state_lock:
            if back_need < state_count_local[1]: state_count_local[1] = back_need
            if state_count[0] > state_count_local[0]:
                while state_count_local[0] < state_count[0]:
                    state_count_local[0] += 1
                    for typ in types:
                        point = Point()
                        if typ >= 20:
                            point.x = state_alg[typ][state_count_local[0], 3*_id+0]
                            point.y = state_alg[typ][state_count_local[0], 3*_id+1]
                            point.z = state_alg[typ][state_count_local[0], 3*_id+2]
                        else:
                            point.x = state_alg[typ][state_count_local[0], 0]
                            point.y = state_alg[typ][state_count_local[0], 1]
                            point.z = state_alg[typ][state_count_local[0], 2]

                        markers[typ].points.append(point)
                        markers[typ].header.frame_id = state_count_local[0]
                        markers[typ].header.stamp = start_time + DELTA_T*state_count_local[0]
            
            elif state_count[1] > state_count_local[1]:
                while state_count_local[1] < state_count[1]:
                    state_count_local[1] += 1
                    for typ in types:
                        point = Point()
                        if typ >= 20:
                            point.x = state_alg[typ][state_count_local[0], 3*_id+0]
                            point.y = state_alg[typ][state_count_local[0], 3*_id+1]
                            point.z = state_alg[typ][state_count_local[0], 3*_id+2]
                        else:
                            point.x = state_alg[typ][state_count_local[0], 0]
                            point.y = state_alg[typ][state_count_local[0], 1]
                            point.z = state_alg[typ][state_count_local[0], 2]
                        
                        markers[typ].points[state_count_local[1]] = point.copy()
        
        for typ in types:
            marker_publisher[typ].publish(markers[typ])




# ------------------
    
if __name__ == '__main__':
    
    init(type = [20, 28], flag = -1)

    thread_motion = threading.Thread(target=motion)
    thread_motion.start()

    thread_meas = threading.Thread(target=Measurement)
    thread_meas.start()
    
    thread_propagation = threading.Thread(target=time_propagation)
    thread_propagation.start()

    thread_meas_update = threading.Thread(target=Mea_update)
    thread_meas_update.start()

    thread_comm_send = threading.Thread(target=Comm_send)
    thread_comm_send.start()

    thread_comm_recv = threading.Thread(target=Comm_recv)
    thread_comm_recv.start()

    while(rospy.is_shutdown()):
        # TODO thread.join
        thread_motion.join()
        thread_meas.join()
        thread_propagation.join()
        thread_meas_update.join()
        thread_comm_send.join()
        thread_comm_recv.join()