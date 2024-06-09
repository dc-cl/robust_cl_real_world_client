#!/usr/bin/env python3
import sys
sys.path.append('../')
from pathlib import Path

import numpy as np
from math import pi
import time
import rospy
import threading
from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import parameters as para
import matplotlib.pyplot as plt
import scienceplots
from algorithms.DCL_GS import Robot_GS_early_paper, Robot_GS_LRHKF
from nlink_parser.msg import LinktrackNodeframe2


_id = 0                         # id
NUM_ROBOTS = para.NUM_ROBOTS    # robo个数
numbers = para.numbers          # robo运动次数
DELTA_T = para.DELTA_T          # 每次运动时间
total_time = numbers * DELTA_T  # 总共运行时间 包含多次运动
types = para.types              # 算法种类
init_X = para.init_X            # robo初始位置
init_v = para.init_v            # robo初始理论输入速度

# 式（32）（33）
SIGMA_V_INPUT, SIGMA_OMEGA_INPUT = para.SIGMA_V_INPUT, para.SIGMA_OMEGA_INPUT
E_V, E_OMEGA = para.E_V, para.E_OMEGA

sigma_v_input_, sigma_omega_input_ = SIGMA_V_INPUT, SIGMA_OMEGA_INPUT
E_v_, E_omega_ = E_V, E_OMEGA

alg_count = len(types)
# -1表示包含DR
if -1 in types:
    alg_count -= 1

# Lock of 'v_all' and 'v_count;Lock of 'state_alg', 'cov_alg' and *_count
v_all_lock = threading.lock()
state_lock = threading.lock()

# int, index about where velocity updates
v_count = -1

measure_count = 0               # 跟踪测量次数
v_all = np.zeros((numbers, 2))  # 存储每个robo的速度数据

# dict, list all the states updated by algorithms
state_alg = {}
cov_alg = {}

# motion_count, meas_count, comm_count = 0, 0, 0,用于跟踪 motion、meas 和 comm 三种操作的进度
state_count = [0, 0, -1]

# int: After comm: index before 'back_need' should reupdate,用于标记在通信后需要重新更新的状态索引
back_need = -1

# 这意味着在使用条件变量时，必须先获得 state_lock 的锁，然后才能操作条件变量。
state_cond = threading.Condition(lock=state_lock)


# class of algorithms
algs_motion, algs_meas, algs_comm = {}, {}, {}

# broadcast the communication history and the init information,定义一个字符串 str_broad,用于存储广播历史的 ROS 参数名
str_broad = '/broadcast_comm_his_GS'
# A lock about 'broadcast_comm_his_GS',定义一个字符串 str_broad_lock,用于存储广播历史锁的 ROS 参数名
str_broad_lock = '/broadcast_lock'

# 初始化 start_time 为 None,用于存储程序启动时间
start_time = None
rospy.init_node('client'+str(_id), anonymous=False)


def init():
    '''
    Generates the results for the specified MRCLAM dataset and algorithm.

    Parameters
    ----------
    init_X: list
            All robots' initial states
    types: int
            20 Resilient-no theta NT
            24 Resilient-with theta and Chi2 detection with Youden index
            -1 DR
            0 Decentralized EKF(BDA)
            2 Decentralized EKF with CU(DCL-CU)
            6 Decentralized EKF(DMV)
            12 CI-CU
            28 multi-centralized + M-estimation
    falg: int
        which measuement model
    '''

    ###### Software init Start######
    global start_time, state_alg, cov_alg, algs_motion, alg_meas, algs_comm

    meas_bia_prob, comm_fail_prob = para.meas_bia_prob, para.comm_fail_prob
    print(f"meas_bia_prob = {meas_bia_prob}, comm_fail_prob = {comm_fail_prob}")

    n2 = len(types)  # the number of algorithms
    # Initialize
    if n2: 
        for type in types:
            if type == 20:
                algs_motion[20] = Robot_GS_early_paper(initial_s=init_X, _id=_id, NUM_ROBOTS=NUM_ROBOTS)
                algs_meas[20] = Robot_GS_early_paper(initial_s=init_X, _id=_id, NUM_ROBOTS=NUM_ROBOTS)
                algs_comm[20] = Robot_GS_early_paper(initial_s=init_X, _id=_id, NUM_ROBOTS=NUM_ROBOTS)
            elif type == 28:    
                algs_motion[28] = Robot_GS_LRHKF(initial_s=init_X, _id=_id, NUM_ROBOTS=NUM_ROBOTS)
                algs_meas[28] = Robot_GS_LRHKF(initial_s=init_X, _id=_id, NUM_ROBOTS=NUM_ROBOTS)
                algs_comm[28] = Robot_GS_LRHKF(initial_s=init_X, _id=_id, NUM_ROBOTS=NUM_ROBOTS)
            elif type == -1:
                algs_motion[-1] = Robot(X=init_X[_id], _id=_id, NUM_ROBOTS=NUM_ROBOTS)
            # 初始化状态和协方差矩阵
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
    # 1. 初始化，接受三个话题的数据
    class TopicSubscriber:
        def __init__(self,topic_name):
            # 初始化 ROS 节点
            rospy.init_node('topic_subscriber', anonymous=True)
            # 订阅话题
            self.sub = rospy.Subscriber(topic_name, LinktrackNodeframe2, self.callback)
            # 存储数据
            self.data_list = []
        def callback(self, msg_data,msg_nodes):
            self.id = msg_data.id
            self.data_list.append(msg_nodes)
        def run(self):
            rate = rospy.Rate(10)  # 10 Hz
            while not rospy.is_shutdown():
                rate.sleep()
    data = []
    a = TopicSubscriber('LinktrackNodeframe2_0')
    b = TopicSubscriber('LinktrackNodeframe2_1')
    c = TopicSubscriber('LinktrackNodeframe2_2')
    a.run()
    b.run()
    c.run()
    for subscriber in (a,b,c):
        if subscriber.id == 1:
            data.append(subscriber)


    ######## Already initialize! ########
    # Register 通过这段代码，机器人能够在ROS系统中同步初始化完成的状态和启动时间，以便它们在后续的操作中能够同步进行
    # start time,定义了一个 ROS 参数名称 /start_time，用于存储程序的启动时间。
    str_start_time = '/start_time'
    # Bool: if 'broadcast_comm_his_GS' is created by this client, then True 用于表示是否创建了通信历史参数
    Create_broad = False
    # Bool: if 'start_time' is created by this client, then True 用于表示是否创建了启动时间参数
    Create_start_time = False
    # broadcast the communication history and the init information,str_broad 用于存储广播历史的 ROS 参数名
    # str_broad = '/broadcast_comm_his_GS'
    if not rospy.has_param(str_broad):
        # 'broadcast_comm_his_GS' not be established in the Parameter Server
        broadcast_comm_his_GS = [0 for r2 in range(NUM_ROBOTS*NUM_ROBOTS)]
        # 将当前客户端的通信次数标记为 1
        broadcast_comm_his_GS[(NUM_ROBOTS + 1)*_id] = 1
        rospy.set_param(str_broad, broadcast_comm_his_GS)
        Create_broad = True

    if not Create_broad:
        # 'broadcast_comm_his_GS' has been established, need to update
        # 首先等待 str_broad_lock 参数不存在,然后将其设置为 True。这个锁是为了防止多个客户端同时更新 broadcast_comm_his_GS 参数
        while rospy.has_param(str_broad_lock):
            rospy.sleep(0.1)
        else: rospy.set_param(str_broad_lock, True)
        broadcast_comm_his_GS = rospy.get_param(str_broad)
        # 将当前客户端的通信次数标记为 1,所以最初将state_count = [0, 0, -1]的通信设置为-1，就是为了在此加入一个初始化
        broadcast_comm_his_GS[(NUM_ROBOTS + 1)*_id] = 1
        # 遍历 broadcast_comm_his_GS 列表,检查是否所有机器人的通信次数都大于 0,表示所有机器人都已初始化完成
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
    delay = 0.1
    while start_time is None:
        rospy.sleep(0.1)

    next_motion_time = start_time
    final_time = start_time + total_time
    velocity = [np.random.randn()*sigma_v_input_ + E_v_, np.random.randn()*sigma_omega_input_ + E_omega_]
    # np.random.randn()是NumPy库中用于生成服从标准正态分布（均值为0，标准差为1）的随机数的函数

    while next_motion_time < final_time:
        if time.time() + delay >= next_motion_time:
            msg_robo = f"chassis speed x {velocity[0]} y 0 z {-velocity[1]}"
            s.send(msg_robo.encode('utf-8'))
            buf_robo = s.recv(1024)

            with v_all_lock:
                # v_count初始为-1
                v_count += 1
                v_all[v_count] = velocity

            velocity = [np.random.randn()*sigma_v_input_ + E_v_, np.random.randn()*sigma_omega_input_ + E_omega_]
            next_motion_time += DELTA_T
# -------------------


# TODO 当观测,通讯完成后,可能有些滞后,需要重新update
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


mea_lock = threading.lock()

# 实机实验 用标签间获得的距离代替
def Measurement():
    global mea_all, mea_count
    next_motion_time = start_time + DELTA_T
    while not rospy.is_shutdown():
        measure_time = time.time()
        if measure_time >= next_motion_time:
            next_motion_time += DELTA_T
        else: continue
# --------------------


def Mea_update():
    global state_count, state_alg, cov_alg, algs_meas
    Need_Update = False
    while not rospy.is_shutdown():
        with state_cond:
            if state_count[1] >= state_count[0]:
                # motion has updated to the current, next the meas update
                # Comm update just now
                state_cond.wait()
            else: mea_count_local = state_count[1]

        with mea_lock:
            if mea_count > mea_count_local:
                Need_Update = True
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
            and comm_times < numbers:

            while rospy.has_param(str_broad_lock):
                rospy.sleep(1e-3)
            else: rospy.set_param(str_broad_lock, True)
            broadcast_comm_his_GS = rospy.get_param(str_broad)
            # 说明这是第一次获取通信数据,需要进行一些初始化操作
            if state_comm is None:
                broadcast_comm_his_GS[(NUM_ROBOTS+1)*_id] = comm_times  # 更新当前机器人在广播历史中的通信次数
                for r in range(NUM_ROBOTS):
                    # 如果某个机器人的通信次数超过了 numbers,则减去 numbers。这可能是为了防止通信次数过大导致的数值溢出问题
                    if broadcast_comm_his_GS[NUM_ROBOTS*_id + r] > numbers:
                        broadcast_comm_his_GS[NUM_ROBOTS*_id + r] -= numbers
                rospy.get_param(str_broad, broadcast_comm_his_GS)  # 将更新后的广播历史重新设置回ROS参数服务器
            rospy.delete_param(str_broad_lock) # 删除掉 str_broad_lock 参数,释放对广播历史的独占访问权

            if state_comm is None:
                # 1st: Obtain the belief for comm，这段代码是在处理 state_comm 为 None 的情况,即第一次获取通信数据
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
            
            # 根据广播历史中各机器人的通信次数对机器人ID进行排序，得到 comm_1st_order 列表
            comm_1st_order = sorted(np.arange(NUM_ROBOTS), key = lambda i: broadcast_comm_his_GS[NUM_ROBOTS*_id + i])
            # 检查排序后的第一个机器人是否已经完成了本次通信。如果是，则设置 comm_complete = True
            if broadcast_comm_his_GS[NUM_ROBOTS*_id + comm_1st_order[0]] == comm_times:
                comm_complete = True
            else:
                # 设置消息的接收机器人ID为排序后的第一个机器人
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


def Comm_recv_callback(recv):
    # 首先检查接收到的消息的接收者 ID 是否与当前机器人的 ID 匹配,如果不匹配则直接返回。
    if int(recv.data[0]) != _id: return
    # 初始化接收标志 succ_recv 为 False
    succ_recv = False
    state_recv, cov_recv = {}, {}
    with comm_lock:
        # 检查接收到的消息的通信次数是否与当前机器人的通信次数匹配
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

    
if __name__ == '__main__':
    init()

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
