#!/usr/bin/env python3
import sys
sys.path.append('../')
from pathlib import Path

import numpy as np
import time
import rospy
import threading
from std_msgs.msg import Float64MultiArray
import parameters as para
from DR import Robot
from DCL_GS import Robot_GS_LRHKF
from geometry_msgs.msg import Twist
from nlink_parser.msg import LinktrackNodeframe2

_id = rospy.get_param('~id', 0) # id
# _id = 0
NUM_ROBOTS = para.NUM_ROBOTS    # robo个数
numbers = para.numbers          # robo运动次数
DELTA_T = para.DELTA_T          # 每次运动时间
total_time = numbers * DELTA_T  # 总共运行时间 包含多次运动
types = para.types              # 算法种类
init_X = para.init_X            # robo初始位置
init_v = para.init_v            # robo初始理论输入速度
np.random.seed(_id)             # 随机种子
flag = 0
'''
flag: int, observation model
1 range-bearing and transform to relative position
0 range-bearing
-1 relative pose
'''

# 式（32）（33）
SIGMA_V_INPUT, SIGMA_OMEGA_INPUT = para.SIGMA_V_INPUT, para.SIGMA_OMEGA_INPUT
E_V, E_OMEGA = para.E_V, para.E_OMEGA

sigma_v_input_, sigma_omega_input_ = SIGMA_V_INPUT, SIGMA_OMEGA_INPUT
E_v_, E_omega_ = E_V, E_OMEGA

alg_count = len(types) # 需要运行几个算法
# -1表示包含DR
if -1 in types:
    alg_count -= 1

# Lock of 'v_all' and 'v_count;Lock of 'state_alg', 'cov_alg' and *_count
v_all_lock = threading.Lock()
state_lock = threading.Lock()


# int, index about where velocity updates 表示最新的速度数据的索引 v_count+1即为一共有多少个速度 保持最新
v_count = -1
v_all = np.zeros((numbers, 2))  # 存储单个robo的速度数据

# dict, list all the states updated by algorithms
state_alg = {}
cov_alg = {}
# motion_count, meas_count, comm_count = 0, 0, 0,用于跟踪 motion、meas 和 comm 三种操作的进度
state_count = [0, 0, -1]
# class of algorithms
algs_motion, algs_meas, algs_comm = {}, {}, {}

# int: After comm: index before 'back_need' should reupdate,用于标记在通信后需要重新更新的状态索引
back_need = -1

# 这意味着在使用条件变量时，必须先获得 state_lock 的锁，然后才能操作条件变量。
state_cond = threading.Condition(lock=state_lock)


# broadcast the communication history and the init information,定义一个字符串 str_broad,用于存储广播历史的 ROS 参数名
str_broad = '/broadcast_comm_his_GS'
# A lock about 'broadcast_comm_his_GS',定义一个字符串 str_broad_lock,用于存储广播历史锁的 ROS 参数名
str_broad_lock = '/broadcast_lock'

# 初始化 start_time 为 None,用于存储所有机器人启动的时间
start_time = None

# 初始化ros节点，发布话题，话题中包含两个字典state_alg = {}、cov_alg = {}
# rospy.init_node('client'+str(_id), anonymous=False)
class TopicSubscriber:
    def __init__(self,_id, topic_name):
        # 初始化 ROS 节点
        rospy.init_node('client'+str(_id), anonymous=False)
        # 订阅话题
        self.sub = rospy.Subscriber(topic_name, LinktrackNodeframe2, self.callback)
        # 存储dis数据
        self.dis_list = []

    def callback(self, msg_data):
        dis_data = msg_data.nodes.dis
        self.dis_list.append(dis_data)  # 存储 node 的 dis 数据
        # self.id = msg_data.id
        # 遍历 nodes 数组并获取 dis 数据  LinktrackNode2[] nodes
        # for node in msg_data.nodes:

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            rate.sleep()
a = TopicSubscriber(_id,'LinktrackNodeframe2_0')
a.run()

# TODO 是否保留
# 创建 ROS 发布者
# state_pub = rospy.Publisher('state_data', String, queue_size=10)
# cov_pub = rospy.Publisher('cov_data', String, queue_size=10)
# # 发布第一个数据
# state_msg = String()
# state_msg.data = str(list(state_alg.items())[0])
# state_pub.publish(state_msg)

# cov_msg = String()
# cov_msg.data = str(list(cov_alg.items())[0])
# cov_pub.publish(cov_msg)
# # rospy.spin()

# # 控制机器人运动
# vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
# # 初始化
# vel_msg = Twist()
# vel_msg.linear.x = 0  # Forward velocity
# vel_msg.angular.z = 0  # No angular velocity initially

# rate = rospy.Rate(30)
# motion_count = 0

# while not rospy.is_shutdown():
#     while motion_count < 10:
#         start_time = rospy.get_time()
#         if (rospy.get_time()-start_time) <= 1:
#             vel_msg.angular.z = 0.01  # Forward velocity
#         if (rospy.get_time()-start_time) <= 2:
#             vel_msg.linear.x = 0.1
#         vel_pub.publish(vel_msg)
#         motion_count += 1
#         rate.sleep()
#     # 停止运动后机器人不动
#     vel_msg.linear.x = 0  # No linear velocity
#     vel_msg.angular.z = 0  # No Angular velocity
#     vel_pub.publish(vel_msg)
#     rate.sleep()

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
    global start_time, state_alg, cov_alg, algs_motion, algs_meas, algs_comm

    meas_bia_prob, comm_fail_prob = para.meas_bia_prob, para.comm_fail_prob
    # print(f"meas_bia_prob = {meas_bia_prob}, comm_fail_prob = {comm_fail_prob}")

    n2 = len(types)  # the number of algorithms
    # Initialize
    if n2:
        for type in types:
            '''
            if type == 20:
                algs_motion[20] = Robot_GS_early_paper(initial_s=init_X, _id=_id, NUM_ROBOTS=NUM_ROBOTS)
                algs_meas[20] = Robot_GS_early_paper(initial_s=init_X, _id=_id, NUM_ROBOTS=NUM_ROBOTS)
                algs_comm[20] = Robot_GS_early_paper(initial_s=init_X, _id=_id, NUM_ROBOTS=NUM_ROBOTS)
            '''
            if type == 28:
                algs_motion[28] = Robot_GS_LRHKF(initial_s=init_X, _id=_id, NUM_ROBOTS=NUM_ROBOTS,flag=0)
                algs_meas[28] = Robot_GS_LRHKF(initial_s=init_X, _id=_id, NUM_ROBOTS=NUM_ROBOTS,flag=0)
                algs_comm[28] = Robot_GS_LRHKF(initial_s=init_X, _id=_id, NUM_ROBOTS=NUM_ROBOTS,flag=0)
            elif type == -1:
                algs_motion[-1] = Robot(X=init_X[_id], _id=_id, NUM_ROBOTS=NUM_ROBOTS)
            # 初始化状态和协方差矩阵
            if type >= 20:
                state_alg[type] = np.zeros((numbers, 3*NUM_ROBOTS))
                cov_alg[type] = np.zeros((numbers, 3*NUM_ROBOTS, 3*NUM_ROBOTS))
                cov_alg[type][0] = algs_motion[type].P_GS.copy()
                state_alg[type][0,:3*NUM_ROBOTS] = np.array(np.reshape(init_X,(1,9)))
            else:
                state_alg[type] = np.zeros((numbers, 3))
                cov_alg[type] = np.zeros((numbers, 3, 3))
                cov_alg[type][0] = algs_motion[type].P.copy()
                state_alg[type][0,:3] = np.array(init_X[_id])

    ######## hardware init starts ##########
    # 1. 初始化，接受三个话题的数据
    # data = []
    # a = TopicSubscriber('LinktrackNodeframe2_0')
    # b = TopicSubscriber('LinktrackNodeframe2_1')
    # c = TopicSubscriber('LinktrackNodeframe2_2')
    # a.run()
    # b.run()
    # c.run()
    # for subscriber in (a,b,c):
    #     if subscriber.id == 1:
    #         data.append(subscriber)


    ######## Already initialize! ########
    # 通过这段代码，机器人能够在ROS系统中同步初始化完成的状态和启动时间，以便它们在后续的操作中能够同步进行
    # 定义了一个 ROS 参数名称 /start_time，用于存储程序的启动时间。
    str_start_time = '/start_time'
    # 用于判断当前节点是否创建了通信历史参数
    # Create_broad = False
    # 用于判断当前节点是否创建了启动时间参数
    Create_start_time = False
    # 用于存储广播历史的 ROS 参数名
    str_broad = '/broadcast_comm_his_GS'
    if not rospy.has_param(str_broad):
        print("broadcast_comm_his_GS不存在")
        # 如果不存在，表示该参数未在参数服务器中创建
        broadcast_comm_his_GS = [0 for r2 in range(NUM_ROBOTS*NUM_ROBOTS)]
        print("创建broadcast_comm_his_GS成功")
        # 将当前客户端的通信次数标记为 1
        broadcast_comm_his_GS[(NUM_ROBOTS + 1)*_id] = 1 # NUM_ROBOTS*_id+_id reshape成矩阵，相当于跟自身的通信次数，以此判断是否初始化成功
        print("将当前客户端的通信次数标记为 1")
        rospy.set_param(str_broad, broadcast_comm_his_GS)
        # 将Create_broad标记为True，表示已创建通信历史参数
        # Create_broad = True

    # if not Create_broad:
    else:
        # broadcast_comm_his_GS参数已被建立
        # 首先检查是否有其他节点在更新 'broadcast_comm_his_GS' 参数，有则等待，没有参数锁存在，则新建参数锁，并将其设置为True
        # 使用参数锁机制确保同步,防止多个客户端同时更新 broadcast_comm_his_GS 参数
        while rospy.has_param(str_broad_lock):
            rospy.sleep(0.1)
        rospy.set_param(str_broad_lock, True)
        print("没有其他客户更新broadcast_comm_his_GS 参数，换我来")
        # 获取 'broadcast_comm_his_GS' 的值
        broadcast_comm_his_GS = rospy.get_param(str_broad)

        # 将当前客户端的通信次数标记为 1,所以最初将state_count = [0, 0, -1]的通信设置为-1，就是为了在此加入一个初始化
        broadcast_comm_his_GS[(NUM_ROBOTS + 1)*_id] = 1 # 同上
        print("broadcast_comm_his_GS存在，通信标记为1")
        # 遍历 broadcast_comm_his_GS 列表,检查是否所有机器人的通信次数都大于 0,表示所有机器人都已初始化完成
        for r in range(NUM_ROBOTS):
            if broadcast_comm_his_GS[(NUM_ROBOTS + 1)*r] <= 0: break
        else:
            # all robots have initialized,
            start_time = time.time() + 5
            rospy.set_param(str_start_time, start_time)
            Create_start_time = True
            print("initial is done")

        rospy.set_param(str_broad, broadcast_comm_his_GS)
        rospy.delete_param(str_broad_lock)

    if not Create_start_time:
        while rospy.has_param(str_start_time):
            # Not all robots have initialized
            rospy.sleep(0.1)
        # start_time = rospy.get_param(str_start_time)
# ---------------------------------

# TODO liuyh 更新代码 控制机器人运动
def motion():
    global v_all, v_count, next_motion_time, start_time,str_start_time
    delay = 0.1
    while start_time is None:
        # 所有机器人还没初始化完成
        rospy.sleep(0.1)
    start_time = rospy.get_param(str_start_time)

    next_motion_time = start_time
    final_time = start_time + total_time
    velocity = [np.random.randn()*sigma_v_input_ + E_v_, np.random.randn()*sigma_omega_input_ + E_omega_]
    # np.random.randn()是NumPy库中用于生成服从标准正态分布（均值为0，标准差为1）的随机数的函数

    while next_motion_time < final_time:
        if time.time() + delay >= next_motion_time:
            with v_all_lock:
                # v_count初始为-1,v_all里提前存储每次运动的速度
                v_count += 1
                v_all[v_count] = velocity
                # 控制机器人运动
                vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
                vel_msg = Twist()
                rate = rospy.Rate(30)
                while not rospy.is_shutdown():
                        start = rospy.get_time()
                        if (rospy.get_time()-start) <= 1:
                            vel_msg.angular.z = v_all[v_count][1]  # Forward velocity
                        if (rospy.get_time()-start) <= 2:
                            vel_msg.linear.x = v_all[v_count][0]
                        vel_pub.publish(vel_msg)
                        rate.sleep()
                        # 停止运动后机器人不动
                        vel_msg.linear.x = 0  # No linear velocity
                        vel_msg.angular.z = 0  # No Angular velocity
                        vel_pub.publish(vel_msg)
                        rate.sleep()

            velocity = [np.random.randn()*sigma_v_input_ + E_v_, np.random.randn()*sigma_omega_input_ + E_omega_]
            next_motion_time += DELTA_T
# -------------------

# condition = threading.Condition()
# measurement_paused = False

# TODO 当观测,通讯完成后,可能有些滞后,需要重新update
def time_propagation():
    global state_alg, cov_alg, state_count, algs_motion
    while not rospy.is_shutdown():
        v_update = None
        with state_lock:
            v_count_local = state_count[0] # 获取当前运动的更新到了哪一步
        with v_all_lock:
            if v_count > v_count_local:                # v_count表示存储的速度次数
                v_update = v_all[v_count_local].copy() # 当前状态更新需要的速度

        if v_update is not None:
            with state_lock:
                for type in types:
                    if state_count[1] >= v_count_local:
                        # time_propagation, the belief after measurement update can be obtained
                        # index of belief in algs_motion equals to v_count_local
                        if type >= 20:
                            algs_motion[type].X_GS[:,0] = state_alg[type][v_count_local].copy()
                            algs_motion[type].P_GS = cov_alg[type][v_count_local].copy()
                        # # TODO ()other type of alg
                    algs_motion[type].motion(v = v_update[0], omega = v_update[1])

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


mea_lock = threading.Lock()
mea_rela_all = np.zeros((numbers, NUM_ROBOTS, 1)) # 只有一个距离
mea_count = 0 # 最新的测量数据的索引，保持最新 下标对应时刻的下标，所以从0开始

# 实机实验标签间获得的距离代替 data为标签获得的dis
def Measurement():
    global mea_rela_all, mea_count
    mea = a.dis_list  # 标签获得的dis数据
    start_time = rospy.get_time()
    next_motion_time = start_time + DELTA_T
    while not rospy.is_shutdown():
        with state_cond:
            if state_count[1] >= state_count[0]:
                state_cond.wait()
        measure_time = time.time()
        if measure_time >= next_motion_time:
            next_motion_time += DELTA_T
        else:
            continue
        _id = 0
        with mea_lock:
            if mea_count < numbers:
                mea_count += 1
                for m in mea:
                    mea_rela_all[mea_count-1 , _id, 0] = m  # 将m赋值给mea_rela_all数组中对应的位置
# --------------------


def Mea_update():
    global state_count, state_alg, cov_alg, algs_meas, mea_rela_all, mea_count
    Need_Update = False
    while not rospy.is_shutdown():
        with state_cond:
            if state_count[1] >= state_count[0]:
                # motion has updated to the current, next the meas update
                # Comm update just now
                state_cond.wait()
            else: mea_count_local = state_count[1]

        with mea_lock:
            # 2种情况：1.常规更新 2.时间传播重新更新
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
    start_time = rospy.get_time()
    pub = rospy.Publisher('comm', Float64MultiArray, queue_size=10) # TODO 不同机器人发布和接收不同话题
    while not rospy.is_shutdown():
        curr_comm_times = time.time() - start_time
        if not comm_complete and \
            (curr_comm_times > comm_interval * comm_times and curr_comm_times < comm_interval * (comm_times + 1)) \
            and comm_times < numbers/COMM_RATE:

            while rospy.has_param(str_broad_lock):
                rospy.sleep(1e-3)
            else: rospy.set_param(str_broad_lock, True)
            broadcast_comm_his_GS = rospy.get_param(str_broad)
            # 说明这是第一次发布通信数据,需要进行一些初始化操作
            if state_comm is None:
                broadcast_comm_his_GS[(NUM_ROBOTS+1)*_id] = comm_times  # 使得对自己通信的次数最大
                for r in range(NUM_ROBOTS):
                    # 如果某个机器人的通信次数超过了 numbers,则减去 numbers。这是由于上次已经更新后设置为+numbers TODO 是否保留
                    if broadcast_comm_his_GS[NUM_ROBOTS*_id + r] > numbers:
                        broadcast_comm_his_GS[NUM_ROBOTS*_id + r] -= numbers
                rospy.set_param(str_broad, broadcast_comm_his_GS)  # 将更新后的广播历史重新设置回ROS参数服务器
            rospy.delete_param(str_broad_lock) # 删除掉 str_broad_lock 参数,释放对广播历史的独占访问权

            if state_comm is None:
                # 1st: Obtain the belief for comm，这段代码是在处理 state_comm 为 None 的情况,即开始发布通信数据
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
            # 检查排序后的第一个机器人是否已经完成了本次通信，说明所有机器人在comm_times下已完成通信。如果是，则设置 comm_complete = True
            if broadcast_comm_his_GS[NUM_ROBOTS*_id + comm_1st_order[0]] == comm_times:
                comm_complete = True
            else:
                # 设置消息的接收机器人ID为排序后的第一个机器人
                message.data[0] = comm_1st_order[0]
                message[1] = _id
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
                # with state_lock:
                #     state_alg[type][comm_times*COMM_RATE-1] = state_comm[type].copy()
                #     cov_alg[type][comm_times*COMM_RATE-1] = cov_comm[type].copy()
                #     state_count[0] = comm_times*COMM_RATE-1
                #     state_count[1] = comm_times*COMM_RATE-1
                    # back_need = comm_times*COMM_RATE-1

                state_comm, cov_comm = None, None
                comm_count += 1
            comm_times += 1
            comm_complete = False


def Comm_recv_callback(recv):
    # 首先检查接收到的消息的接收者 ID 是否与当前机器人的 ID 匹配,如果不匹配则直接返回。
    if int(recv.data[0]) != _id: return
    global state_comm, cov_comm
    # 初始化接收标志 succ_recv 为 False
    succ_recv = False
    state_recv, cov_recv = {}, {}

    while rospy.has_param(str_broad_lock):
        rospy.sleep(1e-3)
    else: rospy.set_param(str_broad_lock, True)
    broadcast_comm_his_GS = rospy.get_param(str_broad)
    comm_self = broadcast_comm_his_GS[NUM_ROBOTS*recv.data[1] + _id].copy()
    rospy.delete_param(str_broad_lock)

    # 检查接收到的消息的通信次数是否与当前机器人的通信次数匹配
    if comm_self < int(recv.data[2]):
        with comm_lock:
            # if int(recv.data[2]) == comm_count:
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

        # with comm_lock:
        #     for type in types:
        #         state_comm[type] = algs_comm[type].X_GS.copy()
        #         cov_comm[type]=  algs_comm[type].P_GS.copy()
            if can:
                with state_lock:
                    state_alg[type][recv.data[2]*COMM_RATE-1] = state_comm[type].copy()
                    cov_alg[type][recv.data[2]*COMM_RATE-1] = cov_comm[type].copy()
                    state_count[0] = recv.data[2]*COMM_RATE-1
                    state_count[1] = recv.data[2]*COMM_RATE-1


def Comm_recv():
    rospy.Subscriber('comm', Float64MultiArray, Comm_recv_callback)
    rospy.spin()


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

    while(not rospy.is_shutdown()):
        # TODO thread.join
        thread_motion.join()
        thread_meas.join()
        thread_propagation.join()
        thread_meas_update.join()
        thread_comm_send.join()
        thread_comm_recv.join()
