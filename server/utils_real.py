import sys
sys.path.append('../')
from pathlib import Path

import numpy as np
from math import pi
import rospy
import matplotlib.pyplot as plt
import scienceplots

from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


import parameters as para

# def recv


NUM_ROBOTS = para.NUM_ROBOTS
start_time = None

def init(init_X):
    
    
    global start_time
    # Initialize
    true_robots = init_X.copy()
    
    
    str_start_time = '/start_time'
    while not rospy.has_param(str_start_time):
        rospy.sleep(0.1)
    start_time = rospy.get_param(str_start_time)


marker = Marker()
# marker.header.frame_id = "base_link"  # Set the frame ID
# marker.header.stamp = rospy.Time.now()
marker.type = Marker.POINTS
marker.action = Marker.ADD
marker.pose.orientation.w = 1.0
marker.scale.x = 0.1  # Point size
marker.scale.y = 0.1
marker.color.r = 1.0  # Red color
marker.color.a = 1.0  # Full opacity

def draw_true(data):
    # TODO 接收真值 TBC
    point = Point()
    point.x = data.x
    point.y = data.y
    point.z = data.z

if __name == '__main__':

    rospy.init_node('server', anonymous=False) # 

    rospy.Subscriber('true', Float64MultiArray, draw_true)

    # for r in range(NUM_ROBOTS):
    #     rospy.Subscriber('client' + str(r), Float64MultiArray, draw_tra)
    
    rospy.spin()




    
    
