#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64

def log_callback(data):
    rospy.loginfo("rospy.Time.now().to_sec() = %f", data.data)

if __name__ == '__main__':
    rospy.init_node('server', anonymous=False)
    rospy.Publisher('Time', Float64, log_callback)
        