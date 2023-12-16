#!/usr/bin/env python
import rospy
import time
from std_msgs.msg import Float64

_id = 0

if __name__ == '__main__':
    rospy.init_node('client' + str(_id), anonymous=False)
    pub = rospy.Publisher('Time', Float64, queue_size=10)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        pub.publish(rospy.Time.now().to_sec())
        # pub.publish(time.time())
        rate.sleep()
        