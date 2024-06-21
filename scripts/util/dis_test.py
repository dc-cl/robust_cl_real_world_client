import rospy
from nlink_parser.msg import LinktrackNodeframe2


class TopicSubscriber:
    def __init__(self, topic_name):
        # 初始化 ROS 节点
        rospy.init_node('client', anonymous=False)
        # 订阅话题
        self.sub = rospy.Subscriber(topic_name, LinktrackNodeframe2, self.callback)
        # 存储dis数据
        self.dis_list = []

    def callback(self, msg_data):
        nodes = msg_data.nodes
        for node in nodes:
            self.dis_list.append(node.dis)  # 存储 node 的 dis 数据
        # 遍历 nodes 数组并获取 dis 数据  LinktrackNode2[] nodes
        # for node in msg_data.nodes:

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':
    a = TopicSubscriber('LinktrackNodeframe2_0')
    print(a.dis_list)