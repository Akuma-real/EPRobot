#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Int32

def pick_up_num():
    rospy.init_node('publisher_node',anonymous=True)
    pub = rospy.Publisher('/pick_up_num', Int32, queue_size=10)
    rate = rospy.Rate(10)  # 发布频率为 10 Hz

    while not rospy.is_shutdown():
        # 生成要发布的消息
        msg1 = Int32()
        # msg1.data = input('3456对应取药点4321:')  # 这里假设要发布的数据是 42
        msg1.data = 4
        # 发布消息
        rospy.sleep(5)
        pub.publish(msg1)
        #rospy.signal_shutdown("接收到取药点数据")
        rate.sleep()


if __name__ == '__main__':
    try:
        pick_up_num()
    except rospy.ROSInternalException:
        pass