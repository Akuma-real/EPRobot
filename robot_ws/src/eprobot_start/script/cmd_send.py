#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Int32

def cmd_send():
    pub = rospy.Publisher("cmd_send",Int32,queue_size=10)
    rospy.init_node("cmd_send",anonymous=True)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        msg = Int32()
        # msg.data =  input('流程号1357:')  # 这里假设要发布的数据是 1
        msg.data = 1
    # 发布消息
        pub.publish(msg)
        # rospy.signal_shutdown("接收到cmd数据")
        rate.sleep()

if __name__ == '__main__':
    try:
        cmd_send()
    except rospy.ROSInternalException:
        pass
