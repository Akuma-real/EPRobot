#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Int32

def dispense_window():
    rospy.init_node('publisher_node', anonymous=True)
    dispense_pub = rospy.Publisher('/dispense_window', Int32, queue_size=10)
    rate = rospy.Rate(10) # 发布频率为 10 Hz

    while not rospy.is_shutdown():
    # 生成要发布的消息
        msg = Int32()
        # msg.data = input('窗口号0C1A2B:')
        msg.data = 1
    # 发布消息
        rospy.sleep(5)
        dispense_pub.publish(msg)
        #rospy.signal_shutdown("接收到配药窗口数据")
        rate.sleep()

if __name__ == '__main__':
    try:
        dispense_window()
    except rospy.ROSInternalException:
        pass