#!/usr/bin/python
# -*- coding:utf8 -*-

import os
import time
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

def main():
    rospy.init_node('image_publisher', anonymous=True)
    pub = rospy.Publisher('/camera/image_raw', Image, queue_size=10)
    bridge = CvBridge()
    rate = rospy.Rate(5)

    cap = cv2.VideoCapture("http://192.168.12.1:8080/stream?topic=/camera/rgb/image_raw")  # 设置摄像头设备，如果有多个设备需要指定

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if ret:
            pub.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))
        rate.sleep()

    cap.release()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
