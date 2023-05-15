#!/usr/bin/python
# -*- coding:utf8 -*-

import os
import time
import cv2, cv_bridge
import rospy
import random
import math
import numpy as np
from enum import Enum
import pytesseract
#from detectColor import detectColor
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
#import matplotlib.pyplot as plt
#import matplotlib.image as mpimg
from std_msgs.msg import Int32


approxPolyDP_epslion=0.02#多边形近似参数，越小越精准
wh_rate=0.4#长宽比系数，越小越接近正方形
min_area=500  #最小面积
max_area=10000  #最大面积
min_center_distance=20#中心距离
fps=0
object_count=50
save_flag=50

msg1 = Int32()

#################话题发布函数###############################
def cmd_send_publish(value1):
    # 初始化ROS节点
    rospy.init_node('cmd_send_publisher', anonymous=True)
    # 创建Int32类型的消息对象
    msg1 = Int32()
    # 将自己设定的数值赋给msg.data
    msg1.data = value1
    # 创建一个发布者对象
    pub = rospy.Publisher('/cmd_send', Int32, queue_size=10)
    # 循环发布消息
    rate = rospy.Rate(10)  # 发布频率为10Hz
    while not rospy.is_shutdown():
        pub.publish(msg1)
        rospy.signal_shutdown("接收到一次cmd话题")
        rate.sleep()

def dispense_window_publish(value2):
    # 初始化ROS节点
    #rospy.init_node('dispense_window_publisher', anonymous=True)
    # 创建Int32类型的消息对象
    msg2 = Int32()
    # 将自己设定的数值赋给msg.data
    msg2.data = value2
    # 创建一个发布者对象
    pub = rospy.Publisher('/dispense_window', Int32, queue_size=10)
    # 循环发布消息
    rate = rospy.Rate(10)  # 发布频率为10Hz
    while not rospy.is_shutdown():
        pub.publish(msg2)
        rospy.signal_shutdown("接收到一次window话题")
        rate.sleep()

def pick_up_num_publish(value3):
    # 初始化ROS节点
    #rospy.init_node('pick_up_num_publisher', anonymous=True)
    # 创建Int32类型的消息对象
    msg3 = Int32()
    # 将自己设定的数值赋给msg.data
    msg3.data = value3
    # 创建一个发布者对象
    pub = rospy.Publisher('/pick_up_num', Int32, queue_size=10)
    # 循环发布消息
    rate = rospy.Rate(10)  # 发布频率为10Hz
    while not rospy.is_shutdown():
        pub.publish(msg3)
        rospy.signal_shutdown("接收到一次pickup话题")
        rate.sleep()



######################计算欧几里得距离函数##########################
def distance(point1, point2):
    """计算距离"""
    return math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)

#################获取定位框函数###############################
def get_locating_point(locating_boxs):
    # 定义两个空列表
    dingweidian=[]
    locating_points=[]
    # 循环定位框列表中的每一个框
    for i in range(len(locating_boxs)):
        # 计算当前框的矩（moments）
        M1 = cv2.moments(locating_boxs[i])
        # 计算当前框的重心坐标
        cx1 = int(M1['m10'] / M1['m00'])   #水平重心
        cy1= int(M1['m01'] / M1['m00'])    #垂直重心
        # 将当前框的重心坐标添加到列表中
        dingweidian.append((cx1,cy1))      #重心位置
    # 将中心距里较近的点合并成一个点
    locating_points=filter_points(dingweidian,10)
    # 返回合并后的点列表
    return locating_points

 # 打开摄像头
cap = cv2.VideoCapture("http://192.168.12.1:8080/stream?topic=/camera/rgb/image_raw")

while (True):
    # 开始用摄像头读数据，返回hx为true则表示读成功，frame为读的图像
    hx, frame = cap.read() 
    
    # 如果hx为Flase表示开启摄像头失败，那么就输出"read vido error"并退出程序
    if hx is False: 
        # 打印报错
        print('read video error')
        # 退出程序
        exit(0)

    #转灰度图
    grayImg=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    display = True  #后面程序执行
    edges = cv2.Canny(grayImg, 50, 150, apertureSize=3)
    _, contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)   #contours为轮廓的坐标点集，边缘端点

    #筛选条件0 面积，保留了面积在500到10000的轮廓坐标点集列表
    contours2=[]
    for i in range(len(contours)):
        M1 = cv2.moments(contours[i])   #计算轮廓几何矩
        if M1['m00']<min_area or M1['m00']>max_area:
            continue
        contours2.append(contours[i])

    #筛选条件1 四边形，保留了闭合的四个四边形
    sibianxing=[]
    for c in contours2:
        # 对轮廓进行多边形逼近
        peri = cv2.arcLength(c, True)   #计算轮廓周长，True表示轮廓闭合
        approx = cv2.approxPolyDP(c, approxPolyDP_epslion * peri, True)  #待逼近曲线，精度，闭合
        hull=approx   #多边形顶点坐标
        if len(hull)==4:
            sibianxing.append(hull)

    #筛选条件2 正方形
    #判断正方形方案1，判断长宽比
    #保留了对四个多边形进行判定为正方形的正方形
    zhengfangxing=[]
    for hull in sibianxing:
        #最小外接矩形
        rect = cv2.minAreaRect(hull)
        (x, y), (w, h), angle = rect  #中心坐标(x, y)；长宽(w, h)；旋转角度angle（表示逆时针旋转的角度）
        abs(w - h)
        if abs(w - h)/(w+h)<wh_rate:#边长归一标准差
            zhengfangxing.append(hull)

    #筛选条件3 正方形的包含关系
    dingweikuang=[]
    jieguo=[]
    # 找面积不为0的正方形，并给出水平和垂直重心，外循环
    for i in range(len(zhengfangxing)-1):
        M1 = cv2.moments(zhengfangxing[i])
        if M1['m00']==0:
             continue
        cx1 = int(M1['m10'] / M1['m00'])
        cy1= int(M1['m01'] / M1['m00'])
        # 找面积不为0的正方形，并给出水平和垂直重心，内循环，找出所有元素对
        for j in range(i+1,len(zhengfangxing)):
            M2 = cv2.moments(zhengfangxing[j])
            if M2['m00']==0:#两者面积为0则跳过
                continue
            cx2 = int(M2['m10'] / M2['m00'])
            cy2 = int(M2['m01'] / M2['m00'])

            #1二者中心互相包含
            res1=cv2.pointPolygonTest(zhengfangxing[i],(cx2,cy2), False)  #计算重心点与多边形的距离
            res2=cv2.pointPolygonTest(zhengfangxing[j],(cx1,cy1), False)
            if res1>0 and res2>0:#如果两个轮廓中心具有包含关系  在内部
                #2计算中心距离
                distance_=distance((cx1,cy1),(cx2,cy2))
                if distance_<min_center_distance:#中心距里小于5个像素
                    dingweikuang+=[zhengfangxing[i],zhengfangxing[j]]
                    jieguo.append(zhengfangxing[i])

    if display:

        dingweikuang_frame=frame.copy()
        if(len(dingweikuang)==48):

            min_x = 1000
            max_x = 0
            min_y = 1000
            max_y = 0
            # min_x_index_group = []
            for i in range(len(jieguo)):
                for j in range(len(jieguo[0])):
                    if jieguo[i][j][0][0] < min_x:
                        min_x = jieguo[i][j][0][0]
                        min_x_index = [i,j]
                        # min_x_index_group.append(min_x_index)
                    if jieguo[i][j][0][0] > max_x:
                        max_x = jieguo[i][j][0][0]
                        max_x_index = [i,j]
                    if jieguo[i][j][0][1] < min_y:
                        min_y = jieguo[i][j][0][1]
                        min_y_index = [i,j]
                    if jieguo[i][j][0][1] > max_y:
                        max_y = jieguo[i][j][0][1]
                        max_y_index = [i,j]


            x0 = jieguo[min_x_index[0]][min_x_index[1]][0][0]
            y0 = jieguo[min_x_index[0]][min_x_index[1]][0][1]
            x1 = jieguo[max_x_index[0]][max_x_index[1]][0][0]
            y1 = jieguo[max_x_index[0]][max_x_index[1]][0][1]
            x2 = jieguo[min_y_index[0]][min_y_index[1]][0][0]
            y2 = jieguo[min_y_index[0]][min_y_index[1]][0][1]
            x3 = jieguo[max_y_index[0]][max_y_index[1]][0][0]
            y3 = jieguo[max_y_index[0]][max_y_index[1]][0][1]

            cv2.circle(dingweikuang_frame,(x0,y0),4,color=(0,0,255),thickness=2)
            cv2.circle(dingweikuang_frame,(x1,y1),4,color=(255,0,0),thickness=2)
            cv2.circle(dingweikuang_frame,(x2,y2),4,color=(255,255,255),thickness=2)
            cv2.circle(dingweikuang_frame,(x3,y3),4,color=(255,255,255),thickness=2)
            
            


            real_box_centers = [(x0,y0),(x1,y1),(x2,y2),(x3,y3)]
            if min_x_index[0] < max_x_index[0]:
                goal_box_centers=[(50,550),(550,50),(50,50),(550,550)]
            else:
                goal_box_centers=[(50,50),(550,550),(550,50),(50,550)]

            dst_pts = np.array(goal_box_centers, dtype=np.float32)
            src_pts = np.array(real_box_centers, dtype=np.float32)
            M = cv2.getPerspectiveTransform(src_pts, dst_pts)
            warped_img = cv2.warpPerspective(dingweikuang_frame, M, (600,600))

            # cv2.imshow('warped_img', warped_img)
            aera1_crop = warped_img[65:255, 65:235]
            # cv2.imshow('aera1_crop', aera1_crop)
            cv2.imwrite("/home/EPRobot/robot_ws/src/eprobot_start/script/aera1.jpg", aera1_crop)
            aera2_crop = warped_img[65:255, 365:535]
            # cv2.imshow('aera2_crop', aera2_crop)
            cv2.imwrite("/home/EPRobot/robot_ws/src/eprobot_start/script/aera2.jpg", aera2_crop)
            aera3_crop = warped_img[335:525, 65:235]
            # cv2.imshow('aera3_crop', aera3_crop)
            cv2.imwrite("/home/EPRobot/robot_ws/src/eprobot_start/script/aera3.jpg", aera3_crop)
            aera4_crop = warped_img[335:525, 365:535]
            # cv2.imshow('aera4_crop', aera4_crop)
            cv2.imwrite("/home/EPRobot/robot_ws/src/eprobot_start/script/aera4.jpg", aera4_crop)
            save_flag -= 1
            print(save_flag)
            print("Found ABC! Result img has saved in robot!")

            # 设置 pytesseract 进行光学字符识别的模式
            # custom_config = r'--psm 6'
            custom_config = r'--psm 6 -c tessedit_char_whitelist=ABC'

            # 读取图片并进行预处理
            img_a = cv2.imread('/home/EPRobot/robot_ws/src/eprobot_start/script/aera1.jpg')
            gray_img1 = cv2.cvtColor(img_a, cv2.COLOR_BGR2GRAY)
            gray_img1 = cv2.threshold(gray_img1, 50, 255, cv2.THRESH_BINARY_INV)[1]
            # 对图片进行光学字符识别
            recognized_text1 = pytesseract.image_to_string(gray_img1, config=custom_config, lang='eng')
            # 输出识别结果

            # 读取图片并进行预处理
            img_b = cv2.imread('/home/EPRobot/robot_ws/src/eprobot_start/script/aera2.jpg')
            gray_img2 = cv2.cvtColor(img_a, cv2.COLOR_BGR2GRAY)
            gray_img2 = cv2.threshold(gray_img2, 50, 255, cv2.THRESH_BINARY_INV)[1]
            # 对图片进行光学字符识别
            recognized_text2 = pytesseract.image_to_string(gray_img2, config=custom_config, lang='eng')
            # 输出识别结果

            # 读取图片并进行预处理
            img_c = cv2.imread('/home/EPRobot/robot_ws/src/eprobot_start/script/aera3.jpg')
            gray_img3 = cv2.cvtColor(img_c, cv2.COLOR_BGR2GRAY)
            gray_img3 = cv2.threshold(gray_img3, 50, 255, cv2.THRESH_BINARY_INV)[1]
            # 对图片进行光学字符识别
            recognized_text3 = pytesseract.image_to_string(gray_img3, config=custom_config, lang='eng')
            # 输出识别结果

            # 读取图片并进行预处理
            img_d = cv2.imread('/home/EPRobot/robot_ws/src/eprobot_start/script/aera4.jpg')
            gray_img4 = cv2.cvtColor(img_d, cv2.COLOR_BGR2GRAY)
            gray_img4 = cv2.threshold(gray_img4, 50, 255, cv2.THRESH_BINARY_INV)[1]
            # 对图片进行光学字符识别
            recognized_text4 = pytesseract.image_to_string(gray_img4, config=custom_config, lang='eng')
            # 输出识别结果
            # 过滤recognized_text1中的非大写字母和空格
            # filtered_text = re.sub("[^A-Z\s]+", "", recognized_text1)
            #

            print(recognized_text1)
            print(recognized_text2)
            print(recognized_text3)
            print(recognized_text4)

            # # 判断第1个框
            # if recognized_text1.strip() == "A":
            #     print('1A')
            #     cmd_send_publish(1)
            #     dispense_window_publish(1)
            #     pick_up_num_publish(6)
            # elif recognized_text1.strip() == "B":
            #     print('1B')
            #     cmd_send_publish(1)
            #     dispense_window_publish(2)
            #     pick_up_num_publish(6)
            # elif recognized_text1.strip() == "C":
            #     print('1C')
            #     cmd_send_publish(1)
            #     dispense_window_publish(0)
            #     pick_up_num_publish(6)
            # else:
            #     print('1JudgeError')
            # # 判断第2个框
            # if recognized_text2.strip() == "A":
            #     print('2A')
            #     cmd_send_publish(1)
            #     dispense_window_publish(1)
            #     pick_up_num_publish(5)
            # elif recognized_text2.strip() == "B":
            #     print('2B')
            #     cmd_send_publish(1)
            #     dispense_window_publish(2)
            #     pick_up_num_publish(5)
            # elif recognized_text2.strip() == "C":
            #     print('2C')
            #     cmd_send_publish(1)
            #     dispense_window_publish(0)
            #     pick_up_num_publish(5)
            # else:
            #     print('2JudgeError')
            # # 判断第3个框
            # if recognized_text3.strip() == "A":
            #     print('3A')
            #     cmd_send_publish(1)
            #     dispense_window_publish(1)
            #     pick_up_num_publish(4)
            # elif recognized_text3.strip() == "B":
            #     print('3B')
            #     cmd_send_publish(1)
            #     dispense_window_publish(2)
            #     pick_up_num_publish(4)
            # elif recognized_text3.strip() == "C":
            #     print('3C')
            #     cmd_send_publish(1)
            #     dispense_window_publish(0)
            #     pick_up_num_publish(4)
            # else:
            #     print('3JudgeError')
            # # 判断第4个框
            # if recognized_text4.strip() == "A":
            #     print('4A')
            #     cmd_send_publish(1)
            #     dispense_window_publish(1)
            #     pick_up_num_publish(3)
            # elif recognized_text4.strip() == "B":
            #     print('4B')
            #     cmd_send_publish(1)
            #     dispense_window_publish(2)
            #     pick_up_num_publish(3)
            # elif recognized_text4.strip() == "C":
            #     print('4C')
            #     cmd_send_publish(1)
            #     dispense_window_publish(0)
            #     pick_up_num_publish(3)
            # else:
            #     print('4JudgeError')
            object_count -= 1
    # 监测键盘输入是否为q，为q则退出程序
    if cv2.waitKey(1) & 0xFF == ord('q'):       # 按q退出
        break


cmd_send_publish(1)
dispense_window_publish(2)
pick_up_num_publish(6)
if object_count == 0:
    exit()
# 释放摄像头
cap.release()

# 结束所有窗口
cv2.destroyAllWindows() 





