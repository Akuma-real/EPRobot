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
#from detectColor import detectColor
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
#import matplotlib.pyplot as plt
#import matplotlib.image as mpimg

approxPolyDP_epslion=0.02#多边形近似参数，越小越精准
wh_rate=0.4#长宽比系数，越小越接近正方形
min_area=500
max_area=20000

min_center_distance=20#中心距离

fps=0


def distance(point1, point2):
    """计算距离"""
    return math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)

def get_locating_point(locating_boxs):
    """
    这个函数名为 get_locating_point，它的作用是获取定位框的中心点。传入的参数 locating_boxs 是一个包含若干个定位框的列表，每个定位框是一个由点坐标构成的列表。
    该函数首先定义了一个空列表 dingweidian 和一个空列表 locating_points。然后遍历传入的所有定位框，通过 OpenCV 的 cv2.moments 函数计算每个定位框的中心点坐标，并将其添加到 dingweidian 列表中。
    接着调用了 filter_points 函数对 dingweidian 中的点进行筛选，把距离较近的点合并成一个点，最终将得到的筛选后的点添加到 locating_points 列表中。
    最后返回 locating_points 列表，其中包含了所有定位框的中心点坐标。
    """
    # 定义两个空列表
    dingweidian=[]
    locating_points=[]
    # 循环定位框列表中的每一个框
    for i in range(len(locating_boxs)):
        # 计算当前框的矩（moments）
        M1 = cv2.moments(locating_boxs[i])
        # 计算当前框的重心坐标
        cx1 = int(M1['m10'] / M1['m00'])
        cy1= int(M1['m01'] / M1['m00'])
        # 将当前框的重心坐标添加到列表中
        dingweidian.append((cx1,cy1))
    # 将中心距里较近的点合并成一个点
    locating_points=filter_points(dingweidian,10)
    # 返回合并后的点列表
    return locating_points


 # 打开摄像头
# cap = cv2.VideoCapture(0) 
cap = cv2.VideoCapture("http://192.168.12.239:8080/stream?topic=/camera/rgb/image_raw")


while (True):

    # 开始用摄像头读数据，返回hx为true则表示读成功，frame为读的图像
    hx, frame = cap.read() 
    
    # 如果hx为Flase表示开启摄像头失败，那么就输出"read vido error"并退出程序
    if hx is False: 
        # 打印报错
        print('read video error')
        # 退出程序
        exit(0)
    
    grayImg=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
     
    # 显示摄像头图像，其中的video为窗口名称，frame为图像
    # cv2.imshow('gray', grayImg)

    # gradX = cv2.Sobel(grayImg, ddepth=cv2.CV_32F, dx=1, dy=0, ksize=-1)
    # gradY = cv2.Sobel(grayImg, ddepth=cv2.CV_32F, dx=0, dy=1, ksize=-1)
    # gradient = cv2.subtract(gradX, gradY)
    # gradient = cv2.convertScaleAbs(gradient)
    
    # cv2.imshow('gradient', gradient)

    # blurred = cv2.blur(gradient, (2, 2))
    # retval, gray = cv2.threshold(blurred, 100, 255, cv2.THRESH_BINARY_INV)

    # cv2.imshow('grey', gray)

    display = True

    # grayImg or gray

    edges = cv2.Canny(grayImg, 50, 150, apertureSize=3)
    #edges = gray
    
    _, contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    #筛选条件0 面积
    contours2=[]
    for i in range(len(contours)):
        M1 = cv2.moments(contours[i])
        if M1['m00']<min_area or M1['m00']>max_area:
            continue
        contours2.append(contours[i])

    
    #筛选条件1 四边形
    sibianxing=[]
    for c in contours2:
        # 对轮廓进行多边形逼近
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, approxPolyDP_epslion * peri, True)
        hull=approx
        if len(hull)==4:
            sibianxing.append(hull)

    
    #筛选条件2 正方形
    #判断正方形方案1，判断长宽比
    zhengfangxing=[]
    for hull in sibianxing:

        rect = cv2.minAreaRect(hull)
        (x, y), (w, h), angle = rect
        abs(w - h)
        if abs(w - h)/(w+h)<wh_rate:#边长归一标准差
            zhengfangxing.append(hull)

    #筛选条件3 正方形的包含关系
    dingweikuang=[]
    jieguo=[]
    for i in range(len(zhengfangxing)-1):
        M1 = cv2.moments(zhengfangxing[i])
        if M1['m00']==0:
             continue
        cx1 = int(M1['m10'] / M1['m00'])
        cy1= int(M1['m01'] / M1['m00'])
        for j in range(i+1,len(zhengfangxing)):
            M2 = cv2.moments(zhengfangxing[j])
            if M2['m00']==0:#两者面积为0则跳过
                continue
            cx2 = int(M2['m10'] / M2['m00'])
            cy2 = int(M2['m01'] / M2['m00'])
            
            #1二者中心互相包含
            res1=cv2.pointPolygonTest(zhengfangxing[i],(cx2,cy2), False)
            res2=cv2.pointPolygonTest(zhengfangxing[j],(cx1,cy1), False)
            if res1>0 and res2>0:#如果两个轮廓中心具有包含关系         
                #2计算中心距离
                distance_=distance((cx1,cy1),(cx2,cy2))
                if distance_<min_center_distance:#中心距里小于5个像素      
                    dingweikuang+=[zhengfangxing[i],zhengfangxing[j]]
                    jieguo.append(zhengfangxing[i])
    if display:
        # cv2.imshow('step1_edges', edges)
        
        # contours_frame=frame.copy()
        # cv2.drawContours(contours_frame,contours, -1, (0, 0, 255), 1)
        # cv2.putText(contours_frame, "counts:{}".format(len(contours)), (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        # cv2.imshow('step2_all_contours', contours_frame)
       
        # contours2_frame=frame.copy()
        # cv2.drawContours(contours2_frame,contours2, -1, (0, 0, 255), 1)
        # "FPS: {:.2f}".format(fps)
        # cv2.putText(contours2_frame, "min_area:{},max_area:{},counts:{}".format(min_area,max_area,len(contours2)), (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        # cv2.imshow('step3_Area_Selected_contours', contours2_frame)  
        
        # sibianxing_frame=frame.copy()
        # cv2.drawContours(sibianxing_frame,sibianxing, -1, (0, 0, 255), 1)
        # cv2.putText(sibianxing_frame, "approxPolyDP_epslion:{:.2f},counts:{}".format(approxPolyDP_epslion,len(sibianxing)), (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        # cv2.imshow('step4_quadrilateral_selection', sibianxing_frame)  
        
        # zhengfangxing_frame=frame.copy()
        # cv2.drawContours(zhengfangxing_frame,zhengfangxing, -1, (0, 0, 255), 1)
        # cv2.putText(zhengfangxing_frame, "wh_rate:{:.2f},counts:{}".format(wh_rate,len(zhengfangxing)), (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        # cv2.imshow('step5_Square_selection', zhengfangxing_frame)  
        
        dingweikuang_frame=frame.copy()
        if(len(dingweikuang)==48):
            # print(len(dingweikuang))
            # print(dingweikuang[0][2][0][0] , dingweikuang[len(dingweikuang)-1][0][0][1])
            # print("rst ------ rst")
            # x0 = dingweikuang[0][2][0][0]
            # y0 = dingweikuang[0][2][0][1]
            # x1 = dingweikuang[15][3][0][0]
            # y1 = dingweikuang[15][3][0][1]
            # x12 = dingweikuang[15][1][0][0]
            # y12 = dingweikuang[15][1][0][1]
            # x2 = dingweikuang[len(dingweikuang)-1][0][0][0]
            # y2 = dingweikuang[len(dingweikuang)-1][0][0][1]
            # x3 = dingweikuang[31][1][0][0]
            # y3 = dingweikuang[31][1][0][1]
            # x32 = dingweikuang[31][3][0][0]
            # y32 = dingweikuang[31][3][0][1]
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
            # print(len(min_x_index_group))
            # print(min_x,max_x,min_y,max_y)
            # print(min_x_index,max_x_index,min_y_index,max_y_index)

            x0 = jieguo[min_x_index[0]][min_x_index[1]][0][0]
            y0 = jieguo[min_x_index[0]][min_x_index[1]][0][1]
            x1 = jieguo[max_x_index[0]][max_x_index[1]][0][0]
            y1 = jieguo[max_x_index[0]][max_x_index[1]][0][1]
            x2 = jieguo[min_y_index[0]][min_y_index[1]][0][0]
            y2 = jieguo[min_y_index[0]][min_y_index[1]][0][1]
            x3 = jieguo[max_y_index[0]][max_y_index[1]][0][0]
            y3 = jieguo[max_y_index[0]][max_y_index[1]][0][1]
            # print("x0 y0 :")
            # print(min_x_index)
            # print("x1 y1 :")
            # print(max_x_index)
            # print("x2 y2 :")
            # print(min_y_index)
            # print("x3 y3 :")
            # print(max_y_index)
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
            
            print("Found ABC! Result img has saved in robot!")
            
        # cv2.drawContours(dingweikuang_frame,dingweikuang, -1, (0, 0, 255), 1)
        # cv2.putText(dingweikuang_frame, "min_center_distance:{:.2f},counts:{}".format(min_center_distance,len(dingweikuang)), (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        # cv2.imshow('step6_Positioning_box_selection', dingweikuang_frame) 
        

       
    # 监测键盘输入是否为q，为q则退出程序
    if cv2.waitKey(1) & 0xFF == ord('q'):       # 按q退出
        break

# 释放摄像头
cap.release()

# 结束所有窗口
cv2.destroyAllWindows() 





