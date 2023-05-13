#! /usr/bin/env python2
# -*- coding: utf-8 -*-
#from __future__ import print_function
import os
import time


from geometry_msgs.msg import Twist, Vector3
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
#import geometry_msgs/PoseWithCovarianceStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import CompressedImage

#import re
import rospy
#import sys
import math
import serial
import string

import actionlib

from std_msgs.msg import  Float64
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class MOVE_ARRIVE:
    def __init__(self):
        #Get params
        
        self.teb_vel_topic= rospy.get_param('~local_vel_topic','/cmd_vel_local')
        self.plan_vel_topic= rospy.get_param('~plan_vel_topic','/cmd_vel_plan')
        self.cmd_vel_topic= rospy.get_param('~cmd_vel_topic','/cmd_vel')
        self.color = rospy.get_param("color_flag_sjq")
        
        self.flag = 0
        self.sendgoal_flag = 0
        self.waitflag = 0
        
        self.px = 0.0
        self.py = 0.0
        self.pz = 0.0
        
        self.px5 = 0.0
        self.py5 = 0.0
        self.pz5 = 0.0

        self.this_pose_x = 0
        self.this_pose_y = 0
        self.this_pose_z = 0
        
        self.trans_x = 0.0
        self.trans_y = 0.0
        self.rotat_z = 0.0

        self.local_trans_x = 0.0
        self.local_trans_y = 0.0
        self.local_rotat_z = 0.0

        self.plan_trans_x = 0.0
        self.plan_trans_y = 0.0
        self.plan_rotat_z = 0.0
        
        self.color = 0

        self.amcl_subsciber = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self._pose_info)
        
        self.local_vel_sub = rospy.Subscriber(self.teb_vel_topic,Twist,self.cmdlocalCB,queue_size=20)
        self.plan_vel_sub = rospy.Subscriber(self.plan_vel_topic,Twist,self.cmdplanCB,queue_size=20)
        self.goal_pub = rospy.Publisher("/move_base_simple/goal",PoseStamped,queue_size=10)
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_topic,Twist,queue_size=10)
        
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

        time.sleep(2.2)

    def cmdlocalCB(self,data):
        self.teb_trans_x = data.linear.x  #*6
        self.teb_trans_y = data.linear.y
        self.teb_rotat_z = data.angular.z
        
        
    def cmdplanCB(self,data):
        self.plan_trans_x = data.linear.x  #*6
        self.plan_trans_y = data.linear.y
        self.plan_rotat_z = data.angular.z
        
        vel_msg = Twist()
        vel_msg.linear.x = self.plan_trans_x
        vel_msg.linear.y = self.plan_trans_y
        vel_msg.angular.z = self.plan_rotat_z
        self.cmd_vel_pub.publish(vel_msg)
        
    def _goal_pose1(self):
        self.px = -1.357
        self.py = 1.865
        self.pz = -3.098
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.pose.position.x = self.px
        self.goal.target_pose.pose.position.y = self.py
        self.goal.target_pose.pose.position.z = 0
        self.goal.target_pose.pose.orientation.x = 0
        self.goal.target_pose.pose.orientation.y = 0
        self.goal.target_pose.pose.orientation.z = math.cos(self.pz/2)
        self.goal.target_pose.pose.orientation.w = math.sin(self.pz/2)
        
        self.goal_msg = PoseStamped()
        self.goal_msg.header.frame_id = 'map'
        self.goal_msg.pose.position.x = self.px
        self.goal_msg.pose.position.y = self.py
        self.goal_msg.pose.position.z = 0
        self.goal_msg.pose.orientation.x = 0
        self.goal_msg.pose.orientation.y = 0
        self.goal_msg.pose.orientation.z = math.cos(self.pz/2)
        self.goal_msg.pose.orientation.w = math.sin(self.pz/2)
        
    def _goal_pose2(self):
        self.px = -1.010
        self.py = 0.790
        self.pz = 0.007
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.pose.position.x = self.px
        self.goal.target_pose.pose.position.y = self.py
        self.goal.target_pose.pose.position.z = 0
        self.goal.target_pose.pose.orientation.x = 0
        self.goal.target_pose.pose.orientation.y = 0
        self.goal.target_pose.pose.orientation.z = math.cos(self.pz/2)
        self.goal.target_pose.pose.orientation.w = math.sin(self.pz/2)
        
        self.goal_msg = PoseStamped()
        self.goal_msg.header.frame_id = 'map'
        self.goal_msg.pose.position.x = self.px
        self.goal_msg.pose.position.y = self.py
        self.goal_msg.pose.position.z = 0
        self.goal_msg.pose.orientation.x = 0
        self.goal_msg.pose.orientation.y = 0
        self.goal_msg.pose.orientation.z = math.cos(self.pz/2)
        self.goal_msg.pose.orientation.w = math.sin(self.pz/2)
        
    def _goal_pose3(self):
        self.px = 2.199
        self.py = 0.884
        self.pz = 0.140
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.pose.position.x = self.px
        self.goal.target_pose.pose.position.y = self.py
        self.goal.target_pose.pose.position.z = 0
        self.goal.target_pose.pose.orientation.x = 0
        self.goal.target_pose.pose.orientation.y = 0
        self.goal.target_pose.pose.orientation.z = math.cos(self.pz/2)
        self.goal.target_pose.pose.orientation.w = math.sin(self.pz/2)
        
        self.goal_msg = PoseStamped()
        self.goal_msg.header.frame_id = 'map'
        self.goal_msg.pose.position.x = self.px
        self.goal_msg.pose.position.y = self.py
        self.goal_msg.pose.position.z = 0
        self.goal_msg.pose.orientation.x = 0
        self.goal_msg.pose.orientation.y = 0
        self.goal_msg.pose.orientation.z = math.cos(self.pz/2)
        self.goal_msg.pose.orientation.w = math.sin(self.pz/2)
        
    def _goal_pose4(self):
        self.px = 2.284
        self.py = 1.863
        self.pz = -3.139
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.pose.position.x = self.px
        self.goal.target_pose.pose.position.y = self.py
        self.goal.target_pose.pose.position.z = 0
        self.goal.target_pose.pose.orientation.x = 0
        self.goal.target_pose.pose.orientation.y = 0
        self.goal.target_pose.pose.orientation.z = math.cos(self.pz/2)
        self.goal.target_pose.pose.orientation.w = math.sin(self.pz/2)
        
        self.goal_msg = PoseStamped()
        self.goal_msg.header.frame_id = 'map'
        self.goal_msg.pose.position.x = self.px
        self.goal_msg.pose.position.y = self.py
        self.goal_msg.pose.position.z = 0
        self.goal_msg.pose.orientation.x = 0
        self.goal_msg.pose.orientation.y = 0
        self.goal_msg.pose.orientation.z = math.cos(self.pz/2)
        self.goal_msg.pose.orientation.w = math.sin(self.pz/2)
    
    def _goal_pose5(self): 
        self.px = 0.355
        self.py = -0.102
        self.pz = 0.025
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.pose.position.x = self.px
        self.goal.target_pose.pose.position.y = self.py
        self.goal.target_pose.pose.position.z = 0
        self.goal.target_pose.pose.orientation.x = 0
        self.goal.target_pose.pose.orientation.y = 0
        self.goal.target_pose.pose.orientation.z = math.cos(self.pz/2)
        self.goal.target_pose.pose.orientation.w = math.sin(self.pz/2)
        
        self.goal_msg = PoseStamped()
        self.goal_msg.header.frame_id = 'map'
        self.goal_msg.pose.position.x = self.px
        self.goal_msg.pose.position.y = self.py
        self.goal_msg.pose.position.z = 0
        self.goal_msg.pose.orientation.x = 0
        self.goal_msg.pose.orientation.y = 0
        self.goal_msg.pose.orientation.z = math.cos(self.pz/2)
        self.goal_msg.pose.orientation.w = math.sin(self.pz/2)
    
    def _goal_pose6(self): 
        self.px = 0.360
        self.py = 1.874
        self.pz = -3.123
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.pose.position.x = self.px
        self.goal.target_pose.pose.position.y = self.py
        self.goal.target_pose.pose.position.z = 0
        self.goal.target_pose.pose.orientation.x = 0
        self.goal.target_pose.pose.orientation.y = 0
        self.goal.target_pose.pose.orientation.z = math.cos(self.pz/2)
        self.goal.target_pose.pose.orientation.w = math.sin(self.pz/2)
        
        self.goal_msg = PoseStamped()
        self.goal_msg.header.frame_id = 'map'
        self.goal_msg.pose.position.x = self.px
        self.goal_msg.pose.position.y = self.py
        self.goal_msg.pose.position.z = 0
        self.goal_msg.pose.orientation.x = 0
        self.goal_msg.pose.orientation.y = 0
        self.goal_msg.pose.orientation.z = math.cos(self.pz/2)
        self.goal_msg.pose.orientation.w = math.sin(self.pz/2)
    
    def _pose_info(self, data):
        self.this_pose_x = data.pose.pose.position.x
        self.this_pose_y = data.pose.pose.position.y
        self.this_pose_z = data.pose.pose.position.z
        
        #print('pose_info')
    
    
    def send_goal1(self):
        self._goal_pose1()
        self.client.send_goal(self.goal)
        self.goal_pub.publish(self.goal_msg) 
        print('发现目标点一')
    def send_goal2(self):
        self._goal_pose2()
        self.client.send_goal(self.goal)
        self.goal_pub.publish(self.goal_msg) 
        print('发现目标点二')
    def send_goal3(self):
        self._goal_pose3()
        self.client.send_goal(self.goal)
        self.goal_pub.publish(self.goal_msg) 
        print('发现目标点三')
    def send_goal4(self):
        self._goal_pose4()
        self.client.send_goal(self.goal)
        self.goal_pub.publish(self.goal_msg) 
        print('发现目标点四')
    def send_goal5(self):
        self._goal_pose5()
        self.client.send_goal(self.goal)
        self.goal_pub.publish(self.goal_msg) 
        print('寻找找维修站')
    def send_goal6(self):
        self._goal_pose6()
        self.client.send_goal(self.goal)
        self.client.send_goal(self.goal)
        self.goal_pub.publish(self.goal_msg)
        print('寻找到终点')
    
                                   
    def sendgoalCB(self):
        while not(rospy.is_shutdown() or self.color):
            self.color = rospy.get_param("color_flag_sjq",0)
            print('color = {colors}'.format(colors = self.color))
            print('请放置绿色信号旗')
            time.sleep(2.5)
        if self.color == 1:
            print('识别到绿色信号旗，小车五秒后出发')
            time.sleep(5)
            self.send_goal1()
            self.sendgoal_flag = 2
            for i in range(7): 
                self.flag = 0
                self.color = rospy.get_param("color_flag_sjq",0)
                while not (rospy.is_shutdown() or self.flag):
                    if self.sendgoal_flag == 2 and self.this_pose_x <= -0.383 and self.this_pose_x >= -1.04 and self.this_pose_y >= 1.55 and self.this_pose_y <= 2.28 :                   
                            self.send_goal2()
                            self.sendgoal_flag = 3
                            time.sleep(0.1)
                    if self.color == 9 and self.this_pose_x >= -2.49 and self.this_pose_x <= -1.52 and self.this_pose_y >= 0.358 and self.this_pose_y <= 1.18 :  
                            print('识别到黄旗，准备进入维修站')
                            self.send_goal5()
                            self.weixiuzhan = 0   
                            self.sendgoal_flag = 3
                            self.waitflag = 1
                            time.sleep(15) #在维修站停止5秒
                            print('离开维修站')
                    if self.sendgoal_flag == 3 and ((self.this_pose_x >= -2.49 and self.this_pose_x <= -1.52 and self.this_pose_y >= 0.358 and self.this_pose_y <= 1.18) or (self.this_pose_x >= -0.438 and self.this_pose_x <= 1.07 and self.this_pose_y >= -0.475 and self.this_pose_y <= 0.379)) :
                            self.send_goal3()
                            self.sendgoal_flag = 4
                         #   self.color = 0
                            time.sleep(0.1)
                    if self.sendgoal_flag == 4 and self.this_pose_x >= 0.919 and self.this_pose_x <= 2.04 and self.this_pose_y >= -0.318 and self.this_pose_y <= 1.18 :
                            self.send_goal4()
                            self.sendgoal_flag = 5
                            time.sleep(0.1)   
                    if self.sendgoal_flag == 5 and i!=6 and self.this_pose_x >= 2.48 and self.this_pose_x <= 3.21 and self.this_pose_y >= 1.53 and self.this_pose_y <= 2.3 :
                            print('第{quan}圈'.format(quan = i))
                            self.send_goal1()
                            self.sendgoal_flag = 2
                            time.sleep(0.1)
                            self.flag = 1
                            self.color_flag = rospy.get_param("color_flag_sjq",0)
                            self.color = self.color_flag
                    if i == 6:
                        self.send_goal6()
                        self.flag = 1
            
        
        
        
#main function
if __name__=="__main__":
    try:
        rospy.init_node("RuntimeError",anonymous=True)
        rospy.loginfo('navigation_target instruction start...')
        MA = MOVE_ARRIVE()
        MA.sendgoalCB()
        print("OK")
        rospy.spin()
    except KeyboardInterrupt:
        #ma.serial.close
        print("Shutting down")


#2341