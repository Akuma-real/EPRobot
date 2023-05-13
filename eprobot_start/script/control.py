#!/usr/bin/env python
# coding:utf-8
from time import sleep
import roslaunch
import time
import rospy
import serial

camera_launch_path = "/home/EPRobot/robot_ws/src/eprobot_start/launch/test.launch"
voice_launch_path = "/home/EPRobot/robot_ws/src/xf_mic_asr_offline/launch/voice_control.launch"

pre_F1_demo_path = "/home/EPRobot/robot_ws/src/eprobot_start/launch/test1.launch"
F1_demo_path = "/home/EPRobot/robot_ws/src/eprobot_start/launch/F1_d.launch"
#voice_open_flag = 0
camera_open_flag = 0

def play_music(number):
    serial_player = serial.Serial('/dev/EPRobot_base', 9600, timeout=10)
    output = chr(0xAA) + chr(0X07) + chr(0x02) + chr(0x00) + chr(number) + chr(0xB3 + number)
    time.sleep(0.5)
    serial_player.write(output)
    #serial_player.close()

def set_launch(path):
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    tracking_launch = roslaunch.parent.ROSLaunchParent(uuid,[path])
    return tracking_launch

def talker():
    rospy.init_node("sjq_tier",anonymous=False)
    rospy.set_param("color_flag_sjq", 0)  #设置参数服务器�?    rospy.set_param("mic_voice", 0)  # 设置参数服务器�?    #=================
    # uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    # roslaunch.configure_logging(uuid)
    # tracking_launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/EPRobot/robot_ws/src/eprobot_start/launch/test.launch"])
    play_music(3)

    voice_launch = set_launch(voice_launch_path)   #打开语音
    voice_launch.start()
    #=================
    while not rospy.is_shutdown():
            color_int = rospy.get_param("color_flag_sjq",0)
            voice_int = rospy.get_param("mic_voice", 0)

            if voice_int == 1:
                rospy.set_param("mic_voice", 0)  # 设置参数服务器值归�?
                play_music(1)  # 播放音乐
                time.sleep(2)
                print"voice shutdown"
                voice_launch.shutdown()
                time.sleep(3)
                # ==============
                # 打开图像识别
                camera_launch = set_launch(camera_launch_path)
                camera_launch.start()
                camera_open_flag=1
                # ==============
            if  color_int==1 and camera_open_flag==1:
                camera_open_flag = 0
                #rospy.set_param("color_flag_sjq", 0)  # 设置参数服务器值归�?
                play_music(2)  # 播放音乐
                time.sleep(1)
                camera_launch.shutdown()   #================camera开启关闭
                pre_F1_demo_launch = set_launch(pre_F1_demo_path)  # 打开F1_demo
                pre_F1_demo_launch.start()
                print("ok---pre")
                time.sleep(2)
                F1_demo_launch = set_launch(F1_demo_path)   #打开F1_demo
                F1_demo_launch.start()
                

            # if color_int==1:
            #     rospy.set_param("color_flag", 0)  # 设置参数服务器值归�?            #     play_music(2)  #播放音乐
            #     time.sleep(1)
            #     camera_launch.shutdown()
            #     time.sleep(5)
            #     #==============
            #     #打开语音
            #     voice_launch = set_launch(voice_launch_path)
            #     voice_launch.start()
            #     voice_open_flag=1
            #     #==============
            # if voice_int==1 and voice_open_flag==1:
            #     rospy.set_param("mic_voice_sjq", 0)  # 设置参数服务器值归�?            #     play_music(1)  # 播放音乐
            #     voice_open_flag = 0
            #     time.sleep(2)
            #     voice_launch.shutdown()

             #camera_launch.shutdown()
        # time.sleep(2)
        # #=================
        # uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        # roslaunch.configure_logging(uuid)
        # tracking_launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/ncut/robot_ws/src/sjq/launch/open.launch"])
        # tracking_launch.start()
        # #=================
    rospy.spin()
    
if __name__ == '__main__':
    # 创建节点
    try:
        talker()
    except KeyboardInterrupt: 
        print("close successfully")
        
        

