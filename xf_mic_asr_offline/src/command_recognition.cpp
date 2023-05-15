/**************************************************************************
作者：caidx1
功能：命令控制器，命令词识别结果转化为对应的执行动作
**************************************************************************/
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>  //ROS 中表示机器人速度的消息类型
#include <std_msgs/String.h>
#include <iostream>
#include <stdio.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>   //ROS 中表示机器人位置的消息类型。
#include <cstdlib>
#include <ctime>


using namespace std;
ros::Publisher follow_flag_pub;    //创建寻找声源标志位话题发布者
ros::Publisher awake_flag_pub;    //创建唤醒标志位话题发布者
geometry_msgs::Twist cmd_msg;    //底盘运动话题消息数据
geometry_msgs::PoseStamped target;    //导航目标点消息数据
ros::Publisher cmd_send_pub;
ros::Publisher pick_up_num_pub;
ros::Publisher dispense_window_pub;


float I_position_x ;
float I_position_y ;
float I_orientation_z ;
float I_orientation_w ;

float J_position_x ;
float J_position_y ;
float J_orientation_z ;
float J_orientation_w ;

float K_position_x ;
float K_position_y ;
float K_orientation_z ;
float K_orientation_w ;

float line_vel_x ;
float ang_vel_z ;
float turn_line_vel_x ;

//毫秒延时
void Delay(int time)
{
	clock_t now = clock();
	while(clock()-now < time);
}

//接收小车指令值
void arrive_Callback(const std_msgs::Float64& msg)
{
	float flag = msg.data;
	cout<<"[cmd_rec] 小车发来指令"<<endl;	
	if(flag == 1.0)
	{
		cout<<"[cmd_rec] 小车到达终点"<<endl;	
	    //system("play ~/Downloads/reach_goal.mp3 &");   //播放音频 使用play播放名为reach_goal.mp3的文件
		cmd_msg.linear.x = 0.0;
		cmd_msg.angular.z = 3.0;
		int tt = 500000;
		while(tt--);
			//vel_pub.publish(cmd_msg);  //底盘运动话题发布
	}
}

/**************************************************************************
函数功能：离线命令词识别结果sub回调函数
入口参数：命令词字符串  voice_control.cpp等
返回  值：无
**************************************************************************/
void voice_words_callback(const std_msgs::String& msg)
{
	/***指令***/
	string str1 = msg.data.c_str();    //取传入数据
	string str2 = "小车去A1点";
	string str3 = "小车去A2点";
	string str4 = "小车去A3点";
	string str5 = "小车去A4点";
	string str6 = "小车去B1点";
	string str7 = "小车去B2点";
	string str8 = "小车去B3点";
	string str9 = "小车去B4点";
	string str10 = "小车去C1点";
	string str11 = "小车去C2点";
	string str12 = "小车去C3点";
	string str13 = "小车去C4点";
	string str14 = "失败5次";
	string str15 = "失败10次";
	string str16 = "遇到障碍物";
	string str17 = "小车唤醒";
	cout<<str1<<endl;

/***********************************
指令：小车去A1点
动作：小车去A1点
***********************************/
	if(str1 == str2)
	{
         std_msgs::Int32 cmd_send1_msg;
         std_msgs::Int32 dispense_window1_msg;
         std_msgs::Int32 pick_up_num1_msg;
	     cmd_send1_msg.data = 1;
         dispense_window1_msg.data = 1;
         pick_up_num1_msg.data = 6;
         dispense_window_pub.publish(dispense_window1_msg);
         pick_up_num_pub.publish(pick_up_num1_msg);
	     cmd_send_pub.publish(cmd_send1_msg);
	     cout<<"[cmd_rec] 好的：小车从A点到1点"<<endl;
	}
/***********************************
指令：小车去A2点
动作：小车去A2点
***********************************/
	else if(str1 == str3)
	{
         std_msgs::Int32 cmd_send2_msg;
         std_msgs::Int32 dispense_window2_msg;
         std_msgs::Int32 pick_up_num2_msg;
	     cmd_send2_msg.data = 1;
         dispense_window2_msg.data = 1;
         pick_up_num2_msg.data = 5;
         dispense_window_pub.publish(dispense_window2_msg);
         pick_up_num_pub.publish(pick_up_num2_msg);
	     cmd_send_pub.publish(cmd_send2_msg);
	     cout<<"[cmd_rec] 好的：小车从A点到1点"<<endl;
	}
/***********************************
指令：小车去A3点
动作：小车去A3点
***********************************/
	else if(str1 == str4)
	{
         std_msgs::Int32 cmd_send3_msg;
         std_msgs::Int32 dispense_window3_msg;
         std_msgs::Int32 pick_up_num3_msg;
	     cmd_send3_msg.data = 1;
         dispense_window3_msg.data = 1;
         pick_up_num3_msg.data = 4;
         dispense_window_pub.publish(dispense_window3_msg);
         pick_up_num_pub.publish(pick_up_num3_msg);
	     cmd_send_pub.publish(cmd_send3_msg);
	     cout<<"[cmd_rec] 好的：小车从A点到1点"<<endl;
	}
/***********************************
指令：小车去A4点
动作：小车去A4点
***********************************/
	else if(str1 == str5)
	{
         std_msgs::Int32 cmd_send4_msg;
         std_msgs::Int32 dispense_window4_msg;
         std_msgs::Int32 pick_up_num4_msg;
	     cmd_send4_msg.data = 1;
         dispense_window4_msg.data = 1;
         pick_up_num4_msg.data = 3;
         dispense_window_pub.publish(dispense_window4_msg);
         pick_up_num_pub.publish(pick_up_num4_msg);
	     cmd_send_pub.publish(cmd_send4_msg);
	     cout<<"[cmd_rec] 好的：小车从A点到1点"<<endl;
	}
/***********************************
指令：小车去B1点
动作：小车去B1点
***********************************/
	else if(str1 == str6)
	{
         std_msgs::Int32 cmd_send5_msg;
         std_msgs::Int32 dispense_window5_msg;
         std_msgs::Int32 pick_up_num5_msg;
	     cmd_send5_msg.data = 1;
         dispense_window5_msg.data = 2;
         pick_up_num5_msg.data = 6;
         dispense_window_pub.publish(dispense_window5_msg);
         pick_up_num_pub.publish(pick_up_num5_msg);
	     cmd_send_pub.publish(cmd_send5_msg);
	     cout<<"[cmd_rec] 好的：小车从A点到1点"<<endl;
	}
/***********************************
指令：小车去B2点
动作：小车去B2点
***********************************/
	else if(str1 == str7)
	{
         std_msgs::Int32 cmd_send6_msg;
         std_msgs::Int32 dispense_window6_msg;
         std_msgs::Int32 pick_up_num6_msg;
	     cmd_send6_msg.data = 1;
         dispense_window6_msg.data = 2;
         pick_up_num6_msg.data = 5;
         dispense_window_pub.publish(dispense_window6_msg);
         pick_up_num_pub.publish(pick_up_num6_msg);
	     cmd_send_pub.publish(cmd_send6_msg);
	     cout<<"[cmd_rec] 好的：小车从A点到1点"<<endl;
	}
/***********************************
指令：小车去B3点
动作：小车去B3点
***********************************/
	else if(str1 == str8)
	{
         std_msgs::Int32 cmd_send7_msg;
         std_msgs::Int32 dispense_window7_msg;
         std_msgs::Int32 pick_up_num7_msg;
	     cmd_send7_msg.data = 1;
         dispense_window7_msg.data = 2;
         pick_up_num7_msg.data = 4;
         dispense_window_pub.publish(dispense_window7_msg);
         pick_up_num_pub.publish(pick_up_num7_msg);
	     cmd_send_pub.publish(cmd_send7_msg);
	     cout<<"[cmd_rec] 好的：小车从A点到1点"<<endl;
	}
/***********************************
指令：小车去B4点
动作：小车去B4点
***********************************/
	else if(str1 == str9)
	{
         std_msgs::Int32 cmd_send8_msg;
         std_msgs::Int32 dispense_window8_msg;
         std_msgs::Int32 pick_up_num8_msg;
	     cmd_send8_msg.data = 1;
         dispense_window8_msg.data = 2;
         pick_up_num8_msg.data = 3;
         dispense_window_pub.publish(dispense_window8_msg);
         pick_up_num_pub.publish(pick_up_num8_msg);
	     cmd_send_pub.publish(cmd_send8_msg);
	     cout<<"[cmd_rec] 好的：小车从A点到1点"<<endl;
	}
/***********************************
指令：小车去C1点
动作：小车去C1点
***********************************/
	else if(str1 == str10)
	{
        std_msgs::Int32 cmd_send9_msg;
        std_msgs::Int32 dispense_window9_msg;
        std_msgs::Int32 pick_up_num9_msg;
        cmd_send9_msg.data = 1;
        dispense_window9_msg.data = 0;
        pick_up_num9_msg.data = 6;
         dispense_window_pub.publish(dispense_window9_msg);
         pick_up_num_pub.publish(pick_up_num9_msg);
	     cmd_send_pub.publish(cmd_send9_msg);
	}
/***********************************
指令：小车去C2点
动作：小车去C2点
***********************************/
	else if(str1 == str11)
	{
         std_msgs::Int32 cmd_send10_msg;
         std_msgs::Int32 dispense_window10_msg;
         std_msgs::Int32 pick_up_num10_msg;
	     cmd_send10_msg.data = 1;
         dispense_window10_msg.data = 0;
         pick_up_num10_msg.data = 5;
         dispense_window_pub.publish(dispense_window10_msg);
         pick_up_num_pub.publish(pick_up_num10_msg);
	     cmd_send_pub.publish(cmd_send10_msg);
	     cout<<"[cmd_rec] 好的：小车从A点到1点"<<endl;
	}
/***********************************
指令：小车去C3点
动作：小车去C3点
***********************************/
	else if(str1 == str12)
	{
         std_msgs::Int32 cmd_send11_msg;
         std_msgs::Int32 dispense_window11_msg;
         std_msgs::Int32 pick_up_num11_msg;
	     cmd_send11_msg.data = 1;
         dispense_window11_msg.data = 0;
         pick_up_num11_msg.data = 4;
         dispense_window_pub.publish(dispense_window11_msg);
         pick_up_num_pub.publish(pick_up_num11_msg);
	     cmd_send_pub.publish(cmd_send11_msg);
	     cout<<"[cmd_rec] 好的：小车从A点到1点"<<endl;
	}
/***********************************
指令：小车去C4点
动作：小车去C4点
***********************************/
	else if(str1 == str13)
	{
         std_msgs::Int32 cmd_send12_msg;
         std_msgs::Int32 dispense_window12_msg;
         std_msgs::Int32 pick_up_num12_msg;
	     cmd_send12_msg.data = 1;
         dispense_window12_msg.data = 0;
         pick_up_num12_msg.data = 3;
         dispense_window_pub.publish(dispense_window12_msg);
         pick_up_num_pub.publish(pick_up_num12_msg);
	     cmd_send_pub.publish(cmd_send12_msg);
	     cout<<"[cmd_rec] 好的：小车从A点到1点"<<endl;
	}
/***********************************
辅助指令：失败5次
动作：用户界面打印提醒
***********************************/
	else if(str1 == str14)
	{
		cout<<"[cmd_rec] 您已经连续【输入空指令or识别失败】5次，累计达15次自动进入休眠，输入有效指令后计数清零"<<endl;
	}
/***********************************
辅助指令：失败10次
动作：用户界面打印提醒
***********************************/
	else if(str1 == str15)
	{
		cout<<"[cmd_rec] 您已经连续【输入空指令or识别失败】10次，累计达15次自动进入休眠，输入有效指令后计数清零"<<endl;
	}
/***********************************
辅助指令：遇到障碍物
动作：用户界面打印提醒
***********************************/
	else if(str1 == str16)
	{
		cout<<"[cmd_rec] 小车遇到障碍物，已停止运动"<<endl;
	}
/***********************************
辅助指令：小车唤醒
动作：用户界面打印提醒
***********************************/
	else if(str1 == str17)
	{	
		cout<<"[cmd_rec] 小车已被唤醒，请说语音指令"<<endl;
		//system("play ~/Downloads/reply_ask.mp3 &");
		cmd_msg.linear.x = 0.1;
		cmd_msg.angular.z = 5.0;	
		//vel_pub.publish(cmd_msg);
		ros::Duration(0.2).sleep();
		cmd_msg.linear.x = 0.1;
		cmd_msg.angular.z = -5.0;
		//vel_pub.publish(cmd_msg);
		ros::Duration(0.2).sleep();
		// Delay(1000);
		cmd_msg.linear.x = 0.0;
		cmd_msg.angular.z = 0.0;
		//vel_pub.publish(cmd_msg);
	}
}

/**************************************************************************
函数功能：主函数
入口参数：无
返回  值：无
**************************************************************************/
int main(int argc, char** argv)
{

	ros::init(argc, argv, "cmd_rec");     //初始化ROS节点

	ros::NodeHandle n;    //创建句柄
	
	string if_akm;

	/***********************************/
	cmd_send_pub = n.advertise<std_msgs::Int32>("/cmd_send", 10);

      pick_up_num_pub = n.advertise<std_msgs::Int32>("/pick_up_num", 10);

      dispense_window_pub = n.advertise<std_msgs::Int32>("/dispense_window", 10);
	/***********************************/


	/***创建寻找声源标志位话题发布者***/
	follow_flag_pub = n.advertise<std_msgs::Int8>("follow_flag",1);

	/***创建唤醒标志位话题发布者***/
	awake_flag_pub = n.advertise<std_msgs::Int8>("awake_flag", 1);

	/***创建离线命令词识别结果话题订阅者***/
	ros::Subscriber voice_words_sub = n.subscribe("voice_words",10,voice_words_callback);

	//订阅器car_reach_goal订阅话题名称为arrive_instruction的队列为10的消息，同时运行回调函数arrive_Callback
	ros::Subscriber car_reach_goal = n.subscribe("/arrive_instruction", 10, arrive_Callback);



    //初始位置参数
	n.param<float>("/I_position_x", I_position_x, 1);
	n.param<float>("/I_position_y", I_position_y, 0);
	n.param<float>("/I_orientation_z", I_orientation_z, 0);
	n.param<float>("/I_orientation_w", I_orientation_w, 1);

	n.param<float>("/J_position_x", J_position_x, 2);
	n.param<float>("/J_position_y", J_position_y, 0);
	n.param<float>("/J_orientation_z", J_orientation_z, 0);
	n.param<float>("/J_orientation_w", J_orientation_w, 1);

	n.param<float>("/K_position_x", K_position_x, 3);
	n.param<float>("/K_position_y", K_position_y, 0);
	n.param<float>("/K_orientation_z", K_orientation_z, 0);
	n.param<float>("/K_orientation_w", K_orientation_w, 1);

	n.param<float>("/line_vel_x", line_vel_x, 1.0);   //获取名为“/line_vel_x”的参数，并将其赋值给变量“line_vel_x”，默认1.0
	n.param<float>("/ang_vel_z", ang_vel_z, 1.0);     //ang_vel_z默认值为1.0
	n.param("/if_akm_yes_or_no", if_akm, std::string("no"));  //if_akm默认值为no


	if(if_akm == "yes")
		turn_line_vel_x = line_vel_x;
	else 
		turn_line_vel_x = 0;

	/***自主导航目标点数据初始化***/
	target.header.seq = 0;
	//target.header.stamp;
	target.header.frame_id = "map";
	target.pose.position.x = 0;
	target.pose.position.y = 0;
	target.pose.position.z = 0;
	target.pose.orientation.x = 0;
	target.pose.orientation.y = 0;
	target.pose.orientation.z = 0;
	target.pose.orientation.w = 1;

  /***用户界面***/
	cout<<"[cmd_rec] 您可以语音控制啦！唤醒词“小易同学”"<<endl;
	cout<<"[cmd_rec] 小车去A1点"<<endl;
	cout<<"[cmd_rec] 小车去A2点"<<endl;
	cout<<"[cmd_rec] 小车去A3点"<<endl;
	cout<<"[cmd_rec] 小车去A4点"<<endl;
	cout<<"[cmd_rec] 小车去B1点"<<endl;
	cout<<"[cmd_rec] 小车去B2点"<<endl;
	cout<<"[cmd_rec] 小车去B3点"<<endl;
	cout<<"[cmd_rec] 小车去B4点"<<endl;
	cout<<"[cmd_rec] 小车去C1点"<<endl;
	cout<<"[cmd_rec] 小车去C2点"<<endl;
	cout<<"\n"<<endl;

	ros::spin();
}

