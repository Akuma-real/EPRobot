
#ifndef	__TRACK_DETECTION_H__
#define __TRACK_DETECTION_H__


#include "ros/ros.h"
#include <opencv2/opencv.hpp>   
#include <opencv2/highgui/highgui.hpp>
#include <string.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <stdlib.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Twist.h>

using namespace std;  
using namespace cv;

#define PROSPECT_VALUE 	10		//0.6M
#define SLOPE_VALUE		0.60f
#define MIDDLE_VALUE 	140	

class Track_Detection
{
public:
	Track_Detection();
	~Track_Detection();

	void Open_Imread();
	void Image_Filter_Process();
	void Show_Image( String WindowsName);
	void ImageMiddleDetection();
	void HiBotMove_Control();
	void HiBotCmd_Vel(float vx, float vz);
	void imageCallback(const sensor_msgs::ImageConstPtr& msg);


private:
	int	GyraThreshold;		//160
	std_msgs::Int32	ImgHeight;
	std_msgs::Int32	ImgWidth;
	unsigned short MiddleArray[60][4];
	int img_mid_value;

	Mat SrcImageRead;
	Mat DestImageOut;
	Mat GyraImage;

	bool StartMove;
	double MixKP, MaxKP;
	double HiBotSpeed;

	VideoCapture capture;
	cv_bridge::CvImagePtr frame;

	ros::NodeHandle n;
	image_transport::Subscriber astra_Image_sub;
	image_transport::Publisher Image_pub;
	image_transport::Publisher RgbImage_pub;
	ros::Publisher cmd_pub;
};





#endif
