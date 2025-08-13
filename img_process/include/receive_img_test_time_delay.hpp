#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <vector>
#include <map>
#include <ros/ros.h>
#include "opencv2/opencv.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <image_transport/image_transport.h>
#include "laser_cam.hpp"
#include "laser_cam.cpp"

class img_process
{
private:
	double x0, x1, x2; // 测试参数
	double y0, y1, y2;
	double z0, z1, z2;
	double x_average; // 测试参数
	double y_average;
	double z_average;

	// 相机通用参数
	bool EQUALIZE;					// 是否对图像做局部自适应均衡化
	unsigned int MAX_CNT, MIN_DIST; // 特征点跟随中的特征点数量上限、区分两个相邻特征点的最小像素距离
	unsigned int ROW, COL;			// 图像尺寸

	laser_cam laser;	   // 激光点采集
	cv::Vec2f laser_depth; // 二维向量 存放两个相机根据激光点计算的最新深度

	unsigned int LASER_DELAY_TIME;	 // 从改变激光标志符到取得激光照片的时间差，ms
	unsigned int OPTICAL_DELAY_TIME; // 从改变激光标志符到暂停或开启光流的时间差，ms
	bool OPTICAL_DELAY_FLAG;

	void optical_delay();

	// 1号相机
	bool if_first_1;  // 判定是否是第一张照片
	bool PUB_FRAME_1; // 判定这一帧图像要不要使用
	unsigned int FOCAL_LENGTH_1;
	int pub_count_1;
	bool init_pub_1;
	cv::Mat K_1; // 相机内参矩阵
	cv::Mat R_1;
	cv::Mat t_1;
	// float fx_1, fy_1, cx_1, cy_1; // 相机内参
	float k1_1, k2_1, p1_1, p2_1; // 相机畸变参数

	cv::Mat mask_1;
	cv::Mat img_0_1, img_1_1, img_now_1;
	cv::Mat img_laser_1;										   // 专用存放拍摄激光点图片
	double img_time_0_1, img_time_1_1, img_time_now_1;			   // 图像时间 前帧、昨帧的时间戳
	std::vector<cv::Point2f> pts_0_1, pts_1_1, pts_now_1;		   // 前帧、昨帧、今帧图像特征点
	std::vector<cv::Point2f> un_pts_0_1, un_pts_1_1, un_pts_now_1; // 校正后前帧、昨帧、今帧图像特征点
	std::vector<int> track_cnt_1;								   // 与函数setMask()有关
	std::vector<int> ids_1;										   // 特征点ID
	std::vector<cv::Point2f> n_pts_1;
	std::map<int, cv::Point2f> un_pts_map_0_1, un_pts_map_1_1; // 存放前帧、昨帧的校正特征点ID ids
	std::vector<cv::Point2f> pts_velocity_1;				   // 特征点光流速度

	std::vector<float> depth_list_1; // 1号相机 深度值的队列
	float delta_depth_1;			 // 1号相机 一个预积分周期内深度差
	float MAX_delta_depth_1;		 // 1号相机 一个预积分周期内深度差阈值

	// 2号相机
	bool if_first_2;  // 判定是否是第一张照片
	bool PUB_FRAME_2; // 判定这一帧图像要不要使用
	unsigned int FOCAL_LENGTH_2;
	int pub_count_2;
	bool init_pub_2;
	cv::Mat K_2; // 相机内参矩阵
	cv::Mat R_2;
	cv::Mat t_2;
	// float fx_2, fy_2, cx_2, cy_2; // 相机内参
	float k1_2, k2_2, p1_2, p2_2; // 相机畸变参数

	cv::Mat mask_2;
	cv::Mat img_0_2, img_1_2, img_now_2;
	cv::Mat img_laser_2;										   // 专用存放拍摄激光点图片
	double img_time_0_2, img_time_1_2, img_time_now_2;			   // 图像时间 前帧、昨帧的时间戳
	std::vector<cv::Point2f> pts_0_2, pts_1_2, pts_now_2;		   // 前帧、昨帧、今帧图像特征点
	std::vector<cv::Point2f> un_pts_0_2, un_pts_1_2, un_pts_now_2; // 校正后前帧、昨帧、今帧图像特征点
	std::vector<int> track_cnt_2;								   // 与函数setMask()有关
	std::vector<int> ids_2;										   // 特征点ID
	std::vector<cv::Point2f> n_pts_2;
	std::map<int, cv::Point2f> un_pts_map_0_2, un_pts_map_1_2; // 存放前帧、昨帧的校正特征点ID ids
	std::vector<cv::Point2f> pts_velocity_2;				   // 特征点光流速度

	std::vector<float> depth_list_2; // 2号相机 深度值的队列
	float delta_depth_2;			 // 2号相机 一个预积分周期内深度差
	float MAX_delta_depth_2;		 // 2号相机 一个预积分周期内深度差阈值

	pthread_mutex_t mutex;
	ros::Publisher img_pub; // 发点云消息
	// ros::Publisher img_pub_test;
	ros::Timer img_timer; // 开灯拍激光的定时器

	bool laser_timer(const ros::TimerEvent &); // 定时开灯，拍照，计算深度

public:
	img_process();
	~img_process();
	void imageCallback_1(const sensor_msgs::ImageConstPtr &);
	void imageCallback_2(const sensor_msgs::ImageConstPtr &);
};
