#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <numeric>
#include <vector>
#include <map>
#include <ros/ros.h>
#include "opencv2/opencv.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <image_transport/image_transport.h>
#include "apriltag_ros/AprilTagDetection.h"
#include "apriltag_ros/AprilTagDetectionArray.h"
// #include <opencv2/features2d/features2d.hpp>
// #include <opencv2/calib3d/calib3d.hpp>
// #include <Eigen/Core>

// 多线程
#include <thread>
#include <mutex>

#include "laser_cam.hpp"
#include "laser_cam.cpp"

#define EIGEN_USE_MKL_ALL //开启mkl优化：在使用Intel处理器并且已经安装好mkl库的情况下，可通过宏开启Eigen3的mkl优化
#define EIGEN_NO_DEBUG //关闭Debug模式：在确保程序无误的情况下，关闭Debug断言，可以提升程序效率

bool EQUALIZE = 1;						// 是否图像局部自适应均衡化
bool BRIGHTNESS = 0;					// 是否增加图像亮度
const int ROW = 480;					// 图像高
const int COL = 640;					// 宽
const int MAX_CNT = 50;					// 特征点跟随中的特征点数量上限
const int MIN_DIST = 36;				// 区分两个相邻特征点的最小像素距离
const int MIN_DIST_half = MIN_DIST / 2; // 区分两个相邻特征点的最小像素距离
const float VELOCITY_THRESHOD = 15;		// 特征点光流速度与均值的离群程度阈值

class img_process
{
private:
	// 相机通用参数
    double average_f_1, average_f_2; // 计算尺度因子时用两个相机的四个焦距计算平均焦距
	laser_cam laser;				 // 激光点采集对象
	cv::Vec2f laser_depth;			 // 二维向量 存放两个相机根据激光点计算的最新深度
	unsigned int LASER_DELAY_TIME;	 // 从改变激光标志符到取得激光照片的时间差，ms
	unsigned int OPTICAL_DELAY_TIME; // 从改变激光标志符到暂停或开启光流的时间差，ms
	bool OPTICAL_DELAY_FLAG;

	// 1号相机
	bool if_first_1;  // 判定是否是第一张照片
	bool PUB_FRAME_1; // 判定这一帧图像要不要使用
	unsigned int FOCAL_LENGTH_1;
	bool init_pub_1;
	cv::Mat K_1; // cam1内参矩阵
	cv::Mat R_1;
	cv::Mat t_1;
	float k1_1, k2_1, p1_1, p2_1; // cam1畸变参数
	cv::Mat distCoeffs_1;		  // cam1畸变参数
	cv::Mat mask_1;
	cv::Mat img_1_1, img_now_1;
	cv::Mat img_laser_1;									   // 专用存放拍摄激光点图片
	double img_time_1_1, img_time_now_1;					   // 图像时间 前帧、昨帧的时间戳
	std::vector<cv::Point2f> pts_1_1, pts_now_1;			   // 昨帧、今帧图像特征点
	std::vector<cv::Point2f> un_pts_now_1;					   // 校正后今帧图像特征点un_pts_1_1
	std::vector<int> ids_1;									   // 特征点ID
	std::vector<cv::Point2f> n_pts_1;						   // 暂存新生成的特征点
	std::map<int, cv::Point2f> un_pts_map_0_1, un_pts_map_1_1; // 存放前帧、昨帧的ID、校正坐标 用来计算光流速度
	std::vector<cv::Point2f> pts_velocity_1;				   // 特征点光流速度
	float depth_list_1;										   // 1号相机 激光深度值

	// 2号相机
	bool if_first_2;			 // 判定是否是第一张照片
	bool PUB_FRAME_2;			 // 判定这一帧图像要不要使用
	unsigned int FOCAL_LENGTH_2; // xy焦距平均值用于计算基础矩阵
	bool init_pub_2;			 // 拍完首张img后置1
	cv::Mat K_2;				 // cam2内参矩阵
	cv::Mat R_2;
	cv::Mat t_2;
	float k1_2, k2_2, p1_2, p2_2; // cam2畸变参数
	cv::Mat distCoeffs_2;		  // cam2畸变参数
	cv::Mat mask_2;
	cv::Mat img_1_2, img_now_2;
	cv::Mat img_laser_2;						 // 专用存放拍摄激光点图片
	double img_time_1_2, img_time_now_2;		 // 图像时间 昨帧】今帧的时间戳
	std::vector<cv::Point2f> pts_1_2, pts_now_2; // 昨帧、今帧图像特征点
	std::vector<cv::Point2f> un_pts_now_2;		 // 校正后昨帧、今帧图像特征点un_pts_1_2
	// std::vector<int> track_cnt_2;					   // 与函数setMask()有关
	std::vector<int> ids_2; // 特征点ID
	std::vector<cv::Point2f> n_pts_2;
	std::map<int, cv::Point2f> un_pts_map_0_2, un_pts_map_1_2; // 存放前帧、昨帧的校正特征点ID ids
	std::vector<cv::Point2f> pts_velocity_2;				   // 特征点光流速度
	float depth_list_2;										   // 2号相机 激光深度值

	float latest_tag_time; // 存放tag时间戳

	pthread_mutex_t mutex;
	ros::Publisher img_pub;			   // 发点云消息
	ros::Publisher laser_distance_pub; // 发布激光测距消息
	ros::Timer img_timer;			   // 开灯拍激光的定时器

	// 数学处理函数
	void reduceVector(std::vector<cv::Point2f> &v, std::vector<uchar> status);
	void reduceVector(std::vector<int> &v, std::vector<uchar> status);
	bool inBorder(const cv::Point2f &pt);

	void liftProjective(const Eigen::Vector2d &p, Eigen::Vector3d &P);				   // 把点从图像平面到投影空间
	void backprojectSymmetric(const Eigen::Vector2d &p_u, double &theta, double &phi); // 根据图像坐标计算theta和phi

	void readImage_1();
	void readImage_2();
	void rejectWithF(bool);
	void setMask(bool);
	void addPoints(bool);
	void undistortedPoints(bool);
	inline void change_brightness(Mat &);													// 调整图像亮度 对数曲线
	void apriltag_callback_cam1(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg); // 相机1 tag坐标 回调
	void velocity_outlier_remove(bool);														// 移除光流速度离群的点
	bool laser_timer(const ros::TimerEvent &);												// 定时开灯，拍照，计算深度
	void optical_delay();																	// 延时函数：laser关闭后延迟开启光流
	inline cv::Point2f calculateMean(const std::vector<cv::Point2f> &);					// 计算Point2f容器的均值
public:
	img_process();
	~img_process();
	void imageCallback_1(const sensor_msgs::ImageConstPtr &);
	void imageCallback_2(const sensor_msgs::ImageConstPtr &);
};
