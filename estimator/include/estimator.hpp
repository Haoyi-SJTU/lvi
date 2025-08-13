#pragma once

#include <iostream>
#include <cmath>
#include <vector>
// ROS
#include <ros/ros.h>
// 点云处理
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>
// 发布结果消息
#include <geometry_msgs/Vector3Stamped.h>
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/TransformStamped.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/PointCloud.h>

#include <eigen3/Eigen/Dense>
#include <deque>

// 优化器
#include "gurobi_c++.h"
// 获取活跃特征点
#include <unordered_map>
#include <algorithm>

// #define DEBUG_estimator // 优化器调试

#define EIGEN_USE_MKL_ALL //开启mkl优化：在使用Intel处理器并且已经安装好mkl库的情况下，可通过宏开启Eigen3的mkl优化
#define EIGEN_NO_DEBUG //关闭Debug模式：在确保程序无误的情况下，关闭Debug断言，可以提升程序效率

const unsigned int WINDOW_SIZE = 10;   // 滑动窗长度
const float MIN_delta_depth_1 = 200;   // 深度预积分阈值
const float MIN_delta_depth_2 = 200;   // 深度预积分阈值
const float MIN_Ps_for_1_image = 0.02; // 能让图像成为关键帧的最小IMU积分阈值 0.02 m
const double Max_Ps = 0.1;			   // 一个窗内的最大位移 单位m
const double Max_Delta_Ps = 0.01;	   // 相邻两个窗的最大位移差值 单位m
const double Max_Vs = 3;			   // 每帧的最大速度 单位m/s
const double Max_Delta_Vs = 1;		   // 相邻两帧的最大速度差值 单位m/s
const double MIN_Active_Feature = 4;   // 活跃特征点个数的最小阈值，小于4则不进行重投影误差计算
const double DT_IMG = 0.050;		   // 相邻两帧的时间差，根据图像消息发布频率7Hz估计
const double TOLERANCE = 0.020;		   // 优化器位移-速度约束的上下公差
const double P_WEIGHT = 1;			   // 优化器 IMU-位移误差项 权重
const double V_WEIGHT = 1.5;		   // 优化器 IMU-速度误差项 权重
const double Q_WEIGHT = 200;		   // 优化器 IMU-转角四元数误差项 权重
const double SCALE_WEIGHT = 0.01;	   // 优化器 相机-尺度因子误差项 权重
const double FEATURE_WEIGHT = 100;	   // 优化器 相机-特征点误差项 权重
const double OPTICAL_WEIGHT = 0.05;	   // 优化器 相机-光流速度误差项 权重
const float P_VIOresult_WEIGHT = 1;	   // TF发布的P中，来自VIO优化结果的权重

const double average_f_1 = 614.7479685;
const double average_f_2 = 617.000043;

int pub_ideal_counter = 0; // 测试用

class VIO_estimator
{
private:
	pthread_mutex_t mutex;
	ros::Publisher pointcloud_pub;			   // 结果点云发布
	tf2_ros::TransformBroadcaster broadcaster; // 结果TF 发布
	ros::NodeHandle nh_imu;					   // IMU消息相应句柄
	ros::NodeHandle nh_gyc;					   // IMU消息相应句柄
	ros::NodeHandle nh_img;					   // 图像消息响应句柄
	ros::NodeHandle nh_tag;					   // tag消息、tag中心坐标、激光点达到预积分阈值响应句柄
	unsigned int img_count;					   // 最新图像在滑动窗里的编号
	bool imu_init;							   // 首次记录IMU
	bool gyr_init;
	bool WINDOW_FULL_FLAG;						   // 滑动窗是否满了
	bool ESTIMATOR_FLAG;						   // 0:初始化阶段; 1:优化阶段
	bool ESTIMATOR_PUB;							   // 正在发布优化器/tag数据
	bool PUB_VIO_FLAG;							   // 产生数据，可以发布
	bool GOOD_VIO_FLAG;							   // 好数据，可以加在发布结果里
	long int imu_t0;							   // IMU时间戳初始化
	double dt;									   // acc时间间隔
	std::pair<double, Eigen::Vector3d> acc_0;	   // IMU 上一次 时间戳+加速度
	std::pair<double, Eigen::Quaterniond> gyr_0;   // gyr 上一帧图像 时间戳+转角
	std::pair<double, Eigen::Vector3d> acc_now;	   // IMU 这一次 时间戳+加速度
	std::pair<double, Eigen::Quaterniond> gyr_now; // gyr 取当前加速度的同时保存下来当前的 转角 (是转角不是角速度)
	std::pair<double, Eigen::Quaterniond> gyr;	   // 时刻更新的 时间戳+转角
	float tag_center_u, tag_center_v;			   // tag中心点的图像坐标
	float delta_x_world;						   // 机器人基坐标系下的运动增量
	float delta_y_world;
	float delta_z_world;

	Eigen::Quaterniond initial_Q; // 初始位置的四元数
	Eigen::Vector3d Ps_now;		  // IMU预积分 当前三方向位置
	Eigen::Vector3d Vs_now;		  // IMU预积分 当前三方向速度
	Eigen::Matrix3d Rs_now;		  // IMU预积分 当前三方向转角
	Eigen::Quaterniond Qs_now;
	Eigen::Vector3f ideal_trace;	  // 理想轨迹的坐标(base_link)
	Eigen::Quaterniond ideal_quatern; // 理想轨迹的转角(base_link)

	// tag产生的数据 相对于tag的坐标
	Eigen::Matrix4d T_imu_cam;	// cam1相对于imu坐标系的变换矩阵，单位m 外参   注意此参数来自于标定！！！！！！！！！！！！！！
	Eigen::Matrix3d R_imu2base; // 机器人基座到IMU的旋转阵
	Eigen::Vector3d P_now;		// 当前位置优化解

	Eigen::Vector3d P_tag;	  // 当前位置（相对于tag）
	Eigen::Quaterniond Q_tag; // 当前转角四元数（相对于tag）
	Eigen::Matrix3d R_tag;	  // 当前转角矩阵（相对于tag）
	Eigen::Matrix4d T_tag;	  // 当前齐次变换矩阵（相对于tag）

	std::deque<std::map<int, Eigen::Matrix<double, 8, 1>>> img_queue; // 滑动窗 图像特征点
	std::deque<Eigen::Vector3d> Ps_queue;							  // 滑动窗 三方向位置
	std::deque<Eigen::Vector3d> Vs_queue;							  // 滑动窗 三方向速度
	std::deque<Eigen::Quaterniond> Qs_queue;						  // 滑动窗 转角四元数
	std::deque<Eigen::Vector3d> Bas_queue;
	std::deque<Eigen::Vector3d> Bgs_queue;

	std::vector<int> Active_feature_id; // 动态更新在整个滑动窗内都活跃的特征点ID
	float scale_factor_1;				// 最新尺度因子初值 cam1
	float scale_factor_2;				// 最新尺度因子初值 cam2
	unsigned int static_thread;			// 用来判定图像是否静止

	// 优化器函数
	void add_Variables(GRBModel &, std::vector<GRBVar> &, std::vector<GRBVar> &, std::vector<GRBVar> &, std::vector<GRBVar> &);					  // 添加优化变量
	void add_Constraints(GRBModel &, std::vector<GRBVar> &, std::vector<GRBVar> &, std::vector<GRBVar> &, std::vector<GRBVar> &);				  // 添加约束条件
	bool initial_optimization_variables(std::vector<GRBVar> &, std::vector<GRBVar> &, std::vector<GRBVar> &, std::vector<GRBVar> &);			  // 为优化变量设置初值
	bool calculate_reprojection_error(GRBQuadExpr &, std::vector<GRBVar> &, std::vector<GRBVar> &, std::vector<GRBVar> &, std::vector<GRBVar> &); // cam残差: 将重投影误差加入目标函数
	bool calculate_preintegrate_error(GRBQuadExpr &, std::vector<GRBVar> &, std::vector<GRBVar> &, std::vector<GRBVar> &);						  // 预积分误差: 最小化位移、速度、转角的估计值与IMU预积分之间的差值
	bool calculate_scale_factor_error(GRBQuadExpr &, std::vector<GRBVar> &);																	  // 优化器目标函数: 尺度因子误差
	inline GRBQuadExpr robust_kernel(const double, GRBVar &, GRBVar &);																			  // 鲁棒核函数 用于加权cam残差
	bool resize_variables(GRBModel &, std::vector<GRBVar> &, std::vector<GRBVar> &, std::vector<GRBVar> &, std::vector<GRBVar> &);				  // 优化失败后收紧变量范围再次优化

	inline void pre_integrate(); // 预积分
	// bool pointcloud_initial(const std::map<int, Eigen::Matrix<double, 8, 1>> &, double, double, double, double);  // 初始化3D点云

	bool filterImage(const std::map<int, Eigen::Matrix<double, 8, 1>> &, double, double, double, double, std::vector<int> &);
	bool add_keyframe(std::map<int, Eigen::Matrix<double, 8, 1>> &); // 向滑动窗添加关键帧
	bool find_Active_feature_id();									 // 获取滑动窗内所有特征点编号的交集，作为在整个滑动窗都活跃的特征点

	void acc_callback(const geometry_msgs::Vector3Stamped::ConstPtr &msg);
	void gyr_callback(const geometry_msgs::QuaternionStamped::ConstPtr &msg);
	void feature_callback_cam1(const sensor_msgs::PointCloudConstPtr &feature_msg);			// 相机1  图像特征点 回调
	void apriltag_callback_cam1(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg); // 相机1 图像apriltag坐标 回调
	void tag_center_callback(const geometry_msgs::PointStamped::ConstPtr &msg);				// 接收tag中心的图像坐标
	void ideal_trace_callack(const geometry_msgs::PoseStamped::ConstPtr &);					// 更新最新ideal trace

	void laser_callback(const geometry_msgs::PointStamped::ConstPtr &msg);	   // laser 深度差值 回调
	inline void local_to_global(std::vector<GRBVar> &, std::vector<GRBVar> &); // 将优化器结果的local位移累加到global的相对于tag系下的位姿上
	inline void publish_vio_result();										   // 发布VIO结果
	inline void pub_pointcloud(std::vector<GRBVar> &);						   // 发布优化后的点云

	// inline void publish_imu_result();	// 发布imu结果

public:
	VIO_estimator();
	~VIO_estimator();
};
