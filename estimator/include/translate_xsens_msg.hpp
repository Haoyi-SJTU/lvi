#pragma once

#include <iostream>
#include <cmath>
#include <vector>
#include <ros/ros.h>

#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <sensor_msgs/Imu.h>
// #include <sensor_msgs/PointCloud.h>
// #include <Eigen/Dense>
#include <eigen3/Eigen/Dense>
#include <queue>
#include "parameter.h"

class VIO_estimator
{
private:
	pthread_mutex_t mutex;
	ros::Timer result_pub_timer;				// 定时器
	ros::Publisher IMU_pub;						// 结果发布
	bool imu_init;								// IMU时间戳是否初始化
	long int imu_t0;							// IMU时间戳初始化
	std::pair<double, Eigen::Vector3d> acc_0;	// IMU 上一次的时间戳 加速度
	std::pair<double, Eigen::Vector3d> gyr_0;	// gyr 上一帧图像的时间戳 角速度
	std::pair<double, Eigen::Vector3d> acc_now; // IMU 这一次的时间戳 加速度
	std::pair<double, Eigen::Vector3d> gyr_now; // gyr 取当前加速度的同时保存下来当前的 角速度
	std::pair<double, Eigen::Vector3d> gyr;		// 时刻更新的时间戳 转角

	void acc_callback(const geometry_msgs::Vector3Stamped::ConstPtr &msg);
	void gyr_callback(const geometry_msgs::Vector3Stamped::ConstPtr &msg);
	void pub_imudata();

public:
	VIO_estimator();
	~VIO_estimator();
};
