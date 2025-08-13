// C++
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <iostream>
#include <vector>
// 多线程
#include <thread>
#include <mutex>
// ROS 独立回调队列
#include "ros/ros.h"
// 消息同步器
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <sensor_msgs/Imu.h>
// 类声明
#include "translate_xsens_msg.hpp"

using namespace message_filters;
using namespace std;
using namespace Eigen;

// 中继节点，把Xsens的数据(geometry_msg类型)合并为imu类型(sensor_msgs::Imu)的数据
// 用于标定

void VIO_estimator::acc_callback(const geometry_msgs::Vector3Stamped::ConstPtr &msg) //
{
  if (!imu_init)
  {
    imu_init = 1;
    acc_0.first = (float)(msg->header.stamp.sec) + (float)msg->header.stamp.nsec / 1000000000;
    acc_0.second << msg->vector.x, msg->vector.y, msg->vector.z;
    gyr_now.first = gyr.first;
    gyr_now.second = gyr.second;
  }
  else
  {
    acc_now.first = (float)(msg->header.stamp.sec) + (float)msg->header.stamp.nsec / 1000000000;
    acc_now.second << msg->vector.x, msg->vector.y, msg->vector.z;
    gyr_now.first = gyr.first;
    gyr_now.second = gyr.second; // 陀螺仪数据跟随加速度计数据更新
    pub_imudata();
  }
}

void VIO_estimator::gyr_callback(const geometry_msgs::Vector3Stamped::ConstPtr &msg) //
{
  if (!imu_init)
  {
    imu_init = 1;
    imu_t0 = (long int)msg->header.stamp.sec;
    gyr.first = (float)(msg->header.stamp.sec) + (float)msg->header.stamp.nsec / 1000000000; // 似乎不需要时间戳
    gyr.second << msg->vector.x, msg->vector.y, msg->vector.z;
  }
  else
  {
    gyr.first = (float)(msg->header.stamp.sec) + (float)msg->header.stamp.nsec / 1000000000; // 似乎不需要时间戳
    gyr.second << msg->vector.x, msg->vector.y, msg->vector.z;
  }
}

void VIO_estimator::pub_imudata()
{
  // cout << "start" << endl;
  sensor_msgs::Imu imu_data;
  imu_data.header.stamp = ros::Time::now();
  imu_data.header.frame_id = "sensor";
  imu_data.angular_velocity.x = gyr_now.second[0];
  imu_data.angular_velocity.y = gyr_now.second[1];
  imu_data.angular_velocity.z = gyr_now.second[2];
  imu_data.linear_acceleration.x = acc_now.second[0];
  imu_data.linear_acceleration.y = acc_now.second[1];
  imu_data.linear_acceleration.z = acc_now.second[2];

  IMU_pub.publish(imu_data);
}

VIO_estimator::VIO_estimator()
{
  imu_init = 0;

  ros::NodeHandle nh_imu;
  ros::Subscriber acc_listener = nh_imu.subscribe("/filter/free_acceleration", 1, &VIO_estimator::acc_callback, this); // IMU回调
  ros::Subscriber gyr_listener = nh_imu.subscribe("/imu/angular_velocity", 1, &VIO_estimator::gyr_callback, this);        // gyr回调
  IMU_pub = nh_imu.advertise<sensor_msgs::Imu>("IMU_data", 10);

  ros::MultiThreadedSpinner spinner(4);
  spinner.spin();
}

VIO_estimator::~VIO_estimator(void)
{
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "estimator");
  VIO_estimator estimator_handle; // argc, argv
  return 1;
}
