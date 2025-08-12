#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

int main(int argc, char** argv)
{
  // 初始化ROS节点
  ros::init(argc, argv, "object_motion_visualizer");

  // 创建ROS节点句柄
  ros::NodeHandle nh;

  // 创建一个发布器，发布visualization_msgs/Marker类型的消息
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("object_marker", 10);

  // 创建一个Marker消息对象
  visualization_msgs::Marker sensor_model;
  sensor_model.header.frame_id = "base_link";  // 坐标系的名称

  // 设置Marker的命名空间、ID和类型
  sensor_model.ns = "object";
  sensor_model.id = 0;
  sensor_model.type = visualization_msgs::Marker::MESH_RESOURCE;

  // 设置Marker的尺寸
  sensor_model.scale.x = 0.01;
  sensor_model.scale.y = 0.01;
  sensor_model.scale.z = 0.01;

  // 设置Marker的颜色
  sensor_model.color.r = 1.0;
  sensor_model.color.g = 0.0;
  sensor_model.color.b = 0.0;
  sensor_model.color.a = 1.0;

  // 设置Marker的位置和姿态信息
  sensor_model.pose.position.x = 1.0;
  sensor_model.pose.position.y = 0.5;
  sensor_model.pose.position.z = 0.2;
  sensor_model.pose.orientation.x = 0.0;
  sensor_model.pose.orientation.y = 0.0;
  sensor_model.pose.orientation.z = 0.0;
  sensor_model.pose.orientation.w = 1.0;

  // 设置Marker的网格模型资源路径
  sensor_model.mesh_resource = "package://display/urdf/sensor.stl";

  // 设置Marker的持续时间
  sensor_model.lifetime = ros::Duration();

  // 发布Marker消息，显示模型
  sensor_model.publish(marker);

  // 进入ROS循环处理
  ros::spin();

  return 0;
}
