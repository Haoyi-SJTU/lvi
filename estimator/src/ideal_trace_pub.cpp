#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>

struct Point3D
{
     float x;
     float y;
     float z;
     float qw;
     float qx;
     float qy;
     float qz;
};

int main(int argc, char **argv)
{
     ros::init(argc, argv, "ideal_trace_node");
     ros::NodeHandle nh;
     ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("ideal_trace", 1);

     std::ifstream file("/home/yuanzhi/catkin_ws/src/estimator/data/ideal_trace_withQ(5hz).txt");
     std::vector<Point3D> points;
     if (file.is_open())
     {
          std::string line;
          while (std::getline(file, line))
          {
               std::stringstream ss(line);
               std::string token;
               Point3D point;
               // 使用制表符分隔每行的坐标xyz分量
               std::getline(ss, token, '\t');
               point.x = std::stof(token);
               std::getline(ss, token, '\t');
               point.y = std::stof(token);
               std::getline(ss, token, '\t');
               point.z = std::stof(token);
               std::getline(ss, token, '\t');
               point.qw = std::stof(token);
               std::getline(ss, token, '\t');
               point.qx = std::stof(token);
               std::getline(ss, token, '\t');
               point.qy = std::stof(token);
               std::getline(ss, token, '\t');
               point.qz = std::stof(token);
               points.push_back(point);
          }
          file.close();
     }

     ros::Rate rate(12.5); // 发布频率
     geometry_msgs::PoseStamped pose_msg;
     pose_msg.header.frame_id = "base_link"; // 设置坐标系为base_link

     for (const auto &point : points)
     {
          pose_msg.header.stamp = ros::Time::now();
          pose_msg.pose.position.x = point.x;
          pose_msg.pose.position.y = point.y;
          pose_msg.pose.position.z = point.z;
          pose_msg.pose.orientation.w = point.qw;
          pose_msg.pose.orientation.x = point.qx;
          pose_msg.pose.orientation.y = point.qy;
          pose_msg.pose.orientation.z = point.qz;
          //发布静止的位置
          // pose_msg.pose.position.x = 0;
          // pose_msg.pose.position.y = 0;
          // pose_msg.pose.position.z = 0;
          // pose_msg.pose.orientation.w = 1;
          // pose_msg.pose.orientation.x = 0;
          // pose_msg.pose.orientation.y = 0;
          // pose_msg.pose.orientation.z = 0;

          pose_pub.publish(pose_msg);
          ros::spinOnce();
          rate.sleep();
     }

     return 0;
}
