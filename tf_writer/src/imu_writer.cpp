#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <fstream>
#include <iostream>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "tf_writer");
  ros::NodeHandle nh;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  // 打开文件准备写入
  std::ofstream outputFile("tf_data.txt");

  ros::Rate rate(25.0);
  while (ros::ok())
  {
    geometry_msgs::TransformStamped transformStamped;
    try
    {
      transformStamped = tfBuffer.lookupTransform("world", "sensor_link", ros::Time(0));
      std::cout << transformStamped.transform.translation.x << " "
                << transformStamped.transform.translation.y << " " << transformStamped.transform.translation.z << "\n";
      outputFile << transformStamped.transform.translation.x << " "
                 << transformStamped.transform.translation.y << " "
                 << transformStamped.transform.translation.z << "\n";
      // outputFile << transformStamped.transform.rotation.w << transformStamped.transform.rotation.x << " " << transformStamped.transform.rotation.y << " " << transformStamped.transform.rotation.z << "\n";
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("%s", ex.what());
    }
    rate.sleep();
  }

  // 关闭文件
  outputFile.close();

  return 0;
}
