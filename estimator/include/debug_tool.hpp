#include "estimator.hpp"



// 测试用 发布点云消息
void publishPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloud, ros::NodeHandle &nh, ros::Publisher &pub)
{
  sensor_msgs::PointCloud2 cloudMsg;
  pcl::toROSMsg(*pointCloud, cloudMsg);
  cloudMsg.header.stamp = ros::Time::now();
  cloudMsg.header.frame_id = "world"; // 设置消息的坐标系
  pub.publish(cloudMsg);
}

// 打印所有优化结果
void print_result_debug(const unsigned int WINDOW_SIZE,
                        std::vector<GRBVar> &position, std::vector<GRBVar> &velocity,
                        std::vector<GRBVar> &quaternion, std::vector<GRBVar> &scale)
{
  for (int i = 0; i < WINDOW_SIZE; i++)
  {
    double p_x = position[3 * i].get(GRB_DoubleAttr_X) / 10;
    double p_y = position[3 * i + 1].get(GRB_DoubleAttr_X) / 10;
    double p_z = position[3 * i + 2].get(GRB_DoubleAttr_X) / 10;
    double v_x = velocity[3 * i].get(GRB_DoubleAttr_X) / 10;
    double v_y = velocity[3 * i + 1].get(GRB_DoubleAttr_X) / 10;
    double v_z = velocity[3 * i + 2].get(GRB_DoubleAttr_X) / 10;
    double q_w = quaternion[4 * i].get(GRB_DoubleAttr_X);
    double q_x = quaternion[4 * i + 1].get(GRB_DoubleAttr_X);
    double q_y = quaternion[4 * i + 2].get(GRB_DoubleAttr_X);
    double q_z = quaternion[4 * i + 3].get(GRB_DoubleAttr_X);
    double scale_1 = scale[2 * i].get(GRB_DoubleAttr_X);
    double scale_2 = scale[2 * i + 1].get(GRB_DoubleAttr_X);
    // std::cout << "优化后 p=(" << p_x << ", " << p_y << ", " << p_z << "), v=("
    //           << v_x << ", " << v_y << ", " << v_z << "), q=("
    //           << q_w << ", " << q_x << ", " << q_y << ", " << q_z << "), scale=("
    //           << scale_1 << ", " << scale_2 << ")" << std::endl;
    std::cout << "优化后 scale= " << scale_1 << ", " << scale_2 << std::endl;
  }
}



// 发布IMU结果
// inline void VIO_estimator::publish_imu_result()
// {
//   if (ESTIMATOR_PUB) // IMU结果发布的优先级小于VIO结果发布
//     return;
//   else
//   {
//     pthread_mutex_lock(&mutex);
//     geometry_msgs::TransformStamped result_Msg;
//     result_Msg.header.stamp = ros::Time::now();
//     result_Msg.header.frame_id = "world"; // 设置消息的坐标系
//     result_Msg.child_frame_id = "sensor_link";
//     result_Msg.transform.translation.x = Ps_all[0];
//     result_Msg.transform.translation.y = Ps_all[1];
//     result_Msg.transform.translation.z = Ps_all[2];
//     result_Msg.transform.rotation.x = gyr_now.second.x();
//     result_Msg.transform.rotation.y = gyr_now.second.y();
//     result_Msg.transform.rotation.z = gyr_now.second.z();
//     result_Msg.transform.rotation.w = gyr_now.second.w();
//     broadcaster.sendTransform(result_Msg);
//     pthread_mutex_unlock(&mutex);
//   }
// }

/*
// tag回调函数调用，取出tag附近的2d特征点，形成初始点云
bool VIO_estimator::pointcloud_initial(const std::map<int, Eigen::Matrix<double, 8, 1>> &image,
                                       double lower_bound_1, double upper_bound_1,
                                       double lower_bound_2, double upper_bound_2)
{
  // tag回调函数调用，取出tag附近的2d特征点，形成初始点云
  std::vector<int> pointIds;
  pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (filterImage(image, lower_bound_1, upper_bound_1, lower_bound_2, upper_bound_2, pointIds))
  {
#pragma omp parallel for           // 并行处理区：for循环
    for (const int &id : pointIds) // 查找编号为 pointIds[i] 的点坐标
    {
      auto it = image.find(id);
      if (it != image.end())
      {
        Eigen::Matrix<double, 8, 1> point = it->second; // 将点坐标加入到 pointCloud 中
#pragma omp critical                                    // 临界区:区域内的代码同一时间只能被一个线程执行
        {
          pointCloud->push_back(pcl::PointXYZ(point(0) / 10, point(1) / 10, point(2) / 10));
        }
      }
    }
    Eigen::Matrix4d T_cam2world = T_now.inverse();
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformedCloudB(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*pointCloud, *transformedCloudB, T_cam2world);
    *pointCloud_world += *transformedCloudB; // 世界系（tag系）下的点云
    // publishPointCloud(pointCloud, nh_tag, pointcloud_pub);

    // std::cout << "点云的点数是 pointCloud " << pointCloud->size() << std::endl;
    // for (const auto &point : pointCloud->points)
    //   std::cout << "point(" << point.x << ", " << point.y << ", " << point.z << ")" << std::endl;
    return 1;
  }
  else
    return 0;
}
*/