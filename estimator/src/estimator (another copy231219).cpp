#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
// 多线程
#include <thread>
#include <mutex>
// ROS 独立回调队列
#include <ros/callback_queue.h>
// #include <ros/callback_queue_interface.h>
// 消息同步器
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/QuaternionStamped.h>
// tag
#include "apriltag_ros/AprilTagDetection.h"
#include "apriltag_ros/AprilTagDetectionArray.h"
// 类声明
#include "estimator.hpp"

#include "debug_tool.hpp"
// 计时
#include <chrono>
#include <fstream>

using namespace message_filters;
using namespace std;
using namespace Eigen;

const int MAX_TIME = 200;
int all_optimize = 0;
int error_optimize = 0;
double single_time;
double total_time[MAX_TIME];
bool single_result;
bool total_result[MAX_TIME];
bool print_flag = 1;

bool print_to_txt()
{
  std::ofstream outfile("/home/yuanzhi/catkin_ws/src/no_laser/data/laser_scale_result.txt");
  for (int i = 0; i < MAX_TIME; i++)
  {
    outfile << total_result[i] << "\t";
  }
  outfile << std::endl;

  for (int i = 0; i < MAX_TIME; i++)
  {
    outfile << total_time[i] << "\t";
  }
  outfile << std::endl;
  outfile.close();
  return 1;
}

void VIO_estimator::acc_callback(const geometry_msgs::Vector3Stamped::ConstPtr &msg) //
{
  if (!imu_init)
  {
    imu_init = 1;
    imu_t0 = (long int)msg->header.stamp.sec;
    acc_0.first = (float)(msg->header.stamp.sec - imu_t0) + (float)msg->header.stamp.nsec / 1000000000;
    acc_0.second << msg->vector.x, msg->vector.y, msg->vector.z;
    gyr_0.first = acc_0.first; // gyc跟随acc更新,用acc的时间戳
    gyr_0.second = gyr.second;
  }
  else
  {
    acc_now.first = (float)(msg->header.stamp.sec - imu_t0) + (float)msg->header.stamp.nsec / 1000000000;
    acc_now.second << msg->vector.x, msg->vector.y, msg->vector.z;
    gyr_now.first = gyr.first; // gyc跟随acc更新,用acc的时间戳
    gyr_now.second = gyr.second * initial_Q.inverse();
    ; // 陀螺仪数据跟随加速度计数据更新
    Qs_now = gyr_now.second;

    dt = acc_now.first - acc_0.first; // 两次加速度数据之间的时间差
    if (dt <= 5e-5)
      return;
    else
    {
      pre_integrate(); // 预积分
      // publish_imu_result();
      acc_0.first = acc_now.first; // 更新acc
      acc_0.second = acc_now.second;
      gyr_0.first = gyr_now.first; // 更新gyr
      gyr_0.second = gyr_now.second;
    }
  }
}

void VIO_estimator::gyr_callback(const geometry_msgs::QuaternionStamped::ConstPtr &msg) //
{
  if (!gyr_init)
  {
    gyr_init = 1;
    gyr.first = (float)(msg->header.stamp.sec) + (float)msg->header.stamp.nsec / 1000000000; // 似乎不需要时间戳
    gyr.second.coeffs() << msg->quaternion.w, msg->quaternion.x, msg->quaternion.y, msg->quaternion.z;
    initial_Q = gyr.second; // 存放初始角度
  }
  else
  {
    gyr.first = (float)(msg->header.stamp.sec) + (float)msg->header.stamp.nsec / 1000000000; // 似乎不需要时间戳
    gyr.second.coeffs() << msg->quaternion.w, msg->quaternion.x, msg->quaternion.y, msg->quaternion.z;
  }
}

void VIO_estimator::feature_callback_cam1(const sensor_msgs::PointCloudConstPtr &feature_msg) //
{
  // 封装单帧图像 得到image图像消息的全部内容
  map<int, Eigen::Matrix<double, 8, 1>> image; // 存放最新图像消息特征点的 编号int、特征点信息Matrix
  int feature_id;
  double x, y, z, p_u, p_v, velocity_x, velocity_y, which_cam;
  Eigen::Matrix<double, 8, 1> xyz_uv_velocity;
  if (feature_msg->points.size() <= 3)
    return;

  for (unsigned int i = 0; i < feature_msg->points.size(); i++)
  {
    x = feature_msg->points[i].x + T_imu_cam(0, 3);
    y = feature_msg->points[i].y + T_imu_cam(1, 3);
    z = feature_msg->points[i].z + T_imu_cam(2, 3); // imu系下的点坐标

    feature_id = feature_msg->channels[0].values[i];
    p_u = feature_msg->channels[1].values[i];
    p_v = feature_msg->channels[2].values[i];
    velocity_x = feature_msg->channels[3].values[i];
    velocity_y = feature_msg->channels[4].values[i];
    which_cam = feature_msg->channels[5].values[i];
    xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y, which_cam; // 单个特征点的坐标、速度存入xyz_uv_velocity
    // image存放一张图的所有特征点信息
    image.emplace(feature_id, xyz_uv_velocity); // 一个feature_id对应一个xyz_uv_velocity
  }

  // 存入滑动窗的空间 每存进去一个滑动窗、就把IMU数据也存进去，并把IMU数据清零
  if (img_queue.size() < WINDOW_SIZE) // 滑动窗不满
  {
    WINDOW_FULL_FLAG = 0;
    if (!add_keyframe(image)) // 图像关键帧进窗img_queue,并获取滑动窗内所有特征点编号的交集
      return;                 // 如果符合关键帧条件，则存入滑动窗，否则退出
    // 读预积分结果、清空预积分
    pthread_mutex_lock(&mutex);   // 加锁 取出预积分结果
    Ps_queue.push_back(Ps_now);   // 位置预积分进窗
    Vs_queue.push_back(Vs_now);   // 速度预积分进窗
    Qs_queue.push_back(Qs_now);   // 转角四元数预积分进窗
    pthread_mutex_unlock(&mutex); // 解锁 结束取预积分结果
    Ps_now.setZero();             // 新一轮预积分，位移置零，速度不变。但是VINS里直接把速度置零了
    // Qs_now.setIdentity();
  }
  else // 滑动窗已满
  {
    WINDOW_FULL_FLAG = 1;
    if (!add_keyframe(image)) // 图像关键帧进窗img_queue,并获取滑动窗内所有特征点编号的交集
      return;                 // 如果符合关键帧条件，则存入滑动窗，否则退出
    // 读预积分结果、清空预积分
    pthread_mutex_lock(&mutex);   // 加锁 取出预积分结果
    Ps_queue.push_back(Ps_now);   // 位置预积分进窗
    Vs_queue.push_back(Vs_now);   // 速度预积分进窗
    Qs_queue.push_back(Qs_now);   // 转角四元数预积分进窗
    pthread_mutex_unlock(&mutex); // 解锁 结束取预积分结果
    Ps_now.setZero();             // 新一轮预积分，位移置零，速度不变。但是VINS里直接把速度置零了
    // Qs_now.setIdentity();
    // 滑动窗已满 需要清理掉最早的数据 对应VINS slidWindow()
    img_queue.pop_front();
    Ps_queue.pop_front();
    Vs_queue.pop_front();
    Qs_queue.pop_front();
  }

  if (WINDOW_FULL_FLAG)
  {
    if (ESTIMATOR_FLAG && !ESTIMATOR_PUB) // 已完成初始化 进入优化阶段 && 且此时未接到tag信息
    {
      GRBEnv env; // 优化环境
      auto start = std::chrono::high_resolution_clock::now();
      env.set(GRB_IntParam_OutputFlag, 1);
      env.set(GRB_IntParam_LogToConsole, 0); // 消息静默

      GRBModel model(env);            // 优化模型
      GRBQuadExpr obj;                // 定义目标函数
      std::vector<GRBVar> position;   // 优化变量: 位姿
      std::vector<GRBVar> velocity;   // 优化变量: 速度
      std::vector<GRBVar> quaternion; // 优化变量: 四元数
      std::vector<GRBVar> scale;      // 优化变量: 尺度因子
      int optimstatus;
      bool re_switch = 1;
      unsigned int optimize_time = 0;
      double object_function_value = 0;

#ifdef DEBUG_estimator
      std::cout << "\n\n\n\n " << std::endl;
      model.set(GRB_IntParam_OutputFlag, 1); // 输出详细信息
#endif

      static_thread = 0;
      model.set(GRB_StringAttr_ModelName, "LVIO_optimization");
      model.set(GRB_IntParam_NonConvex, 2); // 允许非凸性，可能会导致无法获得全局最优解，因为非凸性问题可能有多个局部最优解
      // model.set(GRB_IntParam_Presolve, 0);                                   // 关闭预求解
      add_Variables(model, position, velocity, quaternion, scale);           // 添加优化变量
      add_Constraints(model, position, velocity, quaternion, scale);         // 添加约束
      initial_optimization_variables(position, velocity, quaternion, scale); // 初值

      // 目标函数
      calculate_reprojection_error(obj, position, velocity, quaternion, scale);
      // cout << "计数器 static_thread / 10帧*Active_feature_id*3维 = " << static_thread << " / " << Active_feature_id.size() * 30 << "="
      //      << (float)static_thread / Active_feature_id.size() / 30 << endl; // debug
      calculate_scale_factor_error(obj, scale);                        // 优化器目标函数: 尺度因子误差
      if ((float)static_thread / Active_feature_id.size() / 30 <= 0.8) // 相机处于运动中
      {
        calculate_preintegrate_error(obj, position, velocity, quaternion);
        try
        {
          model.setObjective(obj);
          model.set("TimeLimit", "2.0"); // 最大求解时间
          GRBConstr *comstrs = 0;        // debug
          while (re_switch && optimize_time <= 1)
          {
            model.update();
            model.optimize();
            optimize_time++;
            optimstatus = model.get(GRB_IntAttr_Status);

            switch (optimstatus)
            {
            case GRB_INF_OR_UNBD:
              ROS_WARN("Model INF_OR_UNBD, try resolve...");
              model.set(GRB_IntParam_Presolve, 0); // 关闭预求解
              re_switch = 1;
              single_result = 0;
              break;
            case GRB_INFEASIBLE:
#ifdef DEBUG_estimator
              ROS_ERROR("Model infeasible");
              model.computeIIS();
              model.write("/home/yuanzhi/catkin_ws/src/estimator/tmp/infeasible_model.ilp");
              model.write("/home/yuanzhi/catkin_ws/src/estimator/tmp/infeasible_model.mps");
#endif
              comstrs = model.getConstrs();
              for (int i = 0; i < model.get(GRB_IntAttr_NumConstrs); ++i)
              {
                if (comstrs[i].get(GRB_IntAttr_IISConstr) == 1)
                {
                  cout << "delete infisable constrain: " << comstrs[i].get(GRB_StringAttr_ConstrName) << endl;
                  // Remove a single constraint from the model
                  model.remove(comstrs[i]);
                  break;
                }
              }
              model.feasRelax(GRB_FEASRELAX_LINEAR, true, false, true); // 松弛变量，最小化松弛变量的一范数
              model.set(GRB_IntParam_Presolve, 0);                      // 关闭预求解
              re_switch = 1;
              single_result = 0;
              break;
            case GRB_UNBOUNDED:
#ifdef DEBUG_estimator
              ROS_ERROR("Model unbounded");
              // model.write("/home/yuanzhi/catkin_ws/src/estimator/tmp/unbounded_model.hnt"); // 提示
              // model.write("/home/yuanzhi/catkin_ws/src/estimator/tmp/unbounded_model.rlp");
              // model.write("/home/yuanzhi/catkin_ws/src/estimator/tmp/unbounded_model.mps"); // 模型
#endif
              resize_variables(model, position, velocity, quaternion, scale);
              re_switch = 0;
              single_result = 0;
              break;
            case GRB_OPTIMAL:
              object_function_value = model.get(GRB_DoubleAttr_ObjVal);
              if (object_function_value <= 10) // || object_function_value >= 1e5) // 优化错误，代价函数为0
              {
#ifdef DEBUG_estimator
                ROS_WARN("ignore 1 error result");
                // model.write("/home/yuanzhi/catkin_ws/src/estimator/tmp/bad_model.prm"); // 配置参数
                // model.write("/home/yuanzhi/catkin_ws/src/estimator/tmp/bad_model.mps"); // 模型
                print_result_debug(WINDOW_SIZE, position, velocity, quaternion, scale); // debug 打印优化结果
#endif
                // model.set(GRB_IntParam_Presolve, 0);
                initial_optimization_variables(position, velocity, quaternion, scale);
                resize_variables(model, position, velocity, quaternion, scale);
                re_switch = 1;
                single_result = 0;
              }
              else
              {
#ifdef DEBUG_estimator
                print_result_debug(WINDOW_SIZE, position, velocity, quaternion, scale); // debug 打印优化结果
                error_optimize++;
                cout << "目标函数: " << object_function_value << endl; // 最优解对应的目标函数值
#endif
                re_switch = 0;
                local_to_global(position, quaternion);
                // pub_pointcloud(scale);
                PUB_VIO_FLAG = 1; // 发布VIO优化结果
                single_result = 1;
              }
              break;
            case GRB_TIME_LIMIT:
              re_switch = 0;
#ifdef DEBUG_estimator
              ROS_WARN("time limit");
              object_function_value = model.get(GRB_DoubleAttr_ObjVal);
              // cout << "目标函数: " << object_function_value << endl;                  // 最优解对应的目标函数值
              print_result_debug(WINDOW_SIZE, position, velocity, quaternion, scale); // debug 打印优化结果
#endif
              single_result = 0;
              break;
            default:
              // cout << "不寻常的错误码 = " << optimstatus << endl;
              single_result = 0;
              break;
            }
          }
          auto end = std::chrono::high_resolution_clock::now();
          // if (all_optimize < MAX_TIME)
          // {
          //   single_time = double(std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()); //
          //   total_time[all_optimize] = single_time;
          //   total_result[all_optimize] = single_result;
          // }
          // else if (print_flag)
          // {
          //   print_to_txt();
          //   print_flag = 0;
          // }
          all_optimize++; // 优化次数计数器
        }
        catch (GRBException &ex)
        {
          cout << "Error code =" << ex.getErrorCode() << endl;
          cout << ex.getMessage() << endl;
        }
      }
      else // 相机静止
      {
        // 由于静止，不调用local_to_global
        PUB_VIO_FLAG = 0;
      }
    }
    else // 初始化阶段
    {
      ROS_INFO("estimator: initialization stage");
      // 这里进行初始化
      ESTIMATOR_FLAG = 1; // 完成初始化
    }
  }
}

// 优化失败后收紧变量范围再次优化
bool VIO_estimator::resize_variables(GRBModel &model,
                                     std::vector<GRBVar> &position, std::vector<GRBVar> &velocity,
                                     std::vector<GRBVar> &quaternion, std::vector<GRBVar> &scale)
{
  // 模型约束
  for (int i = 0; i < WINDOW_SIZE; i++)
  {
    int P_id = 3 * i; // 位置、速度下标
    int Q_id = 4 * i; // 四元数下标
    // 位置约束   约束相邻两个窗的最大位移差值 <= 0.01m
    model.addConstr(position[P_id] <= 10 * (std::max(Ps_queue[i][0] + 0.1, Max_Ps)), "resize_p_x_max");
    model.addConstr(position[P_id + 1] <= 10 * (std::max(Ps_queue[i][1] + 0.1, Max_Ps)), "resize_p_y_max");
    model.addConstr(position[P_id + 2] <= 10 * (std::max(Ps_queue[i][2] + 0.1, Max_Ps)), "resize_p_z_max");
    model.addConstr(position[P_id] >= 10 * (std::min(Ps_queue[i][0] - 0.1, -Max_Ps)), "resize_p_x_min");
    model.addConstr(position[P_id + 1] >= 10 * (std::min(Ps_queue[i][1] - 0.1, -Max_Ps)), "resize_p_y_min");
    model.addConstr(position[P_id + 2] >= 10 * (std::min(Ps_queue[i][2] - 0.1, -Max_Ps)), "resize_p_z_min");

    // 速度约束   约束相邻两帧的最大速度差值 <= 0.01m
    model.addConstr(velocity[P_id] <= 10 * (std::max(Vs_queue[i][0] + 0.05, Max_Vs)), "resize_v_x_max");
    model.addConstr(velocity[P_id + 1] <= 10 * (std::max(Vs_queue[i][1] + 0.05, Max_Vs)), "resize_v_y_max");
    model.addConstr(velocity[P_id + 2] <= 10 * (std::max(Vs_queue[i][2] + 0.05, Max_Vs)), "resize_v_z_max");
    model.addConstr(velocity[P_id] >= 10 * (std::min(Vs_queue[i][0] - 0.05, -Max_Vs)), "resize_v_x_min");
    model.addConstr(velocity[P_id + 1] >= 10 * (std::min(Vs_queue[i][1] - 0.05, -Max_Vs)), "resize_v_y_min");
    model.addConstr(velocity[P_id + 2] >= 10 * (std::min(Vs_queue[i][2] - 0.05, -Max_Vs)), "resize_v_z_min");

    // 转角四元数约束  顺序: 虚部x,y,z 实部w,
    // model.addConstr(quaternion[Q_id] <= Max_Delta_Vs * 10);
    // model.addConstr(quaternion[Q_id + 1] <= Max_Delta_Vs * 10);
    // model.addConstr(quaternion[Q_id + 2] <= Max_Delta_Vs * 10);
    // model.addConstr(quaternion[Q_id + 3] <= Max_Delta_Vs * 10);
  }
  return 1;
}

// 为优化变量设置初值
bool VIO_estimator::initial_optimization_variables(
    std::vector<GRBVar> &position, std::vector<GRBVar> &velocity,
    std::vector<GRBVar> &quaternion, std::vector<GRBVar> &scale)
{
  // 向尺度因子数组末尾（最近的帧）填入scale factor初值
  scale[2 * WINDOW_SIZE - 2].set(GRB_DoubleAttr_Obj, scale_factor_1);
  scale[2 * WINDOW_SIZE - 1].set(GRB_DoubleAttr_Obj, scale_factor_2);

  for (int i = 0; i < WINDOW_SIZE; i++)
  {
    int Q_id = 4 * i; // 四元数下标
    int P_id = 3 * i; // 位置、速度下标
    // 添加位姿变量
    position[P_id].set(GRB_DoubleAttr_Obj, 10 * Ps_queue[i][0]);
    position[P_id + 1].set(GRB_DoubleAttr_Obj, 10 * Ps_queue[i][1]);
    position[P_id + 2].set(GRB_DoubleAttr_Obj, 10 * Ps_queue[i][2]);
    velocity[P_id].set(GRB_DoubleAttr_Obj, 10 * Vs_queue[i][0]);
    velocity[P_id + 1].set(GRB_DoubleAttr_Obj, 10 * Vs_queue[i][1]);
    velocity[P_id + 2].set(GRB_DoubleAttr_Obj, 10 * Vs_queue[i][2]);
    // 添加四元数变量 顺序: 实部w, 虚部x,y,z
    quaternion[Q_id].set(GRB_DoubleAttr_Obj, Qs_queue[i].w());
    quaternion[Q_id + 1].set(GRB_DoubleAttr_Obj, Qs_queue[i].x());
    quaternion[Q_id + 2].set(GRB_DoubleAttr_Obj, Qs_queue[i].y());
    quaternion[Q_id + 3].set(GRB_DoubleAttr_Obj, Qs_queue[i].z());
  }
#ifdef DEBUG_estimator
  // for (int i = 0; i < WINDOW_SIZE - 1; i++)
  // {
  //   double p_x = Ps_queue[i][0];
  //   double p_y = Ps_queue[i][1];
  //   double p_z = Ps_queue[i][2];
  //   double v_x = Vs_queue[i][0];
  //   double v_y = Vs_queue[i][1];
  //   double v_z = Vs_queue[i][2];
  //   double q_x = Qs_queue[i].x();
  //   double q_y = Qs_queue[i].y();
  //   double q_z = Qs_queue[i].z();
  //   double q_w = Qs_queue[i].w();
  //   std::cout << "优化前 p=(" << p_x << ", " << p_y << ", " << p_z << "), \t v=("
  //             << v_x << ", " << v_y << ", " << v_z << "), \t q=(" << q_w << ", " << q_x << ", "
  //             << q_y << ", " << q_z << "), \t scale=(" << scale_factor_1 << ", " << scale_factor_2 << ")" << std::endl;
  // }
#endif
  return 1;
}

// 优化目标: 预积分误差
bool VIO_estimator::calculate_preintegrate_error(GRBQuadExpr &obj,
                                                 std::vector<GRBVar> &position, std::vector<GRBVar> &velocity,
                                                 std::vector<GRBVar> &quaternion)
{
  // if (Active_feature_id.size() < MIN_Active_Feature) // 若小于4则不进行重投影误差计算
  //   return 0;
  // 位移
  for (int i = 0; i < WINDOW_SIZE; i++)
  {
    int P_id = 3 * i; // 位置、速度下标
    obj +=
        ((10 * Ps_queue[i][0] - position[P_id]) * (10 * Ps_queue[i][0] - position[P_id]) +
         (10 * Ps_queue[i][1] - position[P_id + 1]) * (10 * Ps_queue[i][1] - position[P_id + 1]) +
         (10 * Ps_queue[i][2] - position[P_id + 2]) * (10 * Ps_queue[i][2] - position[P_id + 2])) *
        P_WEIGHT;
  }
  // 速度
  for (int i = 0; i < WINDOW_SIZE; i++)
  {
    int P_id = 3 * i;
    obj +=
        ((10 * Vs_queue[i][0] - velocity[P_id]) * (10 * Vs_queue[i][0] - velocity[P_id]) +
         (10 * Vs_queue[i][1] - velocity[P_id + 1]) * (10 * Vs_queue[i][1] - velocity[P_id + 1]) +
         (10 * Vs_queue[i][2] - velocity[P_id + 2]) * (10 * Vs_queue[i][2] - velocity[P_id + 2])) *
        V_WEIGHT;
  }

  // 转角
  for (int i = 0; i < WINDOW_SIZE; i++)
  {
    int Q_id = 4 * i; // 四元数下标
    obj +=
        ((Qs_queue[i].w() - quaternion[Q_id]) * (Qs_queue[i].w() - quaternion[Q_id]) +
         (Qs_queue[i].x() - quaternion[Q_id + 1]) * (Qs_queue[i].x() - quaternion[Q_id + 1]) +
         (Qs_queue[i].y() - quaternion[Q_id + 2]) * (Qs_queue[i].y() - quaternion[Q_id + 2]) +
         (Qs_queue[i].z() - quaternion[Q_id + 3]) * (Qs_queue[i].z() - quaternion[Q_id + 3])) *
        Q_WEIGHT;
  }
  return 1;
}

// 优化目标: 尺度因子误差:滑动窗内的所有尺度因子与最新尺度因子之间的差距。越近的帧，差距权重越大
bool VIO_estimator::calculate_scale_factor_error(GRBQuadExpr &obj, std::vector<GRBVar> &scale)
{
  int i;
  for (i = 0; i < WINDOW_SIZE - 1; i++)
  {
    int S_id = 2 * i; // 尺度因子下标
    obj +=
        ((scale_factor_1 - scale[S_id]) * (scale_factor_1 - scale[S_id]) +
         (scale_factor_2 - scale[S_id + 1]) * (scale_factor_2 - scale[S_id + 1])) *
        i * i * SCALE_WEIGHT;
  }
  obj +=
      ((scale_factor_1 - scale[2 * i]) * (scale_factor_1 - scale[2 * i]) +
       (scale_factor_2 - scale[2 * i + 1]) * (scale_factor_2 - scale[2 * i + 1])) *
      i * i * SCALE_WEIGHT;
  return 1;
}

// 优化目标: 重投影误差
bool VIO_estimator::calculate_reprojection_error(GRBQuadExpr &obj,
                                                 std::vector<GRBVar> &position, std::vector<GRBVar> &velocity,
                                                 std::vector<GRBVar> &quaternion, std::vector<GRBVar> &scale)
{
  if (Active_feature_id.size() < MIN_Active_Feature) // 若小于4则不进行重投影误差计算
    return 0;
  // xyz_uv_velocity矩阵包含：x, y, z, p_u, p_v, velocity_x, velocity_y, which_cam
  // image包含：feature_id, xyz_uv_velocity
  float velocity_x, velocity_y, velocity_z;                     // 每一帧之归一化速度
  float velocity_x_1, velocity_y_1, velocity_x_2, velocity_y_2; // 每一帧之归一化速度
  unsigned int cam1_counter, cam2_counter;
  Eigen::Vector3d sum_delta_feature; // debug
  for (int i = 0; i < WINDOW_SIZE - 1; i++)
  {
    int P_id = 3 * i;
    int S_id = 2 * i; // 尺度因子下标
    // 遍历滑动窗内的所有帧, 找出所有活跃点的三维坐标，变换到下一帧的坐标系上
    for (const auto &id : Active_feature_id) // 利用了find_Active_feature_id函数的查找结果
    {
      if (img_queue[i].find(id) != img_queue[i].end() && img_queue[i + 1].find(id) != img_queue[i + 1].end()) // 对于前帧的每一个活跃特征点
      {
        // 前后两帧归一化后的特征点坐标之差值
        Eigen::Vector3d delta_feature = img_queue[i + 1][id].head<3>() / img_queue[i + 1][id].head<3>().norm() -
                                        img_queue[i][id].head<3>() / img_queue[i][id].head<3>().norm();
        sum_delta_feature += delta_feature; // debug
        if (img_queue[i][id][7])            // 根据which_cam区分相机
        {
          pthread_mutex_lock(&mutex);
          // 变换后的特征点坐标与后帧对应特征点之间的重投影误差
          if (delta_feature[0] > 1e-6)
            obj += robust_kernel(delta_feature[0], position[P_id], scale[S_id]) * FEATURE_WEIGHT;
          else
            static_thread++;
          if (delta_feature[1] > 1e-6)
            obj += robust_kernel(delta_feature[1], position[P_id + 1], scale[S_id]) * FEATURE_WEIGHT;
          else
            static_thread++;
          if (delta_feature[2] > 1e-6)
            obj += robust_kernel(delta_feature[2], position[P_id + 2], scale[S_id]) * FEATURE_WEIGHT;
          else
            static_thread++;
          pthread_mutex_unlock(&mutex);

          velocity_x_1 += img_queue[i][id][5];
          velocity_y_1 += img_queue[i][id][6];
          cam1_counter++;
        }
        else
        {
          pthread_mutex_lock(&mutex);
          // 变换后的特征点坐标与后帧对应特征点之间的重投影误差
          if (delta_feature[0] > 1e-6)
            obj += robust_kernel(delta_feature[0], position[P_id], scale[S_id + 1]) * FEATURE_WEIGHT;
          else
            static_thread++;
          if (delta_feature[1] > 1e-6)
            obj += robust_kernel(delta_feature[1], position[P_id + 1], scale[S_id + 1]) * FEATURE_WEIGHT;
          else
            static_thread++;
          if (delta_feature[2] > 1e-6)
            obj += robust_kernel(delta_feature[2], position[P_id + 2], scale[S_id + 1]) * FEATURE_WEIGHT;
          else
            static_thread++;
          pthread_mutex_unlock(&mutex);
          velocity_x_2 += img_queue[i][id][5];
          velocity_y_2 += img_queue[i][id][6];
          cam2_counter++;
        }
      }
    }

    obj += ((10 * scale[2 * i + 1] * velocity_x_2 / cam2_counter + 1e5 * velocity[P_id]) * (10 * scale[2 * i + 1] * velocity_x_2 / cam2_counter + 1e5 * velocity[P_id]) +
            (-10 * scale[2 * i] * velocity_x_1 / cam1_counter + velocity[P_id + 1]) *
                (-10 * scale[2 * i] * velocity_x_1 / cam1_counter + 1e5 * velocity[P_id + 1]) +
            (-5 * (scale[2 * i] * velocity_y_1 / cam1_counter + scale[2 * i + 1] * velocity_y_2 / cam2_counter) + 1e5 * velocity[P_id + 2]) *
                (-5 * (scale[2 * i] * velocity_y_1 / cam1_counter + scale[2 * i + 1] * velocity_y_2 / cam2_counter) + 1e5 * velocity[P_id + 2])) *
           OPTICAL_WEIGHT * 1e-10;
  }
  return 1;
}

// 鲁棒核函数 用于加权cam残差
inline GRBQuadExpr VIO_estimator::robust_kernel(const double delta_feature_component, GRBVar &position_component, GRBVar &scale_component)
{
  // if (10 * delta_feature_component * scale_component - position_component > 10) // 如果误差过大则返回一范数
  //   return (10 * delta_feature_component * scale_component - position_component);//abs
  // else // 重投影误差的二范数
  return (10 * delta_feature_component * scale_component + 1e5 * position_component) *
         (10 * delta_feature_component * scale_component + 1e5 * position_component) * 1e-10;
  // return (10 * delta_feature_component * scale_component +1e5 * position_component) *
  //        (10 * delta_feature_component * scale_component + 1e5 * position_component);
}

// 获取滑动窗内所有特征点编号的交集，作为在整个滑动窗都活跃的特征点
bool VIO_estimator::find_Active_feature_id()
{
  if (!WINDOW_FULL_FLAG) // 若滑动窗不满
    return 0;
  std::unordered_map<int, int> intersection;
  Active_feature_id.clear();
  // 统计整数出现的次数
  for (const auto &set : img_queue)
  {
    for (const auto &pair : set)
    {
      int num = pair.first;
      if (intersection.find(num) != intersection.end())
        intersection[num]++;
      else
        intersection[num] = 1;
    }
  }
  // 找出出现次数等于队列长度的整数，将其添加到结果集合中
  for (const auto &pair : intersection)
  {
    if (pair.second >= WINDOW_SIZE)
      Active_feature_id.push_back(pair.first);
  }
  return 1;
}

// 将优化器结果的local位移累加到global的相对于tag系下的位姿上
inline void VIO_estimator::local_to_global(std::vector<GRBVar> &position, std::vector<GRBVar> &quaternion)
{
  // 累积量
  // P_now[0] += position[3 * WINDOW_SIZE - 3].get(GRB_DoubleAttr_X) / 10;
  // P_now[1] += position[3 * WINDOW_SIZE - 2].get(GRB_DoubleAttr_X) / 10;
  // P_now[2] += position[3 * WINDOW_SIZE - 1].get(GRB_DoubleAttr_X) / 10;
  // 增量
  P_now[0] = position[3 * WINDOW_SIZE - 3].get(GRB_DoubleAttr_X) / 10;
  P_now[1] = position[3 * WINDOW_SIZE - 2].get(GRB_DoubleAttr_X) / 10;
  P_now[2] = position[3 * WINDOW_SIZE - 1].get(GRB_DoubleAttr_X) / 10;
  if (abs(P_now[0] == 0.1) || abs(P_now[1] == 0.1))
    GOOD_VIO_FLAG = 0; // 坏结果
  else
    GOOD_VIO_FLAG = 1; // 好结果
  // 顺序 w x y z
  // Eigen::Quaterniond Q_local(quaternion[3 * WINDOW_SIZE - 4].get(GRB_DoubleAttr_X), quaternion[3 * WINDOW_SIZE - 3].get(GRB_DoubleAttr_X),
  //                            quaternion[3 * WINDOW_SIZE - 2].get(GRB_DoubleAttr_X), quaternion[3 * WINDOW_SIZE - 1].get(GRB_DoubleAttr_X));
  // Qs_now = Qs_now * Q_local;
  // R_now = Qs_now.toRotationMatrix();
}

// 发布优化后的点云
inline void VIO_estimator::pub_pointcloud(std::vector<GRBVar> &scale)
{
  float scale_optimized_1 = scale[2 * WINDOW_SIZE - 2].get(GRB_DoubleAttr_X);
  float scale_optimized_2 = scale[2 * WINDOW_SIZE - 1].get(GRB_DoubleAttr_X);

  sensor_msgs::PointCloudPtr feature_points(new sensor_msgs::PointCloud); // 测试点云发布
  feature_points->header.stamp = ros::Time::now();
  feature_points->header.frame_id = "sensor_link";

  std::map<int, Eigen::Matrix<double, 8, 1>> &last_map = img_queue.back();
  for (const auto &pair : last_map)
  {
    const Eigen::Matrix<double, 8, 1> &matrix = pair.second;
    geometry_msgs::Point32 p; // 测试点云发布
    if (matrix(7))
    {
      p.x = matrix(0) * scale_optimized_1 * 1e-4;
      p.y = matrix(1) * scale_optimized_1 * 1e-4;
      p.z = matrix(2) * scale_optimized_1 * average_f_1 * 1e-4;
    }
    else
    {
      p.x = matrix(0) * scale_optimized_2 * 1e-4;
      p.y = matrix(1) * scale_optimized_2 * 1e-4;
      p.z = matrix(2) * scale_optimized_2 * 1e-4 * average_f_1;
    }

    feature_points->points.emplace_back(p);
  }
  pointcloud_pub.publish(feature_points); // 测试点云发布
}

// 发布VIO优化结果: TF
inline void VIO_estimator::publish_vio_result()
{
  // VIO优化结果
  geometry_msgs::TransformStamped result_Msg;
  result_Msg.header.stamp = ros::Time::now();
  result_Msg.header.frame_id = "world"; // 设置消息的坐标系
  result_Msg.child_frame_id = "sensor_link";

  pthread_mutex_lock(&mutex); // 发布VIO优化结果
  result_Msg.transform.rotation.x = ideal_quatern.x();
  result_Msg.transform.rotation.y = ideal_quatern.y();
  result_Msg.transform.rotation.z = ideal_quatern.z();
  result_Msg.transform.rotation.w = ideal_quatern.w();

  // 转换到IMU坐标系下，把进给方向的运动清零
  Rs_now = ideal_quatern.toRotationMatrix();
  Eigen::Vector3d local_P;
  // P_now和Ps_now都是初始IMU坐标系，先转换到基座坐标系，再转换到当前IMU坐标系
  if (GOOD_VIO_FLAG) // 若好结果则使用优化解
    local_P = Rs_now * R_imu2base * P_now;
  else // 若坏结果则使用IMU积分解
    local_P = Rs_now * R_imu2base * Ps_now;
  local_P.y() = 0; // 不计算进给方向的加速度
  Eigen::Vector3d global_P = Rs_now.transpose() * local_P;
  delta_y_world = P_VIOresult_WEIGHT * global_P[0];
  delta_z_world = P_VIOresult_WEIGHT * global_P[1];
  delta_x_world = P_VIOresult_WEIGHT * global_P[2];

  // if (PUB_VIO_FLAG) // 如果产生了好数据
  {
    // pub_ideal_counter = 5;
    result_Msg.transform.translation.x = delta_x_world + ideal_trace[0];
    result_Msg.transform.translation.y = delta_y_world + ideal_trace[1];
    result_Msg.transform.translation.z = delta_z_world + ideal_trace[2];
  }
  // else // 否则只发布理想值
  {
    // if(pub_ideal_counter-- <= 0)
    // {
    // result_Msg.transform.translation.x = ideal_trace[0];
    // result_Msg.transform.translation.y = ideal_trace[1];
    // result_Msg.transform.translation.z = ideal_trace[2];
    // }
  }
  pthread_mutex_unlock(&mutex);
  // if (PUB_VIO_FLAG) // 如果产生了好数据
  {
    broadcaster.sendTransform(result_Msg);
    PUB_VIO_FLAG = 0;
  }
}

// 判断特征点的图像坐标是否很靠近已知深度值的像素点
bool VIO_estimator::filterImage(const std::map<int, Eigen::Matrix<double, 8, 1>> &image,
                                double lower_bound_1, double upper_bound_1,
                                double lower_bound_2, double upper_bound_2, std::vector<int> &result)
{
  // 筛选出符合条件的编号
  // 8维向量的内容: x, y, z, p_u, p_v, velocity_x, velocity_y, which_Cam
  std::for_each(image.begin(), image.end(), [&](const std::pair<int, Eigen::Matrix<double, 8, 1>> &pair) -> void
                {
                  // 存在问题：两张图片的数据已经合并在一起了，cam2的数据也会进来，需要再判断一下xyz坐标，筛选出cam1的点
                  const Eigen::Matrix<double, 8, 1> &vec = pair.second;
                  bool condition_1 = (vec[3] > lower_bound_1 && vec[3] < upper_bound_1) ? 1 : 0; // 检查第四个元素是否在区间内
                  bool condition_2 = (vec[4] > lower_bound_2 && vec[4] < upper_bound_2) ? 1 : 0; // 检查第五个元素是否在区间内
                  if (condition_1 && condition_2)
                    result.push_back(pair.first); });
  if (!result.empty())
    return 1;
  else
  {
    ROS_WARN("Cannot find features near tag");
    return 0;
  }
}

// 计算传感器相对于tag系(世界系)的变换阵
void VIO_estimator::apriltag_callback_cam1(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg)
{
  if (!ESTIMATOR_FLAG)
    return;
  if (msg->detections.empty())
    return;
  else
  {
    ESTIMATOR_PUB = 1; // 暂停VIO计算
    const std_msgs::Header &Header = msg->header;
    const std::vector<apriltag_ros::AprilTagDetection> detections = msg->detections; // 接收 AprilTagDetection[] 类型的 detections
    int num_detections = detections.size();                                          // 获取 detections 数组的大小                                                       // 存储 id 和 pose
    geometry_msgs::PoseWithCovarianceStamped pose_temp;
    Eigen::Quaterniond q_cam_tag; // tag在cam1系下的四元数
    Eigen::Vector3d t_cam_tag;    // tag在cam1系下的平移向量

    for (int i = 0; i < num_detections; i++) // 处理 detections 数据
    {
      pose_temp = detections[i].pose;
      t_cam_tag << pose_temp.pose.pose.position.x, pose_temp.pose.pose.position.y, pose_temp.pose.pose.position.z;
      q_cam_tag.coeffs() << pose_temp.pose.pose.orientation.w, pose_temp.pose.pose.orientation.x, pose_temp.pose.pose.orientation.y, pose_temp.pose.pose.orientation.z;
    }

    // 计算 T_cam_tag
    Eigen::Matrix4d T_cam_tag = Eigen::Matrix4d::Identity();
    T_cam_tag.block<3, 1>(0, 3) = t_cam_tag;
    T_cam_tag.block<3, 3>(0, 0) = q_cam_tag.toRotationMatrix();
    // 计算 T_tag_imu
    pthread_mutex_lock(&mutex);
    T_tag = T_imu_cam.inverse() * T_cam_tag.inverse(); //  imu系在tag系下的齐次变换矩阵
    P_tag = T_tag.block<3, 1>(0, 3);                   //  imu系在tag系下的平移向量
    R_tag = T_tag.block<3, 3>(0, 0).transpose();       //  imu系在tag系下的旋转矩阵
    Q_tag = Quaterniond(R_tag);                        //  imu系在tag系下的四元数
    pthread_mutex_unlock(&mutex);

    // publish_vio_result();
  }
  ESTIMATOR_PUB = 0;
}

void VIO_estimator::tag_center_callback(const geometry_msgs::PointStamped::ConstPtr &msg)
{
  tag_center_u = msg->point.x;
  tag_center_v = msg->point.y;
  int tag_id = msg->point.z;
  float plane_range = 250;
  if (tag_id == 0) // home位放置0号tag
  {
    // 取出tag附近的2d特征点，形成初始点云
    // if (!pointcloud_initial(img_queue.back(), tag_center_u - plane_range, tag_center_u + plane_range, tag_center_v - plane_range, tag_center_v + plane_range))
    //   ROS_ERROR("fail to add pointcloud from Apriltag -_- ");
  }
}

// 激光点深度值测量的回调函数：接收两个cam的尺度因子.在图像节点放大 e05倍
void VIO_estimator::laser_callback(const geometry_msgs::PointStamped::ConstPtr &msg) // laser 深度差值 回调
{
  if (ESTIMATOR_FLAG) // 只在优化阶段填入scale factor初值
  {
    scale_factor_1 = msg->point.x;
    scale_factor_2 = msg->point.y;
  }
}

// 更新最新ideal trace
void VIO_estimator::ideal_trace_callack(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  ideal_trace[0] = msg->pose.position.x;
  ideal_trace[1] = msg->pose.position.y;
  ideal_trace[2] = msg->pose.position.z;
  ideal_quatern.w() = msg->pose.orientation.w;
  ideal_quatern.x() = msg->pose.orientation.x;
  ideal_quatern.y() = msg->pose.orientation.y;
  ideal_quatern.z() = msg->pose.orientation.z;
  publish_vio_result();
}

// IMU预积分 每次收到最新imu数据后运行 每运行一次就把最新IMU数据累加到PS、VS里
inline void VIO_estimator::pre_integrate()
{
  // Vector3d un_acc_1 = Rs[j] * (linear_acceleration - Bas[img_count]) - g;
  // Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1); // 均值滤波：上一次的加速度和当前值分别去偏差去重力后求平均

  // 加锁 更新预积分结果
  pthread_mutex_lock(&mutex);
  Ps_now += Vs_now * dt + 0.5 * dt * dt * acc_now.second;
  Vs_now += dt * acc_now.second;
  pthread_mutex_unlock(&mutex);
}

// 图像关键帧进窗img_queue,并获取滑动窗内所有特征点编号的交集
bool VIO_estimator::add_keyframe(std::map<int, Eigen::Matrix<double, 8, 1>> &image)
{
  // 如果IMU的任意一轴位置积分值超过0.05m,则允许存入关键帧
  bool imu_flag = ((Ps_now[0] > MIN_Ps_for_1_image) || (Ps_now[1] > MIN_Ps_for_1_image) || (Ps_now[2] > MIN_Ps_for_1_image) ? 1 : 0);
  if (imu_flag || !ESTIMATOR_FLAG) // 如果符合关键帧条件之一:1 IMU积分条件 2 还没有进行初始化
  {
    img_queue.push_back(image);
    if (!find_Active_feature_id() && ESTIMATOR_FLAG) // 获取滑动窗内所有特征点编号的交集
      ROS_WARN("no Active feature id!");
    return 1;
  }
  else
    return 0;
}

// 添加优化变量
void VIO_estimator::add_Variables(GRBModel &model,
                                  std::vector<GRBVar> &position, std::vector<GRBVar> &velocity,
                                  std::vector<GRBVar> &quaternion, std::vector<GRBVar> &scale)
{
  // 位姿、速度和四元数
  position.resize(3 * WINDOW_SIZE);
  velocity.resize(3 * WINDOW_SIZE);
  quaternion.resize(4 * WINDOW_SIZE);
  scale.resize(2 * WINDOW_SIZE); // 每一帧内两个相机存放两个尺度因子
  for (int i = 0; i < WINDOW_SIZE; i++)
  {
    int Q_id = 4 * i; // 四元数下标
    int P_id = 3 * i; // 位置、速度下标
    // 添加位姿变量
    position[P_id] = model.addVar(-Max_Ps * 10, Max_Ps * 10, 0.0, GRB_CONTINUOUS, "position_x");
    position[P_id + 1] = model.addVar(-Max_Ps * 10, Max_Ps * 10, 0.0, GRB_CONTINUOUS, "position_y");
    position[P_id + 2] = model.addVar(-Max_Ps * 10, Max_Ps * 10, 0.0, GRB_CONTINUOUS, "position_z");
    // 添加速度变量
    velocity[P_id] = model.addVar(-Max_Vs * 10, Max_Vs * 10, 0.0, GRB_CONTINUOUS, "velocity_x");
    velocity[P_id + 1] = model.addVar(-Max_Vs * 10, Max_Vs * 10, 0.0, GRB_CONTINUOUS, "velocity_y");
    velocity[P_id + 2] = model.addVar(-Max_Vs * 10, Max_Vs * 10, 0.0, GRB_CONTINUOUS, "velocity_z");
    // 添加四元数变量 顺序: 实部w, 虚部x,y,z
    quaternion[Q_id] = model.addVar(-1.0, 1.0, 0.0, GRB_CONTINUOUS, "quaternion_w");
    quaternion[Q_id + 1] = model.addVar(-1.0, 1.0, 0.0, GRB_CONTINUOUS, "quaternion_x");
    quaternion[Q_id + 2] = model.addVar(-1.0, 1.0, 0.0, GRB_CONTINUOUS, "quaternion_y");
    quaternion[Q_id + 3] = model.addVar(-1.0, 1.0, 0.0, GRB_CONTINUOUS, "quaternion_z");
  }
  // 添加尺度因子变量
  for (int j = 0; j < 2 * WINDOW_SIZE; j++)
  {
    scale[j] = model.addVar(1, 1e3, 0.0, GRB_CONTINUOUS);
  }
}

// 添加约束
void VIO_estimator::add_Constraints(GRBModel &model,
                                    std::vector<GRBVar> &position, std::vector<GRBVar> &velocity,
                                    std::vector<GRBVar> &quaternion, std::vector<GRBVar> &scale)
{
  // 模型约束
  for (int i = 0; i < WINDOW_SIZE - 1; i++)
  {
    int P_id = 3 * i; // 位置、速度下标
    // 位置约束   约束相邻两个窗的最大位移差值 <= 0.01m
    model.addConstr(position[P_id + 3] - position[P_id] <= Max_Delta_Ps * 10, "Max_delta_Ps_x");
    model.addConstr(position[P_id + 4] - position[P_id + 1] <= Max_Delta_Ps * 10, "Max_delta_Ps_y");
    model.addConstr(position[P_id + 5] - position[P_id + 2] <= Max_Delta_Ps * 10, "Max_delta_Ps_z");

    // 速度约束   约束相邻两帧的最大速度差值 <= 0.01m
    model.addConstr(velocity[P_id + 3] - velocity[P_id] <= Max_Delta_Vs * 10, "Max_delta_Vs_x");
    model.addConstr(velocity[P_id + 4] - velocity[P_id + 1] <= Max_Delta_Vs * 10, "Max_delta_Vs_y");
    model.addConstr(velocity[P_id + 5] - velocity[P_id + 2] <= Max_Delta_Vs * 10, "Max_delta_Vs_z");

    // 位置、速度联合约束
    model.addConstr(position[P_id + 3] <= velocity[P_id] * DT_IMG + TOLERANCE * 10, "Ps_x_Vs_x_upper"); // 约束位移-速度关系
    model.addConstr(position[P_id + 4] <= velocity[P_id + 1] * DT_IMG + TOLERANCE * 10, "Ps_y_Vs_y_upper");
    model.addConstr(position[P_id + 5] <= velocity[P_id + 2] * DT_IMG + TOLERANCE * 10, "Ps_z_Vs_z_upper");
    model.addConstr(position[P_id + 3] >= velocity[P_id] * DT_IMG - TOLERANCE * 10, "Ps_x_Vs_x_lower"); // 约束位移-速度关系
    model.addConstr(position[P_id + 4] >= velocity[P_id + 1] * DT_IMG - TOLERANCE * 10, "Ps_y_Vs_y_lower");
    model.addConstr(position[P_id + 5] >= velocity[P_id + 2] * DT_IMG - TOLERANCE * 10, "Ps_z_Vs_z_lower");
  }
  for (int i = 0; i < WINDOW_SIZE; i++)
  {
    int Q_id = 4 * i; // 四元数下标
    // 转角四元数约束  顺序: 虚部x,y,z 实部w,
    model.addQConstr(quaternion[Q_id] * quaternion[Q_id] + quaternion[Q_id + 1] * quaternion[Q_id + 1] +
                             quaternion[Q_id + 2] * quaternion[Q_id + 2] + quaternion[Q_id + 3] * quaternion[Q_id + 3] ==
                         1,
                     "standard_quaternion");
  }
}

VIO_estimator::VIO_estimator()
{
  mutex = PTHREAD_MUTEX_INITIALIZER; // 多result线程互斥锁 初始化
  imu_init = 0;                      // imu初始化并更新时间戳起点
  gyr_init = 0;
  WINDOW_FULL_FLAG = 0; // 滑动窗不满
  ESTIMATOR_FLAG = 0;   // 0:初始化阶段; 1:优化阶段
  ESTIMATOR_PUB = 0;
  PUB_VIO_FLAG = 0;
  GOOD_VIO_FLAG = 0;
  imu_t0 = 0;
  dt = 0;
  img_count = 0;      // 图像计数器，在填满滑动窗时使用
  scale_factor_1 = 1e5; // 最新尺度因子初值 cam1
  scale_factor_2 = 1e5; // 最新尺度因子初值 cam2

  // cam1在imu坐标系下的变换矩阵，外参 需要标定!!!!!!!!!!!!!
  T_imu_cam << 0, 0, 1, 0.0181,
      -1, 0, 0, -0.0024,
      0, -1, 0, 0.0029,
      0, 0, 0, 1;
  R_imu2base << 0, 0, -1,
      1, 0, 0,
      0, -1, 0;
  Qs_now.coeffs() << 1, 0, 0, 0; // 根据IMU得到的转角

  P_tag << 0, 0, 0;                   // 根据tag得到的当前位置（相对于home）
  Q_tag.coeffs() << 1, 0, 0, 0;       // 根据tag得到的当前转角四元数（相对于home）
  R_tag << 1, 0, 0, 0, 1, 0, 0, 0, 1; // 根据tag得到的当前转角矩阵（相对于home）
  T_tag.setIdentity();

  // 对于不同频率的消息设置独立回调队列,独立的ROS句柄
  ros::CallbackQueue queue_img, queue_tag, queue_imu, queue_gyc;
  nh_img.setCallbackQueue(&queue_img);
  nh_tag.setCallbackQueue(&queue_tag);
  nh_imu.setCallbackQueue(&queue_imu);
  nh_gyc.setCallbackQueue(&queue_gyc);

  ros::Subscriber acc_listener = nh_imu.subscribe("/filter/free_acceleration", 5, &VIO_estimator::acc_callback, this); // IMU回调
  ros::Subscriber gyr_listener = nh_gyc.subscribe("/filter/quaternion", 5, &VIO_estimator::gyr_callback, this);        // gyr回调

  ros::Subscriber img_listener_1 = nh_img.subscribe("pointcloud_talker", 1, &VIO_estimator::feature_callback_cam1, this); // 相机 图像特征点 回调

  ros::Subscriber sub = nh_tag.subscribe("/laser_distance_talker", 2, &VIO_estimator::laser_callback, this);             // laser 深度差值 回调
  ros::Subscriber tag_listener_1 = nh_tag.subscribe("/tag_detections", 1, &VIO_estimator::apriltag_callback_cam1, this); // 相机1 图像apriltag坐标 回调
  ros::Subscriber ideal_trace_sub = nh_tag.subscribe("/ideal_trace", 5, &VIO_estimator::ideal_trace_callack, this);      // 理想轨迹 回调
  // ros::Subscriber tag_center_listener_1 = nh_tag.subscribe("/tag_img_coordinate", 1, &VIO_estimator::tag_center_callback, this); // 相机1 图像apriltag坐标 回调

  pointcloud_pub = nh_tag.advertise<sensor_msgs::PointCloud>("transformed_cloud", 1);
  // result_pub = nh_tag.advertise<geometry_msgs::TransformStamped>("vio_result", 1);

  // 每个队列内再单独设置多线程
  std::thread spinner_thread_img([&queue_img]()
                                 {ros::MultiThreadedSpinner spinner_img; spinner_img.spin(&queue_img); });
  std::thread spinner_thread_tag([&queue_tag]()
                                 {ros::MultiThreadedSpinner spinner_tag; spinner_tag.spin(&queue_tag); });
  std::thread spinner_thread_imu([&queue_imu]()
                                 {ros::MultiThreadedSpinner spinner_tag; spinner_tag.spin(&queue_imu); });
  std::thread spinner_thread_gyc([&queue_gyc]()
                                 {ros::MultiThreadedSpinner spinner_tag; spinner_tag.spin(&queue_gyc); });

  ROS_INFO("estimator initialization finished");
  ros::spin();
  spinner_thread_img.join();
  spinner_thread_tag.join();
  spinner_thread_imu.join();
  spinner_thread_gyc.join();
}

VIO_estimator::~VIO_estimator(void)
{
  Active_feature_id.clear();
  img_queue.clear();
  Ps_queue.clear();
  Vs_queue.clear();
  Qs_queue.clear();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "estimator");
  VIO_estimator estimator_handle; // argc, argv

  return 1;
}
