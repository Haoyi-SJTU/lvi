#include <iostream>
#include "opencv2/opencv.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <map>
// ROS
#include <ros/callback_queue.h> // ROS 独立回调队列
#include <ros/ros.h>
#include "geometry_msgs/PointStamped.h"
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud.h>
// 多线程
#include <thread>
#include <mutex>

// 计时
// #include <chrono>

#include "receive_img_1.hpp"

using namespace std;
using namespace cv;

// double single_time_process, single_time_all;

img_process::img_process()
{
  mutex = PTHREAD_MUTEX_INITIALIZER; // 多线程互斥锁 初始化
  latest_tag_time = 0;
  // 相机通用参数
  LASER_DELAY_TIME = 80;  // 从改变激光标志符到取得激光照片的时间差，ms
  OPTICAL_DELAY_TIME = 5; // 从改变激光标志符laser.laser_on到暂停或开启光流的时间差，ms
  OPTICAL_DELAY_FLAG = 0; // 标志符，从改变激光标志符laser.laser_on并延迟OPTICAL_DELAY_TIME后，暂停或开启光流

  // 1号相机
  K_1 = (Mat_<double>(3, 3) << 614.668562, 0, 313.603877,
         0, 614.827375, 279.414349,
         0, 0, 1);
  // 畸变参数
  k1_1 = 0.056723;
  k2_1 = -0.141970;
  p1_1 = -0.000390;
  p2_1 = -0.000620;
  distCoeffs_1 = (cv::Mat_<double>(5, 1) << k1_1, k2_1, p1_1, p2_1, 0);
  FOCAL_LENGTH_1 = (int)(K_1.at<double>(0, 0) + K_1.at<double>(1, 1)) / 2;
  if_first_1 = 1;
  img_time_now_1 = 0;
  img_time_1_1 = 0;
  init_pub_1 = 0;
  PUB_FRAME_1 = 1;

  // 2号相机
  K_2 = (Mat_<double>(3, 3) << 617.018253, 0, 321.903251,
         0, 616.981833, 274.560306,
         0, 0, 1);
  // 畸变参数
  k1_2 = 0.076770;
  k2_2 = -0.197154;
  p1_2 = -0.000200;
  p2_2 = 0.000091;
  distCoeffs_2 = (cv::Mat_<double>(5, 1) << k1_2, k2_2, p1_2, p2_2, 0);
  FOCAL_LENGTH_2 = (int)(K_2.at<double>(0, 0) + K_2.at<double>(1, 1)) / 2;
  if_first_2 = 1;
  // pub_count_2 = 0; // 照片计数器
  img_time_now_2 = 0;
  img_time_1_2 = 0;
  init_pub_2 = 0;
  PUB_FRAME_2 = 1;
  // 计算尺度因子时用两个相机的四个焦距计算平均焦距，// 错误：不应当结合像元尺寸得物理焦距
  average_f_1 = (K_1.at<double>(0, 0) + K_1.at<double>(1, 1)) / 2; // * 1.12e-6;
  average_f_2 = (K_2.at<double>(0, 0) + K_2.at<double>(1, 1)) / 2; // * 1.12e-6;

  cv::namedWindow("img_now_1", cv::WINDOW_NORMAL);
  cv::namedWindow("img_now_2", cv::WINDOW_NORMAL);
  cv::resizeWindow("img_now_1", 640, 480);
  cv::resizeWindow("img_now_2", 640, 480);
  cv::startWindowThread();

  std::thread laser_thread(&laser_cam::laser_switch, &laser); // 激光控制 子线程
  laser_thread.detach();

  std::thread optical_delay_thread(&img_process::optical_delay, this); // 光流控制 子线程
  optical_delay_thread.detach();

  ros::NodeHandle nh_img; // 专用
  ros::NodeHandle nh_pub;
  ros::CallbackQueue queue_pub;
  nh_pub.setCallbackQueue(&queue_pub);

  image_transport::ImageTransport it(nh_img);
  image_transport::Subscriber sub1 = it.subscribe("camera1/image", 1, &img_process::imageCallback_1, this);
  image_transport::Subscriber sub2 = it.subscribe("camera2/image", 1, &img_process::imageCallback_2, this);

  ros::Subscriber tag_listener_1 = nh_img.subscribe("/tag_detections", 1, &img_process::apriltag_callback_cam1, this); // 相机1 图像apriltag坐标 回调

  img_pub = nh_pub.advertise<sensor_msgs::PointCloud>("pointcloud_talker", 10);                   // 发点云消息
  laser_distance_pub = nh_pub.advertise<geometry_msgs::PointStamped>("laser_distance_talker", 2); // 发布激光测距消息
  img_timer = nh_pub.createTimer(ros::Duration(1), boost::bind(&img_process::laser_timer, this, _1), false, true);
  // 定时器：发布者回调函数timer 激光测距的时间间隔1s

  std::thread spinner_thread_pub([&queue_pub]()
                                 {ros::MultiThreadedSpinner spinner_pub; spinner_pub.spin(&queue_pub); });

  ROS_DEBUG("img_process initialization finished.");
  ros::spin();
  spinner_thread_pub.join();
}

img_process::~img_process(void)
{
  cv::destroyAllWindows();
  pts_1_1.clear();
  pts_now_1.clear();
  un_pts_now_1.clear();
  ids_1.clear();
  n_pts_1.clear();
  un_pts_map_0_1.clear();
  un_pts_map_1_1.clear();
  pts_velocity_1.clear();

  pts_1_2.clear();
  pts_now_2.clear();
  un_pts_now_2.clear();
  // track_cnt_2.clear();
  ids_2.clear();
  n_pts_2.clear();
  un_pts_map_0_2.clear();
  un_pts_map_1_2.clear();
  pts_velocity_2.clear();
}

void img_process::optical_delay()
{
  while (1)
  {
    if (laser.laser_on == OPTICAL_DELAY_FLAG)
    {
      if (!OPTICAL_DELAY_FLAG) // 从laser切回光流
      {
        waitKey(OPTICAL_DELAY_TIME + 70);
      }
      else // 从光流切到laser
      {
        // waitKey(OPTICAL_DELAY_TIME);
      }
      OPTICAL_DELAY_FLAG = !laser.laser_on;
    }
  }
}

bool img_process::laser_timer(const ros::TimerEvent &tmEvt)
{
  laser.laser_on = 1;        // 打开laser
  waitKey(LASER_DELAY_TIME); // 等待一点时间，这时拍到完整的激光点

  pthread_mutex_lock(&mutex);
  if (!laser.run_once(img_laser_1, img_laser_2, laser_depth)) // 返回的depth里深度的顺序来自于给的图像里深度的顺序
  {
    ROS_WARN("双相机没提取到深度");
    return 0;
  }
  pthread_mutex_unlock(&mutex);
  laser.laser_on = 0; // 关闭laser
  geometry_msgs::PointStamped msg;
  // 上次看到tag,冷却2s后才能再次用激光更新cam1的深度值
  ros::Time current_time = ros::Time::now();
  if (current_time.toSec() - latest_tag_time >= 2)
  {
    pthread_mutex_lock(&mutex);
    depth_list_1 = laser_depth[0]; // 将depth存入
    pthread_mutex_unlock(&mutex);
  }
  depth_list_2 = laser_depth[1]; // 将depth存入
  cout << "深度(" << depth_list_1 << ", " << depth_list_2 << ") m " << endl;
  cout << "优化前尺度因子(" << depth_list_1 / average_f_1 << ", " << depth_list_2 / average_f_2 << ") m " << endl;
  pthread_mutex_lock(&mutex);
  msg.point.x = depth_list_1 / average_f_1 * 1e05; // 尺度因子 = 深度值/焦距
  pthread_mutex_unlock(&mutex);
  msg.point.y = depth_list_2 / average_f_2 * 1e05; // 尺度因子 = 深度值/焦距
  laser_distance_pub.publish(msg);
  return 1;
}

// 根据status删减容器v
void img_process::reduceVector(vector<cv::Point2f> &v, vector<uchar> status)
{
  int j = 0;
  for (int i = 0; i < int(v.size()); i++)
  {
    if (status[i])
    {
      v[j++] = v[i];
    }
  }
  v.resize(j);
}

void img_process::reduceVector(vector<int> &v, vector<uchar> status)
{
  int j = 0;
  for (int i = 0; i < int(v.size()); i++)
    if (status[i])
      v[j++] = v[i];
  v.resize(j);
}

// 判断特征点是否超出画面边界
bool img_process::inBorder(const cv::Point2f &pt)
{
  int img_x = cvRound(pt.x);
  int img_y = cvRound(pt.y);
  return (2 <= img_x && img_x < COL - 2 && 2 <= img_y && img_y < ROW - 2);
}

// 反向投影对称
// 似乎是根据图像坐标计算它相对于坐标轴的偏角
void img_process::backprojectSymmetric(const Eigen::Vector2d &p_u, double &theta, double &phi)
{
  double tol = 1e-10;
  double p_u_norm = p_u.norm();
  if (p_u_norm < 1e-10)
    phi = 0.0;
  else
    phi = atan2(p_u(1), p_u(0));
  // int npow = 3;

  Eigen::MatrixXd coeffs(4, 1);
  coeffs.setZero();
  coeffs(0) = -p_u_norm;
  coeffs(1) = 1;
  coeffs(3) = k2_1;
  // 得到对应于多项式的伴随矩阵的特征值。 特征值对应于多项式的根
  Eigen::MatrixXd A(3, 3);
  A.setZero();
  A.block(1, 0, 2, 2).setIdentity();
  A.col(2) = -coeffs.block(0, 0, 3, 1) / coeffs(3);
  Eigen::EigenSolver<Eigen::MatrixXd> es(A);
  Eigen::MatrixXcd eigval = es.eigenvalues();

  std::vector<double> thetas;
  for (int i = 0; i < eigval.rows(); ++i)
  {
    if (fabs(eigval(i).imag()) > tol)
      continue;
    double t = eigval(i).real();
    if (t < -tol)
      continue;
    else if (t < 0.0)
      t = 0.0;
    thetas.emplace_back(t);
  }
  if (thetas.empty())
    theta = p_u_norm;
  else
    theta = *std::min_element(thetas.begin(), thetas.end());
}

// 将点从图像平面变换到归一化平面.参数p：图像坐标系下的坐标；参数P：投影射线坐标
void img_process::liftProjective(const Eigen::Vector2d &p, Eigen::Vector3d &P)
{
  // 根据θ和φ得到一条射线，空间中这条线上的点都会被投影到图像上的这个点上
  double theta, phi;
  backprojectSymmetric(p, theta, phi);
  // 归一化平面:半径为1的球面上,射线与球面的交点
  P(0) = sin(theta) * cos(phi);
  P(1) = sin(theta) * sin(phi);
  P(2) = cos(theta);
}

// 用相机焦距、光心修正图像特征点cur_pts、forw_pts，得到校正后的特征点un_cur_pts、un_forw_pts
// 用校正后的点计算基础矩阵，剔除离群值
void img_process::rejectWithF(bool which_cam)
{
  if (which_cam) // 1号相机
  {
    if (pts_now_1.size() >= 8)
    { // z_cam
      vector<cv::Point2f> tmp_un_pts_1(pts_1_1.size()), tmp_un_pts_now(pts_now_1.size());
      for (unsigned int i = 0; i < pts_1_1.size(); i++)
      {
        Eigen::Vector3d tmp_p;
        liftProjective(Eigen::Vector2d(pts_1_1[i].x, pts_1_1[i].y), tmp_p); // Lifts a point from the image plane to its projective ray
        tmp_p.x() = FOCAL_LENGTH_1 * tmp_p.x() / tmp_p.z() + COL / 2.0;
        tmp_p.y() = FOCAL_LENGTH_1 * tmp_p.y() / tmp_p.z() + ROW / 2.0;
        tmp_un_pts_1[i] = cv::Point2f(tmp_p.x(), tmp_p.y());

        liftProjective(Eigen::Vector2d(pts_now_1[i].x, pts_now_1[i].y), tmp_p); // Lifts a point from the image plane to its projective ray
        tmp_p.x() = FOCAL_LENGTH_1 * tmp_p.x() / tmp_p.z() + COL / 2.0;
        tmp_p.y() = FOCAL_LENGTH_1 * tmp_p.y() / tmp_p.z() + ROW / 2.0;
        tmp_un_pts_now[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
      }
      vector<uchar> status;
      // 通过寻找基础矩阵,找出特征点离群值，记录在status中，再做一次过滤
      // status: 含N个元素的输出向量, 0表示对应特征点为离群值，1则为正常特征点
      if (pts_now_1.size() >= 30)
      {
        cv::Mat essential_matrix;
        // cv::Mat homography_matrix;
        // fundamental_matrix = findFundamentalMat(points1_1, points2_1, cv::FM_8POINT); // 基础矩阵
        essential_matrix = findEssentialMat(pts_1_1, pts_now_1, K_1, LMEDS, 0.999, 1, status); // 计算本质矩阵
        // homography_matrix = findHomography(pts_1_1, pts_now_1, RANSAC, 3, status); // 计算单应矩阵   但是本例中场景不是平面，单应矩阵意义不大

        reduceVector(pts_1_1, status);
        reduceVector(pts_now_1, status);
        reduceVector(un_pts_now_1, status);
        reduceVector(ids_1, status);
      }
    }
    else // 2号相机
    {
      if (pts_now_2.size() >= 8)
      {
        vector<cv::Point2f> tmp_un_pts_1(pts_1_2.size()), tmp_un_pts_now(pts_now_2.size());
        for (unsigned int i = 0; i < pts_1_2.size(); i++)
        {
          Eigen::Vector3d tmp_p;
          liftProjective(Eigen::Vector2d(pts_1_2[i].x, pts_1_2[i].y), tmp_p); // Lifts a point from the image plane to its projective ray
          tmp_p.x() = FOCAL_LENGTH_2 * tmp_p.x() / tmp_p.z() + COL / 2.0;
          tmp_p.y() = FOCAL_LENGTH_2 * tmp_p.y() / tmp_p.z() + ROW / 2.0;
          tmp_un_pts_1[i] = cv::Point2f(tmp_p.x(), tmp_p.y());

          liftProjective(Eigen::Vector2d(pts_now_2[i].x, pts_now_2[i].y), tmp_p); // Lifts a point from the image plane to its projective ray
          tmp_p.x() = FOCAL_LENGTH_2 * tmp_p.x() / tmp_p.z() + COL / 2.0;
          tmp_p.y() = FOCAL_LENGTH_2 * tmp_p.y() / tmp_p.z() + ROW / 2.0;
          tmp_un_pts_now[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
        }
        vector<uchar> status;
        if (pts_now_2.size() >= 30)
        {
          cv::Mat essential_matrix;
          essential_matrix = findEssentialMat(pts_1_2, pts_now_2, K_2, LMEDS, 0.999, 1, status); // 计算本质矩阵
          reduceVector(pts_1_2, status);
          reduceVector(pts_now_2, status);
          // reduceVector(un_pts_1_2, status);
          reduceVector(un_pts_now_2, status);
          reduceVector(ids_2, status);
          // reduceVector(track_cnt_2, status);
        }
      }
    }
  }
}

// 设置焦点检测的mask，在mask=0处不检测角点
void img_process::setMask(bool which_cam)
{
  if (which_cam) // 1号相机
  {
    mask_1 = cv::Mat(ROW, COL, CV_8UC1, cv::Scalar(255));
    vector<pair<int, cv::Point2f>> cnt_pts_id; // cnt_pts_id 包含 track_cnt_1 特征点pts_now_1 特征点编号ids_1
    for (unsigned int i = 0; i < pts_now_1.size(); i++)
    {
      cnt_pts_id.emplace_back(make_pair(ids_1[i], pts_now_1[i]));
    }

    // 对cnt_pts_id中的元素按照其 ids 值降序排序
    sort(cnt_pts_id.begin(), cnt_pts_id.end(), [](const pair<int, cv::Point2f> &a, const pair<int, cv::Point2f> &b)
         { return a.first < b.first; }); // 改成升序排列
    pts_now_1.clear();
    ids_1.clear();
    // 排列后的点重新存入pts_now
    for (auto &it : cnt_pts_id)
    {
      pts_now_1.emplace_back(it.second);
      ids_1.emplace_back(it.first);
    }
    // 把已有的特征点从蒙版上删去，避免特征点挤成一坨
    for (const cv::Point2f &pt : pts_now_1)
    {
      // 计算圆心坐标
      int centerX = static_cast<int>(pt.x);
      int centerY = static_cast<int>(pt.y);
      // 遍历圆内的像素位置
      for (int dx = -MIN_DIST_half; dx <= MIN_DIST_half; ++dx)
      {
        for (int dy = -MIN_DIST_half; dy <= MIN_DIST_half; ++dy)
        {
          int x = centerX + dx; // 计算当前像素位置
          int y = centerY + dy;
          // 检查像素位置是否在蒙版边界内// 设置蒙版上的像素值为0
          if (x >= 0 && x < mask_1.cols && y >= 0 && y < mask_1.rows)
          {

            mask_1.at<uchar>(y, x) = 0;
          }
        }
      }
    }
  }
  else // 2号相机
  {
    mask_2 = cv::Mat(ROW, COL, CV_8UC1, cv::Scalar(255));

    vector<pair<int, cv::Point2f>> cnt_pts_id;

    for (unsigned int i = 0; i < pts_now_2.size(); i++)
    {
      // cnt_pts_id.emplace_back(make_pair(track_cnt_2[i], make_pair(pts_now_2[i], ids_2[i])));
      cnt_pts_id.emplace_back(make_pair(ids_2[i], pts_now_2[i]));
    }
    sort(cnt_pts_id.begin(), cnt_pts_id.end(), [](const pair<int, cv::Point2f> &a, const pair<int, cv::Point2f> &b)
         { return a.first < b.first; });
    pts_now_2.clear();
    ids_2.clear();
    // track_cnt_2.clear();
    for (auto &it : cnt_pts_id)
    {
      pts_now_2.emplace_back(it.second);
      ids_2.emplace_back(it.first);
      // track_cnt_2.emplace_back(it.first);
    }
    // 把已有的特征点从蒙版上删去，避免特征点挤成一坨
    for (const cv::Point2f &pt : pts_now_2)
    {
      int centerX = static_cast<int>(pt.x); // 计算圆心坐标
      int centerY = static_cast<int>(pt.y);
      // 遍历圆内的像素位置
      for (int dx = -MIN_DIST_half; dx <= MIN_DIST_half; ++dx)
      {
        for (int dy = -MIN_DIST_half; dy <= MIN_DIST_half; ++dy)
        {
          int x = centerX + dx; // 计算当前像素位置
          int y = centerY + dy;
          if (x >= 0 && x < mask_1.cols && y >= 0 && y < mask_1.rows) // 检查像素位置是否在蒙版边界内
          {
            mask_2.at<uchar>(y, x) = 0;
          }
        }
      }
    }
  }
}

// 将新产生的特征点n_pts 存入pts_now
void img_process::addPoints(bool which_cam)
{
  if (which_cam) // 1号相机
  {
    for (auto &p : n_pts_1)
    {
      pts_now_1.emplace_back(p);
      // 在ids_1 中找到一个未被使用的 ID给新特征点用
      unsigned int ids_length = ids_1.size();
      unsigned int n = 1;
      while (find(ids_1.cbegin(), ids_1.cend(), ids_length + n) != ids_1.cend() && ids_1.size() > 0)
      {
        n++;
      }
      ids_1.emplace_back(ids_length + n); // id直接在当前的ids长度上往上加
    }
    // ids_1.emplace_back(-1);
  }
  else // 2号相机
  {
    for (auto &p : n_pts_2)
    {
      pts_now_2.emplace_back(p);
      unsigned int ids_length = ids_2.size();
      unsigned int n = 1;
      while (find(ids_2.cbegin(), ids_2.cend(), ids_length + n) != ids_2.cend() && ids_2.size() > 0)
      {
        n++;
      }
      ids_2.emplace_back(ids_length + n); // id直接在当前的ids长度上往上加
      // track_cnt_2.emplace_back(1);
    }
    // ids_2.emplace_back(-1);
  }
}

// 计算Point2f容器的均值
inline cv::Point2f img_process::calculateMean(const std::vector<cv::Point2f> &vec)
{
  cv::Point2f sum = std::accumulate(vec.begin(), vec.end(), cv::Point2f(0, 0));
  return cv::Point2f(sum.x / vec.size(), sum.y / vec.size());
}

// 移除光流速度离群的点
void img_process::velocity_outlier_remove(bool which_cam)
{
  if (which_cam) // 1号相机
  {
    vector<uchar> status;
    status.resize(pts_velocity_1.size());
    cv::Point2f mean = calculateMean(pts_velocity_1);

    for (int i = 0; i < pts_velocity_1.size(); ++i)
    {
      float diff = cv::norm(pts_velocity_1[i] - mean); // 计算向量元素与平均值的差异
      if (diff <= VELOCITY_THRESHOD)
        status[i] = 1; // 将差异在阈值内的元素加入到筛选后的向量中
      else
        status[i] = 0;
    }
    reduceVector(pts_1_1, status);
    reduceVector(pts_now_1, status);
    reduceVector(un_pts_now_1, status);
    reduceVector(ids_1, status);
    reduceVector(pts_velocity_1, status);
  }
  else // 2号相机
  {
    vector<uchar> status;
    status.resize(pts_velocity_2.size());
    cv::Point2f mean = calculateMean(pts_velocity_2);
    for (int i = 0; i < pts_velocity_1.size(); ++i)
    {
      float diff = cv::norm(pts_velocity_2[i] - mean); // 计算向量元素与平均值的差异
      if (diff <= VELOCITY_THRESHOD)
        status[i] = 1; // 将差异在阈值内的元素加入到筛选后的向量中
      else
        status[i] = 0;
    }
    reduceVector(pts_1_1, status);
    reduceVector(pts_now_2, status);
    reduceVector(un_pts_now_2, status);
    reduceVector(ids_2, status);
    reduceVector(pts_velocity_2, status);
  }
}

// 根据像素坐标计算图像坐标和光流速度
void img_process::undistortedPoints(bool which_cam)
{
  if (which_cam) // 1号相机
  {
    un_pts_now_1.clear();
    un_pts_map_1_1.clear();
    cv::undistortPoints(pts_1_1, un_pts_now_1, K_1, distCoeffs_1);
    for (size_t i = 0; i < ids_1.size(); i++) // 存储本次的特征点ID及其归一化图像坐标，下次使用
      un_pts_map_1_1[ids_1[i]] = un_pts_now_1[i];
    // 计算光流点的运动速度
    if (!un_pts_map_0_1.empty())
    {
      double dt = img_time_now_1 - img_time_1_1;
      pts_velocity_1.clear();
      for (unsigned int i = 0; i < un_pts_now_1.size(); i++)
      {
        // if (i < ids_1.size())
        // {
        if (un_pts_map_0_1.find(ids_1[i]) != un_pts_map_0_1.end()) // 在un_pts_map_0中查找ids[i]
        {
          float vx = (un_pts_now_1[i].x - un_pts_map_0_1[ids_1[i]].x) / dt;
          float vy = (un_pts_now_1[i].y - un_pts_map_0_1[ids_1[i]].y) / dt;
          pts_velocity_1.emplace_back(cv::Point2f((vx, vy)));
        }
        else
          pts_velocity_1.emplace_back(cv::Point2f(0, 0));
        // }
        // else
        //   pts_velocity_1.emplace_back(cv::Point2f(0, 0));
      }
    }
    else
    {
      for (unsigned int i = 0; i < pts_1_1.size(); i++)
      {
        pts_velocity_1.emplace_back(cv::Point2f(0, 0));
      }
    }
    un_pts_map_0_1 = un_pts_map_1_1;
  }
  else // 2号相机
  {
    un_pts_now_2.clear();
    un_pts_map_1_2.clear();
    cv::undistortPoints(pts_now_2, un_pts_now_2, K_2, distCoeffs_2);
    for (size_t i = 0; i < ids_2.size(); i++)
      un_pts_map_1_2[ids_2[i]] = un_pts_now_2[i];
    // 计算光流点的运动速度
    if (!un_pts_map_0_2.empty())
    {
      double dt = img_time_now_2 - img_time_1_2;
      pts_velocity_2.clear();
      for (unsigned int i = 0; i < un_pts_now_2.size(); i++)
      {
        // if (i < ids_2.size())
        {
          if (un_pts_map_0_2.find(ids_2[i]) != un_pts_map_0_2.end()) // 在un_pts_map_0中查找ids[i] 若找到ids[i]
          {
            pts_velocity_2.emplace_back(cv::Point2f(
                (un_pts_now_2[i].x - un_pts_map_0_2[ids_2[i]].x) / dt, (un_pts_now_2[i].y - un_pts_map_0_2[ids_2[i]].y) / dt));
          }
          else
            pts_velocity_2.emplace_back(cv::Point2f(0, 0));
        }
        // else
        // {
        //   pts_velocity_2.emplace_back(cv::Point2f(0, 0));
        // }
      }
    }
    else
    {
      for (unsigned int i = 0; i < pts_1_2.size(); i++)
      {
        pts_velocity_2.emplace_back(cv::Point2f(0, 0));
      }
    }
    un_pts_map_0_2 = un_pts_map_1_2;
  }
}

void img_process::readImage_1()
{
  pts_now_1.clear();

  if (pts_1_1.size() > 0)
  {
    vector<uchar> status;
    vector<float> err;
    cv::calcOpticalFlowPyrLK(img_1_1, img_now_1, pts_1_1, pts_now_1, status, err, cv::Size(21, 21), 2);
    // 金字塔LK
    // 输入前一帧 输入后一帧 输入前一帧特征点 输出后一帧特征点pts_now 特征点状态status 错误向量err 金字塔每层的搜索窗尺寸 金字塔层数
    for (int i = 0; i < int(pts_now_1.size()); i++)
      if (status[i] && !inBorder(pts_now_1[i]))
        status[i] = 0; // 金字塔光流找到的特征点，如果已经跑出图像外，则不使用

    reduceVector(pts_1_1, status);
    reduceVector(pts_now_1, status);
    reduceVector(ids_1, status);
    reduceVector(un_pts_now_1, status);
  }

  if (PUB_FRAME_1)
  {
    rejectWithF(1); // 用相机焦距、光心修正图像特征点，得到校正后的特征点.用校正后的点计算基础矩阵，从而剔除离群值
    setMask(1);     // 开始设置mask,mask=0处不检测角点
    // 只有当根据金字塔光流产生的特征点数不足才会进入这里
    if (MAX_CNT - static_cast<int>(pts_now_1.size()) > 0)
    {
      if (mask_1.empty())
        ROS_WARN("mask 1 empty");
      if (mask_1.type() != CV_8UC1)
        ROS_WARN("mask 1 format error");
      if (mask_1.size() != img_now_1.size())
        cout << "mask 1 尺寸不匹配!mask尺寸" << mask_1.size() << ",img尺寸" << img_now_1.size() << endl;
      cv::goodFeaturesToTrack(img_now_1, n_pts_1, MAX_CNT - pts_now_1.size(), 0.01, MIN_DIST, mask_1);
      // 参数：输入图像;检测到的所有角点;用于限定检测到的点数的最大值;
      // 角点质量水平;区分相邻两个角点的最小距离;mask，维度须和输入图像一致，在mask值为0处不进行角点检测
      addPoints(1); // 将n_pts加入pts_now
    }
    else
      n_pts_1.clear();
  }
  img_1_1 = img_now_1.clone();
  pts_1_1 = pts_now_1;
  undistortedPoints(1);
  velocity_outlier_remove(1);

  // vector<float> diffs;
  // cv::Point2f mean = calculateMean(pts_velocity_1);
  // for (const auto &point : pts_velocity_1)
  // {
  //   float diff = cv::norm(point - mean);
  //   diffs.push_back(diff);
  // }
  // // 计算最大值和最小值
  // float max_diff = *max_element(diffs.begin(), diffs.end());
  // float min_diff = *min_element(diffs.begin(), diffs.end());
  // // 输出结果
  // cout << "Max_delta " << max_diff << "Min_delta " << min_diff << endl;
}

void img_process::readImage_2()
{
  pts_now_2.clear();

  if (pts_1_2.size() > 0)
  {
    vector<uchar> status;
    vector<float> err;
    cv::calcOpticalFlowPyrLK(img_1_2, img_now_2, pts_1_2, pts_now_2, status, err, cv::Size(21, 21), 3);
    // 金字塔LK
    for (int i = 0; i < int(pts_now_2.size()); i++)
      if (status[i] && !inBorder(pts_now_2[i]))
        status[i] = 0;
    reduceVector(pts_1_2, status);
    reduceVector(pts_now_2, status);
    reduceVector(ids_2, status);
    reduceVector(un_pts_now_2, status);
    // reduceVector(track_cnt_2, status);
  }

  if (PUB_FRAME_2)
  {
    rejectWithF(0);                       // 用相机焦距、光心修正图像特征点，得到校正后的特征点.用校正后的点计算基础矩阵，从而剔除离群值
    setMask(0);                           // 开始设置mask
    if (MAX_CNT - (pts_now_2.size()) > 0) // 只有当根据光流产生的特征点数不足才会进入这里
    {
      if (mask_2.empty())
        ROS_WARN("img_process::readImage_2 mask empty");
      if (mask_2.type() != CV_8UC1)
        ROS_WARN("img_process::readImage_2 mask format error");
      if (mask_2.size() != img_now_2.size())
        cout << "warning! mask与图像尺寸不匹配!当前mask尺寸" << mask_2.size() << ", 当前img尺寸" << img_now_2.size() << endl;
      cv::goodFeaturesToTrack(img_now_2, n_pts_2, MAX_CNT - pts_now_2.size(), 0.01, MIN_DIST, mask_2);
      addPoints(0); // 将n_pts加入pts_now
    }
    else
      n_pts_2.clear();
  }
  img_1_2 = img_now_2.clone();

  pts_1_2 = pts_now_2;
  undistortedPoints(0);
  velocity_outlier_remove(0);
}

inline void img_process::change_brightness(Mat &src) // 调整图像亮度 对数曲线
{
  src.convertTo(src, CV_32F);
  src = src + 1;     // 图像矩阵元素进行加1操作
  cv::log(src, src); // 图像对数操作
  src = 1.1 * src;
  normalize(src, src, 0, 255, NORM_MINMAX); // 图像归一化
  convertScaleAbs(src, src);
}

void img_process::apriltag_callback_cam1(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg) // 相机1 tag坐标 回调
{
  if (msg->detections.empty())
    return;
  const std::vector<apriltag_ros::AprilTagDetection> detections = msg->detections; // 接收 AprilTagDetection[] 类型的 detections
  int num_detections = detections.size();                                          // 获取 detections 数组的大小                                                       // 存储 id 和 pose
  geometry_msgs::PoseWithCovarianceStamped pose_temp;
  float depth;
  for (int i = 0; i < num_detections; i++) // 处理 detections 数据
  {
    pose_temp = detections[i].pose;
    depth = (pose_temp.pose.pose.position.z);
  }
  latest_tag_time = msg->header.stamp.toSec();
  pthread_mutex_lock(&mutex);
  depth_list_1 = depth; // 将depth存入1号相机的深度值中depth_list_1
  pthread_mutex_unlock(&mutex);
}

void img_process::imageCallback_1(const sensor_msgs::ImageConstPtr &msg)
{
  try
  {
    if (laser.laser_on) // 若灯开，则把图片存入img_laser_1
    {
      img_laser_1 = cv_bridge::toCvShare(msg, "bgr8")->image;
      std::vector<cv::Mat> channels;
      cv::split(img_laser_1, channels); // 拆分颜色通道
      img_laser_1 = channels[2];        // 提取红色通道
      if (EQUALIZE)                     // 图像局部自适应均衡化
      {
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        clahe->apply(img_laser_1, img_laser_1);
      }
    }
    else if (OPTICAL_DELAY_FLAG) // 若没开灯，则计算光流
    {
      // auto start = std::chrono::high_resolution_clock::now();//计时
      img_now_1 = cv_bridge::toCvShare(msg, "bgr8")->image;
      cv::cvtColor(img_now_1, img_now_1, CV_RGB2GRAY);
      if (BRIGHTNESS)
        change_brightness(img_now_1);

      cv::Scalar fontColor(0, 0, 255);                    // BGR颜色
      for (unsigned int j = 0; j < pts_now_1.size(); j++) // 画上一帧之特征点
      {
        cv::circle(img_now_1, pts_now_1[j], 2, cv::Scalar(100, 0, 100, 255), 2);
        cv::putText(img_now_1, std::to_string(ids_1[j]), pts_now_1[j], cv::FONT_HERSHEY_SIMPLEX, 0.4, fontColor);
      }
      imshow("img_now_1", img_now_1);

      img_time_now_1 = msg->header.stamp.toSec();
      if (if_first_1)
      {
        img_time_1_1 = img_time_now_1;
        img_1_1 = img_now_1.clone();
      }
      else
      {
        if (img_time_now_1 - img_time_1_1 > 0.2 || img_time_now_1 < img_time_1_1) // 如果当前帧与上一帧时间差超过0.5s，或者当前帧早于上一帧,则重启图像模块
        {
          ROS_WARN("cam1 is Disordered !!!");
          PUB_FRAME_1 = 0;
        }
        else
          PUB_FRAME_1 = 1;
        if (EQUALIZE) // 图像局部自适应均衡化
        {
          cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
          clahe->apply(img_now_1, img_now_1);
        }

        if (!img_1_1.empty() && !img_now_1.empty()) // 读图 求光流
        {
          readImage_1(); // 读相机数据，和特征点
        }
        else
        {
          ROS_WARN("cam1 is Null !!!");
          return;
        }
        img_time_1_1 = img_time_now_1;

        // auto end_process = std::chrono::high_resolution_clock::now();//计时

        // 发布点云消息
        sensor_msgs::PointCloudPtr feature_points(new sensor_msgs::PointCloud);
        sensor_msgs::ChannelFloat32 id_of_point;
        sensor_msgs::ChannelFloat32 u_of_point;
        sensor_msgs::ChannelFloat32 v_of_point;
        sensor_msgs::ChannelFloat32 velocity_x_of_point;
        sensor_msgs::ChannelFloat32 velocity_y_of_point;
        sensor_msgs::ChannelFloat32 which_cam; // 1表示cam1 2表示cam2

        feature_points->header = msg->header;
        feature_points->header.frame_id = "cam";
        // 为了保证cam2的特征点编号不会与cam1重复，需要找出cam1特征点编号最大值，作为偏置量加到Cam2编号上
        //  1号相机
        for (unsigned int j = 0; j < ids_1.size(); j++)
        {
          geometry_msgs::Point32 p;
          laser_depth[0] = laser_depth[0] || 1; // 如果深度值为0则赋值为1
          p.x = un_pts_now_1[j].x;              // 特征点坐标
          p.y = un_pts_now_1[j].y;              // 特征点坐标
          p.z = 1;
          feature_points->points.emplace_back(p);
          id_of_point.values.emplace_back(ids_1[j]);
          u_of_point.values.emplace_back(pts_now_1[j].x);
          v_of_point.values.emplace_back(pts_now_1[j].y);
          velocity_x_of_point.values.emplace_back(pts_velocity_1[j].x);
          velocity_y_of_point.values.emplace_back(pts_velocity_1[j].y);
          which_cam.values.emplace_back(1);
        }
        // 2号相机
        for (unsigned int j = 0; j < ids_2.size(); j++)
        {
          // if (track_cnt_2[j] > 1)
          {
            geometry_msgs::Point32 p;
            laser_depth[1] = laser_depth[1] || 1; // 如果深度值为0则赋值为1
            p.x = -1 + 0.02095;                   // 特征点坐标    加了cam2向cam1的坐标变换 单位m
            p.y = un_pts_now_2[j].y;              // 特征点坐标
            p.z = un_pts_now_2[j].x + 0.02905;    // 这里深度坐标用最新laser计算的深度

            feature_points->points.emplace_back(p);
            id_of_point.values.emplace_back(ids_2[j] + MAX_CNT);
            u_of_point.values.emplace_back(pts_now_2[j].x);
            v_of_point.values.emplace_back(pts_now_2[j].y);
            velocity_x_of_point.values.emplace_back(pts_velocity_2[j].x);
            velocity_y_of_point.values.emplace_back(pts_velocity_2[j].y);
            which_cam.values.emplace_back(0);
          }
        }
        feature_points->channels.emplace_back(id_of_point);
        feature_points->channels.emplace_back(u_of_point);
        feature_points->channels.emplace_back(v_of_point);
        feature_points->channels.emplace_back(velocity_x_of_point);
        feature_points->channels.emplace_back(velocity_y_of_point);
        feature_points->channels.emplace_back(which_cam);

        // auto end_all = std::chrono::high_resolution_clock::now();//计时
        // single_time_process = double(std::chrono::duration_cast<std::chrono::milliseconds>(end_process - start).count());
        // single_time_all = double(std::chrono::duration_cast<std::chrono::milliseconds>(end_all - start).count());
        // std::cout<<"耗时"<<single_time_process<<"    "<<single_time_all<<std::endl;

        if (!(init_pub_1))
          init_pub_1 = 1;
        else
          img_pub.publish(feature_points); // 发布特征点消息"feature"
      }
      if_first_1 = 0;
    }
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("cam1: cannnot get latest frame");
  }
}

void img_process::imageCallback_2(const sensor_msgs::ImageConstPtr &msg)
{
  try
  {
    if (laser.laser_on) // 若灯开，则把图片存入img_laser_1
    {
      img_laser_2 = cv_bridge::toCvShare(msg, "bgr8")->image;
      std::vector<cv::Mat> channels;
      cv::split(img_laser_2, channels); // 拆分颜色通道
      img_laser_2 = channels[2];        // 红色通道
      if (EQUALIZE)                     // 图像局部自适应均衡化
      {
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        clahe->apply(img_laser_2, img_laser_2);
      }
    }
    else if (OPTICAL_DELAY_FLAG) // 若没开灯，则计算光流
    {
      img_now_2 = cv_bridge::toCvShare(msg, "bgr8")->image;
      cv::cvtColor(img_now_2, img_now_2, CV_RGB2GRAY);
      if (BRIGHTNESS)
        change_brightness(img_now_2);
      cv::Scalar fontColor(0, 0, 255); // BGR颜色
      for (unsigned int j = 0; j < pts_now_2.size(); j++)
      {
        cv::circle(img_now_2, pts_now_2[j], 2, cv::Scalar(100, 0, 100, 255), 2);
        cv::putText(img_now_2, std::to_string(ids_2[j]), pts_now_2[j], cv::FONT_HERSHEY_SIMPLEX, 0.4, fontColor);
      }
      imshow("img_now_2", img_now_2);

      img_time_now_2 = msg->header.stamp.toSec();
      if (if_first_2)
      {
        img_time_1_2 = img_time_now_2;
        img_1_2 = img_now_2.clone();
      }
      else
      {
        if ((img_time_now_2 - img_time_1_2 > 0.2) || (img_time_now_2 < img_time_1_2)) // 如果当前帧与上一帧时间差超过0.5s，或者当前帧早于上一帧,则重启图像模块
        {
          ROS_WARN("cam2 is Disordered !!!");
          PUB_FRAME_2 = 0;
        }
        else
        {
          PUB_FRAME_2 = 1;
        }

        if (EQUALIZE) // 图像局部自适应均衡化
        {
          cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
          clahe->apply(img_now_2, img_now_2);
        }

        if (!img_1_2.empty() && !img_now_2.empty())
        {
          readImage_2(); // 读相机数据，和特征点
        }
        else
        {
          ROS_WARN("cam2 is Null !!!");
          return;
        }
        img_time_1_2 = img_time_now_2;
      }
      if_first_2 = 0;
    }
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("cam1: cannnot get latest frame");
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");

  img_process img_process_handle_cam;
  return 1;
}
