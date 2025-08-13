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


// 用在图像模块 打印光流速度的与均值的离群差距
void print_delta(std::vector<cv::Point2f> pts_velocity_1)
{
  vector<float> diffs;
  cv::Point2f mean = calculateMean(pts_velocity_1);
  for (const auto &point : pts_velocity_1)
  {
    float diff = cv::norm(point - mean);
    diffs.push_back(diff);
  }
  // 计算最大值和最小值
  float max_diff = *max_element(diffs.begin(), diffs.end());
  float min_diff = *min_element(diffs.begin(), diffs.end());
  // 输出结果
  cout << "Max_delta " << max_diff << "Min_delta " << min_diff << endl;
}
