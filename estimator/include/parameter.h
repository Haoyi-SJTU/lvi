#pragma once

// #include <ros/ros.h>
#include <vector>
#include <eigen3/Eigen/Dense>
// #include "utility/utility.h"
// #include <opencv2/opencv.hpp>
// #include <opencv2/core/eigen.hpp>
// #include <fstream>

// #define EIGEN_USE_MKL_ALL //开启mkl优化：在使用Intel处理器并且已经安装好mkl库的情况下，可通过宏开启Eigen3的mkl优化
// #define EIGEN_NO_DEBUG //关闭Debug模式：在确保程序无误的情况下，关闭Debug断言，可以提升程序效率

const unsigned int WINDOW_SIZE = 10;// 滑动窗长度
