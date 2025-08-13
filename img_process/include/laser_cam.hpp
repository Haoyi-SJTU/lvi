#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>
// 串口
#include <stdio.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
// 线程管理
#include <thread>
#include <condition_variable>

using namespace cv;

unsigned char turn_on[4] = {0xA0, 0x01, 0x01, 0xA2};
unsigned char turn_off[4] = {0xA0, 0x01, 0x00, 0xA1};

class laser_cam
{
private:
    unsigned int HEIGHT;               // 图像高度
    unsigned int WIDTH;                // 图像宽度
    unsigned int r_min;                // 识别圆最小半径
    unsigned int r_max;                // 识别圆最大半径
    unsigned int ROI_HEIGHT_min;       // 圆提取ROI范围
    unsigned int ROI_HEIGHT_max;       // 圆提取ROI范围
    unsigned int ROI_WIDTH_min;        // 圆提取ROI范围
    unsigned int ROI_WIDTH_max;        // 圆提取ROI范围
    int fd;                            // 文件描述符
    struct termios SerialPortSettings; // 串口设置
    bool which_cam;                    // 1表示1号相机，0表示2号相机
    bool previous_laser_state;         // 记录前一个激光开关状态: 1开0关

    // 1号相机
    Mat src_1;                      // 相机图像
    Vec3f circle_result_1;          // 结果
    float previous_circle_result_1; // 上一次圆心的像素坐标 横坐标
    float h_1;                      // 物理深度
    // 2号相机
    Mat src_2;                      // 相机图像
    Vec3f circle_result_2;          // 结果
    float previous_circle_result_2; // 上一次圆心的像素坐标 横坐标
    float h_2;                      // 物理深度

    std::thread laser_thread; // 循环等待标志符开灯关灯

    bool find_laser_circle(Mat &, Vec3f &);  // 找圆
    inline float guess_h(const float, bool); // 根据u猜深度h

public:
    bool laser_on;            // 1打开激光，0关闭激光
    unsigned int img_counter; // 记录从上一次预积分结束到现在有多少帧

    laser_cam();
    ~laser_cam();
    bool laser_switch();                  // 开灯
    bool draw_circle(bool);               // 画圆
    bool run_once(Mat &, Mat &, Vec2f &); // 运行一次
};