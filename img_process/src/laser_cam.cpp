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
#include <mutex>
#include "laser_cam.hpp"

using namespace cv;

laser_cam::laser_cam()
{
    HEIGHT = 480; // 图像尺寸
    WIDTH = 640;
    h_1 = 0.27;
    h_2 = 0.26;
    ROI_HEIGHT_min = HEIGHT * 0.45;
    ROI_HEIGHT_max = HEIGHT * 0.7;
    ROI_WIDTH_min = 5;
    WIDTH * 0.01;
    ROI_WIDTH_max = WIDTH * 0.95; // 圆拟合ROI
    r_min = 5;                    // 圆拟合阈值
    r_max = 50;                   // 圆拟合阈值
    which_cam = 1;                // 1:1号相机; 2:2号相机
    laser_on = 0;
    previous_laser_state = 0;
    previous_circle_result_1 = 160; // 中间值作为初值以完成第一次循环
    previous_circle_result_2 = 160; // 中间值作为初值以完成第一次循环
    // laser_thread_ready_FLAG = 0;
    img_counter = 0;

    fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1)
        ROS_WARN("cannot read ttyUSB0!");

    tcgetattr(fd, &SerialPortSettings);
    cfsetispeed(&SerialPortSettings, B9600); // 设置波特率
    cfsetospeed(&SerialPortSettings, B9600);
    SerialPortSettings.c_cflag &= ~PARENB; // 设置没有校验
    SerialPortSettings.c_cflag &= ~CSTOPB; // 停止位 = 1
    SerialPortSettings.c_cflag &= ~CSIZE;
    SerialPortSettings.c_cflag |= CS8; // 设置数据位 = 8
    SerialPortSettings.c_cflag &= ~CRTSCTS;
    SerialPortSettings.c_cflag |= CREAD | CLOCAL;
    SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);         // 关闭软件流动控制
    SerialPortSettings.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG); // 设置操作模式
    SerialPortSettings.c_oflag &= ~OPOST;
    if ((tcsetattr(fd, TCSANOW, &SerialPortSettings)) != 0)
        ROS_WARN("串口设置失败!");

    laser_thread = std::thread(&laser_cam::laser_switch, this); // 激光控制 子线程
    laser_thread.detach();

    cv::namedWindow("1号相机", cv::WINDOW_NORMAL); // 1号相机
    cv::namedWindow("2号相机", cv::WINDOW_NORMAL); // 2号相机
    // cv::namedWindow("dst 1", cv::WINDOW_NORMAL); // 1号相机
    // cv::namedWindow("dst 2", cv::WINDOW_NORMAL); // 2号相机
    // cv::startWindowThread();
    ROS_INFO("laser_cam initialization finished.");
}

laser_cam::~laser_cam()
{
    laser_on = 0;
    cv::destroyAllWindows();
    close(fd);
}

// 通过串口打开激光灯
bool laser_cam::laser_switch()
{
    bool write_result;
    while (1)
    {
        printf(""); // 怪哉! 加了这一行就能运行了
        if (previous_laser_state != laser_on)
        {
            if (laser_on)
                write_result = write(fd, turn_on, sizeof(turn_on)); // 串口写数据
            else
                write_result = write(fd, turn_off, sizeof(turn_off)); // 串口写数据
            previous_laser_state = laser_on;
        }
    }
}

// 强假设：激光点坐标不会突然改变
// 可以给坐标数据加一个均值滤波，用最近几个坐标的数据均值大致划一个上下界限从而直接忽略离群值
bool laser_cam::find_laser_circle(cv::Mat &src, Vec3f &result)
{
    Mat dst;
    std::vector<Vec3f> circles;
    if (src.empty())
    {
        img_counter--;
        return 0;
    }
    dst = src.clone();
    dst = dst(Range(ROI_HEIGHT_min, ROI_HEIGHT_max), Range(ROI_WIDTH_min, ROI_WIDTH_max));
    threshold(dst, dst, 240, 255, THRESH_BINARY_INV);
    medianBlur(dst, dst, 5); // 中值滤波:霍夫圆检测对噪声敏感
    // imshow("dst 1", dst);
    if (!dst.empty() && dst.type() == CV_8UC1)
    {
        HoughCircles(dst, circles, HOUGH_GRADIENT, 2, 5, 80, 50, r_min, r_max);
    }
    else
        return 0;
    float x, y, r;
    unsigned int i;
    for (i = 0; i < circles.size() && i < 3; i++)
    {
        x += circles[i][0];
        y += circles[i][1];
        r += circles[i][2];
    }
    if (i)
    {
        x = x / i + ROI_WIDTH_min;
        y = y / i + ROI_HEIGHT_min;
        r = r / i;
        if (x <= ROI_HEIGHT_max && y <= ROI_HEIGHT_max && r <= r_max)
            result = Vec3f(x, y, r);
    }
    else
    {
        result = Vec3f(0, 0, 0);
    }
    return 1;
}

// 根据像素点坐标查表、插值，得到单点深度
inline float laser_cam::guess_h(const float u, bool which_cam)
{
    if (which_cam) // 相机1
    {
        return 10.7030804577 / (u - 44.12582616367665); // 根据相机1的标定结果返回h 单位m
    }
    else // 相机2
    {
        return 10.364707549 / (u - 6.994309429542132); // 根据相机2的标定结果返回h 单位m
    }
}

bool laser_cam::draw_circle(bool which_cam)
{
    if (which_cam) // 相机1
    {
        circle(src_1, Point(circle_result_1[0], circle_result_1[1]), circle_result_1[2], Scalar(100, 0, 0, 255), int(circle_result_1[2] / 2));
        imshow("1号相机", src_1);
        return 1;
    }
    else // 相机2
    {
        circle(src_2, Point(circle_result_2[0], circle_result_2[1]), circle_result_2[2], Scalar(100, 0, 0, 255), int(circle_result_2[2] / 2));
        imshow("2号相机", src_2);
        return 1;
    }
}

bool laser_cam::run_once(Mat &Img_1, Mat &Img_2, Vec2f &depth)
{
    src_1 = Img_1.clone();
    src_2 = Img_2.clone();

    laser_on = 0; // 关闭laser
    if (img_counter % 2 == 0)
    {
        if (!find_laser_circle(src_1, circle_result_1)) // 读取1号相机圆心坐标
            ROS_WARN("1号相机 laser读入图像为空");
    }
    else
    {
        if (!find_laser_circle(src_2, circle_result_2)) // 读取1号相机圆心坐标
            ROS_WARN("2号相机 laser读入图像为空");
    }
    img_counter++;

    if (circle_result_1[0] > 0 && circle_result_1[0] <= 400 && circle_result_1[2] <= r_max && abs(circle_result_1[0] - previous_circle_result_1) <= 200)
    {
        previous_circle_result_1 = circle_result_1[0];
        h_1 = guess_h(circle_result_1[0], 1); // 计算1号相机物理深度
        draw_circle(1);
    }

    if (circle_result_2[0] > 0 && circle_result_2[0] <= 400 && circle_result_2[2] <= r_max && abs(circle_result_2[0] - previous_circle_result_2) <= 200)
    {
        previous_circle_result_2 = circle_result_2[0];
        h_2 = guess_h(circle_result_2[0], 0); // 计算2号相机物理深度
        draw_circle(0);
    }

    depth = Vec2f(h_1, h_2);
    if (h_1 || h_2)
        return 1;
    else
        return 0;
}
