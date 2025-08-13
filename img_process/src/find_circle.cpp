#include <iostream>
#include <opencv2/opencv.hpp>
// 串口
#include <stdio.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#include <deque>
#include <fstream>

using namespace cv;
using namespace std;

unsigned int HEIGHT = 480; // 图像尺寸
unsigned int WIDTH = 640;
unsigned int r_min = 3;
unsigned int r_max = 70;
VideoCapture capture;
Mat dst;
Mat src; // 相机图像
std::vector<Vec3f> circles;
Vec3f circle_result;

// 承认一个强假设：激光点坐标不会突然改变
// 可以给坐标数据加一个均值滤波，用最近几个坐标的数据均值大致划一个上下界限从而直接忽略离群值
Vec3f find_laser_circle(Mat src)
{

    if (src.empty())
    {
        printf("图像为空 \n");
        return Vec3f(0, 0, 0);
    }
    dst = src.clone();
    dst = dst(Range(HEIGHT * 0.45, HEIGHT* 0.7), Range(WIDTH * 0.05, WIDTH * 0.95));
    threshold(dst, dst, 230, 255, THRESH_BINARY_INV);
    medianBlur(dst, dst, 3); // 中值滤波:霍夫圆检测对噪声敏感
    HoughCircles(dst, circles, HOUGH_GRADIENT, 2, 5, 80, 50, r_min, r_max);
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
        x = x / i + WIDTH * 0.05;
        y = y / i + HEIGHT * 0.45;
        r = r / i;
        // if (x <= WIDTH && y <= HEIGHT && r <= r_max)
        // if (1)
            return Vec3f(x, y, r);
        // else
        //     return Vec3f(0, 0, 0);
    }
    else
        return Vec3f(0, 0, 0);
}

double calculateAverage(const std::deque<int> &myDeque)
{
    double sum = 0.0;
    for (const auto &num : myDeque)
    {
        sum += num;
    }

    return sum / static_cast<double>(myDeque.size());
}

int main()
{

    // 1号相机
    capture.set(cv::CAP_PROP_FPS, 30);
    capture.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    capture.open("/dev/video0", cv::CAP_V4L2);
    if (!capture.isOpened())
    {
        printf("\n 错误! 妹打开1号相机！ \n");
    }
    cv::namedWindow("原图", cv::WINDOW_NORMAL);
    // cv::startWindowThread();
    int imageCount = 0;
    std::deque<int> DequeX;
    std::deque<int> DequeY;
    std::ofstream outFileNum;
    outFileNum.open("num.txt", std::ios::out); // 打开/创建文本文件
    std::ofstream outFileX;
    outFileX.open("x.txt", std::ios::out); // 打开/创建文本文件
    std::ofstream outFileY;
    outFileY.open("y.txt", std::ios::out); // 打开/创建文本文件
    // std::ofstream outFileR("r.txt"); // 打开/创建文本文件

    while (1)
    {
        capture >> src;
        cvtColor(src, src, COLOR_RGB2GRAY);
        circle_result = find_laser_circle(src);
        if (circle_result[2] > 0)
        {
            circle(src, Point(circle_result[0], circle_result[1]), circle_result[2], Scalar(100, 0, 0, 255), int(circle_result[2] / 2));

            DequeX.push_back(circle_result[0]);
            DequeY.push_back(circle_result[1]);
        }
        imshow("原图", src);

        if (DequeX.size() < 10)
            continue;
        else
        {
            std::cout << "圆心:( " << circle_result[0] << ", " << circle_result[1] << ")" << std::endl; //<< circle_result[2] << std::endl;
            DequeX.pop_front();
            DequeY.pop_front();
        }

        int key = cv::waitKey(50);   // 等待1毫秒
        if (key == 115 || key == 83) // 如果按下了"S"键
        {
            std::stringstream ss;
            imageCount++;
            ss << "cam1_" << imageCount << ".jpg"; // 构造文件名
            std::string filename = ss.str();
            cv::imwrite(filename, src); // 保存图片
            std::cout << "保存图片成功" << std::endl;
            outFileNum << imageCount << std::endl;
            for (const auto &num1 : DequeX)
            {
                outFileX << num1 << "\t";
            }
            for (const auto &num2 : DequeY)
            {
                outFileY << num2 << "\t";
            }
            outFileX << std::endl; // 将值写入文件
            outFileY << std::endl; // 将值写入文件
            // outFileX << calculateAverage(DequeX) << std::endl; // 将值写入文件
            // outFileY << calculateAverage(DequeY) << std::endl; // 将值写入文件
            DequeX.clear();
            DequeY.clear();
            // break; // 结束循环
        }

        if (key == 'q' || key == 'Q') // 如果按下了"S"键
            break;
    }
    outFileNum.close(); // 关闭文件
    outFileX.close();
    outFileY.close();
    return 1;
}
