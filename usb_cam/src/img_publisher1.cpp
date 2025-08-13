#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>
#include <iostream>

using namespace cv;

// 修改图像压缩程度以提高消息发布频率：
// rosrun rqt_reconfigure rqt_reconfigure

int main(int argc, char **argv)
{
	ros::init(argc, argv, "img_publisher1");
	ros::NodeHandle nh;

	// Apriltag需要相机内参和畸变系数
	sensor_msgs::CameraInfo cam_info_msg;
	cam_info_msg.header.frame_id = "usb_camera_frame";
	cam_info_msg.height = 480;
	cam_info_msg.width = 640;
	cam_info_msg.distortion_model = "plumb_bob";
	cam_info_msg.D.resize(5);
	cam_info_msg.D[0] = 0.056723;
	cam_info_msg.D[1] = -0.141970;
	cam_info_msg.D[2] = -0.000390;
	cam_info_msg.D[3] = -0.000620;
	cam_info_msg.D[4] = 0;
	cam_info_msg.K = {614.668562, 0.0, 313.603877, 0.0, 614.827375, 279.414349, 0.0, 0.0, 1.0};
	cam_info_msg.R = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
	cam_info_msg.P = {400.0, 0.0, 320.0, 0.0, 0.0, 400.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0};
	ros::Publisher cam_info_pub = nh.advertise<sensor_msgs::CameraInfo>("camera1/camera_info", 1); // 创建发布器

	image_transport::ImageTransport it(nh);
	image_transport::Publisher pub = it.advertise("camera1/image", 1);

	cv::VideoCapture capture;
	// capture.set(cv::CAP_PROP_BUFFERSIZE, 38);
	// capture.set(cv::CAP_PROP_POS_FRAMES, 1);
	// std::cout << "WIDTH:" << capture.set(cv::CAP_PROP_FRAME_WIDTH, 5000) << std::endl;
	// std::cout << "HEIGHT:" << capture.set(cv::CAP_PROP_FRAME_HEIGHT, 5000) << std::endl;
	capture.set(cv::CAP_PROP_FPS, 60);
	// std::cout << "fourcc:" << capture.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G')) << std::endl;
	capture.open("/dev/video0", cv::CAP_V4L2);
	// 为使用V4L控件指定V4L捕获后端。在没有指定的情况下，使用gstreamer后端，它将pipeline视频流实例化为v4l2src，但创建后无法更改分辨率。
	// 因此，最简单的解决方案就是使用V4L2捕获后端 cv::CAP_V4L2
	// std::cout << "图像宽=" << capture.get(cv::CAP_PROP_FRAME_WIDTH) << ",高=" << capture.get(cv::CAP_PROP_FRAME_HEIGHT) << std::endl;
	// std::cout << "帧率=" << capture.get(cv::CAP_PROP_FPS) << std::endl;
	if (!capture.isOpened())
	{
		ROS_ERROR("can`t open cam1!");
		return 0;
	}

	Mat Img;
	ros::Rate loop_rate(60);
	while (nh.ok())
	{
		capture >> Img;
		if (!Img.empty())
		{
			sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", Img).toImageMsg();
			msg->header.stamp = ros::Time::now();

			cam_info_msg.header.stamp = msg->header.stamp; 
			cam_info_pub.publish(cam_info_msg);
			pub.publish(msg);
		}
		ros::spinOnce();
		// loop_rate.sleep();
	}
	return 0;
}
