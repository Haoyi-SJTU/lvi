#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>

using namespace cv;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "img_publisher2");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("camera2/image", 1);

  cv::VideoCapture capture;
	capture.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
	// capture.set(cv::CAP_PROP_BUFFERSIZE, 38);
	// capture.set(cv::CAP_PROP_POS_FRAMES, 1);
	// capture.set(cv::CAP_PROP_FRAME_WIDTH, 1280);	
	// capture.set(cv::CAP_PROP_FRAME_HEIGHT, 960); 
	capture.set(cv::CAP_PROP_FPS, 60);
	// capture.set(cv::CAP_PROP_EXPOSURE, 0.02); //设置曝光时间
	capture.open("/dev/video2", cv::CAP_V4L2);
	// 为使用V4L控件指定V4L捕获后端。在没有指定的情况下，使用gstreamer后端，它将pipeline视频流实例化为v4l2src，但创建后无法更改分辨率。
	// 因此，最简单的解决方案就是使用V4L2捕获后端 cv::CAP_V4L2
	// std::cout << "图像宽=" << capture.get(cv::CAP_PROP_FRAME_WIDTH) << ",高=" << capture.get(cv::CAP_PROP_FRAME_HEIGHT) << std::endl;
	if (!capture.isOpened())
	{
		ROS_ERROR("can`t open cam2!");
		return 0;
	}

	Mat Img;
	// capture.read(Img);
	// capture >> Img;

  ros::Rate loop_rate(60);
  while (nh.ok())
  {
    capture >> Img;
    // if (!Img.empty())
    {
      sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", Img).toImageMsg();
	  msg->header.stamp=ros::Time::now();
      pub.publish(msg);
    }
    ros::spinOnce();
    // loop_rate.sleep();
  }
  return 0;
}
