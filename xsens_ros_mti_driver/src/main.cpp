#include <ros/ros.h>
#include "xdainterface.h"

#include <iostream>
#include <stdexcept>
#include <string>

using std::chrono::milliseconds;// 定义以ms为单位的时间段变量

Journaller *gJournal = 0;

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "xsens_driver");
	ros::NodeHandle node;

	XdaInterface *xdaInterface = new XdaInterface();
	xdaInterface->registerPublishers(node);//注册多种消息发布器
	if (!xdaInterface->connectDevice())//获取传感器ID
		return -1;
	if (!xdaInterface->prepare())//传感器config，进入测量模式
		return -1;

	while (ros::ok())
	{
		xdaInterface->spinFor(milliseconds(100));//设置timeout为100ms
		// 取回缓存中最早的数据包
		ros::spinOnce();
	}

	delete xdaInterface;
	return 0;
}
