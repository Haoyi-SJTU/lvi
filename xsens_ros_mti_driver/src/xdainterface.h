#ifndef XDAINTERFACE_H
#define XDAINTERFACE_H

#include <ros/ros.h>
#include "xdacallback.h"
#include <xstypes/xsportinfo.h>
#include "chrono"

struct XsControl;
struct XsDevice;


class PacketCallback;

class XdaInterface
{
public:
	XdaInterface();
	~XdaInterface();

	void spinFor(std::chrono::milliseconds timeout);// 取回缓存中最早的数据包
	void registerPublishers(ros::NodeHandle &node);// 根据get到的参数注册ROS消息发布器

	bool connectDevice();// 获取传感器ID
	bool prepare();// 传感器config，进入测量模式
	void close();

private:
	void registerCallback(PacketCallback *cb);//调用XsControl的push_back函数
	bool handleError(std::string error);//打印错误

	XsControl *m_control;
	XsDevice *m_device;
	XsPortInfo m_port;
	XdaCallback m_xdaCallback;
	std::list<PacketCallback *> m_callbacks;
};

#endif
