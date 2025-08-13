#include "xdainterface.h"

#include <xscontroller/xsscanner.h>
#include <xscontroller/xscontrol_def.h>
#include <xscontroller/xsdevice_def.h>

#include "messagepublishers/packetcallback.h"
#include "messagepublishers/accelerationpublisher.h"
#include "messagepublishers/angularvelocitypublisher.h"
#include "messagepublishers/freeaccelerationpublisher.h"
#include "messagepublishers/gnsspublisher.h"
#include "messagepublishers/imupublisher.h"
#include "messagepublishers/magneticfieldpublisher.h"
#include "messagepublishers/orientationincrementspublisher.h"
#include "messagepublishers/orientationpublisher.h"
#include "messagepublishers/pressurepublisher.h"
#include "messagepublishers/temperaturepublisher.h"
#include "messagepublishers/timereferencepublisher.h"
#include "messagepublishers/transformpublisher.h"
#include "messagepublishers/twistpublisher.h"
#include "messagepublishers/velocityincrementpublisher.h"
#include "messagepublishers/positionllapublisher.h"
#include "messagepublishers/velocitypublisher.h"


// 类的初始化函数，创建Xsens控制对象
XdaInterface::XdaInterface() : m_device(nullptr)
{
	m_control = XsControl::construct();
	assert(m_control != 0);// 当控制对象为0时，触发assert预处理宏报警，中断程序
}

// 释放Xsens控制对象
XdaInterface::~XdaInterface()
{
	close();
	m_control->destruct();
}

// 取回缓存中最早的数据包
void XdaInterface::spinFor(std::chrono::milliseconds timeout)
{
	RosXsDataPacket rosPacket = m_xdaCallback.next(timeout);//取回缓存中最早的数据包
	if (!rosPacket.second.empty())
	{
		for (auto &cb : m_callbacks)//不太清楚这个循环在干什么
		{
			cb->operator()(rosPacket.second, rosPacket.first);
		}
	}
}

// 根据get到的参数注册ROS消息发布器
void XdaInterface::registerPublishers(ros::NodeHandle &node)
{
	bool should_publish;

	if (ros::param::get("~pub_imu", should_publish) && should_publish)
	{
		registerCallback(new ImuPublisher(node));
	}
	if (ros::param::get("~pub_quaternion", should_publish) && should_publish)
	{
		registerCallback(new OrientationPublisher(node));
	}
	if (ros::param::get("~pub_acceleration", should_publish) && should_publish)
	{
		registerCallback(new AccelerationPublisher(node));
	}
	if (ros::param::get("~pub_angular_velocity", should_publish) && should_publish)
	{
		registerCallback(new AngularVelocityPublisher(node));
	}
	if (ros::param::get("~pub_mag", should_publish) && should_publish)
	{
		registerCallback(new MagneticFieldPublisher(node));
	}
	if (ros::param::get("~pub_dq", should_publish) && should_publish)
	{
		registerCallback(new OrientationIncrementsPublisher(node));
	}
	if (ros::param::get("~pub_dv", should_publish) && should_publish)
	{
		registerCallback(new VelocityIncrementPublisher(node));
	}
	if (ros::param::get("~pub_sampletime", should_publish) && should_publish)
	{
		registerCallback(new TimeReferencePublisher(node));
	}
	if (ros::param::get("~pub_temperature", should_publish) && should_publish)
	{
		registerCallback(new TemperaturePublisher(node));
	}
	if (ros::param::get("~pub_pressure", should_publish) && should_publish)
	{
		registerCallback(new PressurePublisher(node));
	}
	if (ros::param::get("~pub_gnss", should_publish) && should_publish)
	{
		registerCallback(new GnssPublisher(node));
	}
	if (ros::param::get("~pub_twist", should_publish) && should_publish)
	{
		registerCallback(new TwistPublisher(node));
	}
	if (ros::param::get("~pub_free_acceleration", should_publish) && should_publish)
	{
		registerCallback(new FreeAccelerationPublisher(node));
	}
	if (ros::param::get("~pub_transform", should_publish) && should_publish)
	{
		registerCallback(new TransformPublisher(node));
	}
	if (ros::param::get("~pub_positionLLA", should_publish) && should_publish)
	{
		registerCallback(new PositionLLAPublisher(node));
	}
	if (ros::param::get("~pub_velocity", should_publish) && should_publish)
	{
		registerCallback(new VelocityPublisher(node));
	}
}

// 获取传感器ID
bool XdaInterface::connectDevice()
{
	// 读取波特率
	XsBaudRate baudrate = XBR_Invalid;
	if (ros::param::has("~baudrate"))
	{
		int baudrateParam = 0;
		ros::param::get("~baudrate", baudrateParam);
		// ROS_INFO("Found baudrate parameter: %d", baudrateParam);
		baudrate = XsBaud::numericToRate(baudrateParam);
	}
	// 读取设备ID
	bool checkDeviceID = false;
	std::string deviceId;
	if (ros::param::has("~device_id"))
	{
		ros::param::get("~device_id", deviceId);
		checkDeviceID = true;
		// ROS_INFO("Found device ID parameter: %s.",deviceId.c_str());

	}
	// 读取串口号
	XsPortInfo mtPort;
	if (ros::param::has("~port"))
	{
		std::string portName;
		ros::param::get("~port", portName);
		// ROS_INFO("Found port name parameter: %s", portName.c_str());
		mtPort = XsPortInfo(portName, baudrate);
		// ROS_INFO("Scanning port %s ...", portName.c_str());
		if (!XsScanner::scanPort(mtPort, baudrate))
			return handleError("No MTi device found. Verify port and baudrate.");
		if (checkDeviceID && mtPort.deviceId().toString().c_str() != deviceId)
			return handleError("No MTi device found with matching device ID.");

	}
	else
	{
		// ROS_INFO("Scanning for devices...");
		XsPortInfoArray portInfoArray = XsScanner::scanPorts(baudrate);

		for (auto const &portInfo : portInfoArray)
		{
			if (portInfo.deviceId().isMti() || portInfo.deviceId().isMtig())
			{
				if (checkDeviceID)
				{
					if (portInfo.deviceId().toString().c_str() == deviceId)
					{
						mtPort = portInfo;
						break;
					}
				}
				else
				{
					mtPort = portInfo;
					break;
				}
			}
		}
	}
	if (mtPort.empty())
		return handleError("can`t find xsens");

	ROS_INFO("Xsens ID: %s @ port: %s, baudrate: %d", mtPort.deviceId().toString().toStdString().c_str(), mtPort.portName().toStdString().c_str(), XsBaud::rateToNumeric(mtPort.baudrate()));

	// ROS_INFO("Opening port %s ...", mtPort.portName().toStdString().c_str());
	if (!m_control->openPort(mtPort))
		return handleError("Could not open port");

	m_device = m_control->device(mtPort.deviceId());
	assert(m_device != 0);
	// ROS_INFO("Device: %s, with ID: %s opened.", m_device->productCode().toStdString().c_str(), m_device->deviceId().toString().c_str());
	m_device->addCallbackHandler(&m_xdaCallback);
	return true;
}

// 传感器config，进入测量模式
bool XdaInterface::prepare()
{
	assert(m_device != 0);

	if (!m_device->gotoConfig())
		return handleError("Could not go to config");

	// read EMTS and device config stored in .mtb file header.
	if (!m_device->readEmtsAndDeviceConfiguration())
		return handleError("无法读取传感器配置");

	// ROS_INFO("Measuring ...");
	if (!m_device->gotoMeasurement())
		return handleError("传感器无法进入测量模式");

	std::string logFile;
	if (ros::param::get("~log_file", logFile))
	{
		if (m_device->createLogFile(logFile) != XRV_OK)
			return handleError("无法创建log文件(" + logFile + ")");
		else
			// ROS_INFO("创建log文件: %s", logFile.c_str());

		// ROS_INFO("Recording to %s ...", logFile.c_str());
		if (!m_device->startRecording())
			return handleError("无法开始测量");
	}
	return true;
}

void XdaInterface::close()
{
	if (m_device != nullptr)
	{
		m_device->stopRecording();
		m_device->closeLogFile();
		m_device->removeCallbackHandler(&m_xdaCallback);
	}
	m_control->closePort(m_port);
}

//调用XsControl的push_back函数
void XdaInterface::registerCallback(PacketCallback *cb)
{
	m_callbacks.push_back(cb);
}

// 打印错误
bool XdaInterface::handleError(std::string error)
{
	ROS_ERROR("%s", error.c_str());
	close();
	return false;
}
