#ifndef XDACALLBACK_H
#define XDACALLBACK_H

#include <ros/ros.h>
#include <xscontroller/xscallback.h>
#include <mutex>
#include <condition_variable>
#include <list>

struct XsDataPacket;
struct XsDevice;

// 存放回调函数，根据回调的信号种类使用不同的回调类型
typedef std::pair<ros::Time, XsDataPacket> RosXsDataPacket;

class XdaCallback : public XsCallback
{
public:
	XdaCallback(size_t maxBufferSize = 5);
	virtual ~XdaCallback() throw();

	RosXsDataPacket next(const std::chrono::milliseconds &timeout);

protected:
	void onLiveDataAvailable(XsDevice *, const XsDataPacket *packet) override;

private:
	std::mutex m_mutex;
	std::condition_variable m_condition;
	std::list<RosXsDataPacket> m_buffer;
	size_t m_maxBufferSize;
};

#endif
