#include "xdacallback.h"

#include <xscontroller/xsdevice_def.h>
#include <xstypes/xsdatapacket.h>

XdaCallback::XdaCallback(size_t maxBufferSize): m_maxBufferSize(maxBufferSize)
{
}

XdaCallback::~XdaCallback() throw()
{
}

// 取出下一组数据包 当timeout时间段结束时返回空数据包
RosXsDataPacket XdaCallback::next(const std::chrono::milliseconds &timeout)
{
	RosXsDataPacket packet;
	std::unique_lock<std::mutex> lock(m_mutex);

	if (m_condition.wait_for(lock, timeout, [&] { return !m_buffer.empty(); }))
	// 为了取缓冲区里的数据。若m_buffer里有数据，则直接返回栈顶，并弹出；若没有数据，则至多等待timeout，超时则会因为assert而报错
	//  函数wait_for作用：会阻塞所在线程（该线程应当拥有lck），直至超过了rel_time，或者谓词返回true
	{
		assert(!m_buffer.empty());//如果缓存队列为0则报警
		packet = m_buffer.front();//缓存队列非空，取最前面的值
		m_buffer.pop_front();//队列向前再推1组数据
	}
	return packet;
}

void XdaCallback::onLiveDataAvailable(XsDevice *, const XsDataPacket *packet)
{
	std::unique_lock<std::mutex> lock(m_mutex);
	ros::Time now = ros::Time::now();

	assert(packet != 0);
	// 若缓存已满，丢弃最早的数据包
	if (m_buffer.size() == m_maxBufferSize)
	    {m_buffer.pop_front();}

	// Push new packet
	m_buffer.push_back(RosXsDataPacket(now, *packet));

	// Manual unlocking is done before notifying, to avoid waking up
	// the waiting thread only to block again
	lock.unlock();
	m_condition.notify_one();
}
