/***********************************************************
@File: DataBuffer.hpp
@Author: lxz
@Date: 2019-06-30
@Description: 数据流缓冲类，用于存放图像流；使用循环缓冲队列进行设计
@History: NULL
************************************************************/
#pragma once
#ifndef DATA_BUFFER_H
#define DATA_BUFFER_H
//STL
#include <condition_variable>
//BOOST
#include <boost/circular_buffer.hpp>  
#include <boost/noncopyable.hpp>

namespace ht {
	/**
	* 数据流缓冲类
	*/
	template <class T>
	class DataBuffer : public boost::noncopyable
	{
	public:
		typedef boost::circular_buffer<T> BufferType;
		typedef typename BufferType::size_type size_type;
		typedef typename BufferType::value_type value_type;

		explicit DataBuffer(size_type size = 3) :m_Qeque(size)
		{}

		void Put(const value_type& data)
		{
			{
				std::lock_guard<std::mutex> lock(m_mutex);
				m_Qeque.push_back(data);
			}

			m_cndNotEmpty.notify_one();

		}

		void Get(value_type& data)
		{

			std::unique_lock<std::mutex> lock(m_mutex);
			while (m_Qeque.empty())
			{
				m_cndNotEmpty.wait(lock);
			}

			data = m_Qeque.front();
		}
		//T Get()
		//{

		//	std::unique_lock<std::mutex> lock(m_mutex);
		//	while (m_Qeque.empty())
		//	{
		//		m_cndNotEmpty.wait(lock);
		//	}

		//	return m_Qeque.front();
		//}

		void operator<< (value_type& data) {
			this->Put(data);
		}
		void operator>> (value_type& data) {
			this->Get(data);
		}
	private:
		BufferType m_Qeque;
		std::mutex m_mutex;
		std::condition_variable m_cndNotEmpty;
	};
}

#endif //DATA_BUFFER_H
