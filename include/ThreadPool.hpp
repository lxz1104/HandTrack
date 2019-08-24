/***********************************************************
@File: ThreadPool.hpp
@Author: lxz
@Date: 2019-07-01
@Description: 数据流缓冲类，用于存放图像流
@History: NULL
************************************************************/
#pragma once
#ifndef THREAD_POOL_H
#define THREAD_POOL_H

#ifndef _VECTOR_
	#include <vector>
#endif // !_VECTOR_

#ifndef _QUEUE_
	#include <queue>
#endif // !_QUEUE_

#ifndef _FUTURE_
	#include <future>
#endif // !_FUTURE_

#ifndef _FUNCTIONAL_
	#include <functional>
#endif // !_FUNCTIONAL_

#ifndef _STDEXCEPT_
	#include <stdexcept>
#endif // !_STDEXCEPT_

namespace ht {
	class ThreadPool {
	public:
		ThreadPool(size_t threads = 3);
		template<class F, class... Args>
		auto enqueue(F&& f, Args&& ... args)
			->std::future<typename std::result_of<F(Args...)>::type>;
		~ThreadPool();
	private:
		// 用于跟踪线程，便于后续操作
		std::vector< std::thread > workers;
		// 任务队列
		std::queue< std::function<void()> > tasks;

		// 任务同步
		std::mutex queue_mutex;
		std::condition_variable condition;
		bool stop;
	};

	// 构造函数用于预启动一些工作线程
	inline ThreadPool::ThreadPool(size_t threads)
		: stop(false)
	{
		for (size_t i = 0; i < threads; ++i)
			workers.emplace_back(
				[this]
		{
			for (;;)
			{
				std::function<void()> task;

				{
					std::unique_lock<std::mutex> lock(this->queue_mutex);
					this->condition.wait(lock,
						[this] { return this->stop || !this->tasks.empty(); });
					if (this->stop && this->tasks.empty())
						return;
					task = std::move(this->tasks.front());
					this->tasks.pop();
				}

				task();
			}
		}
		);
	}

	// 添加新的工作线程到线程池中
	template<class F, class... Args>
	auto ThreadPool::enqueue(F&& f, Args&& ... args)
		-> std::future<typename std::result_of<F(Args...)>::type>
	{
		using return_type = typename std::result_of<F(Args...)>::type;

		auto task = std::make_shared< std::packaged_task<return_type()> >(
			std::bind(std::forward<F>(f), std::forward<Args>(args)...)
			);

		std::future<return_type> res = task->get_future();
		{
			std::unique_lock<std::mutex> lock(queue_mutex);

			// 线程池停止状态禁止添加新的工作线程
			if (stop)
				throw std::runtime_error("enqueue on stopped ThreadPool");

			tasks.emplace([task]() { (*task)(); });
		}
		condition.notify_one();
		return res;
	}

	// 在析构函数处等待所有线程结束
	inline ThreadPool::~ThreadPool()
	{
		{
			std::unique_lock<std::mutex> lock(queue_mutex);
			stop = true;
		}
		condition.notify_all();
		for (std::thread& worker : workers)
			worker.join();
	}
}



#endif
