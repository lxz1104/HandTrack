/***********************************************************
@File: DepthCamera.h
@Author: lxz
@Date: 2019-07-22
@Description: 深度相机与算法之间的统一接口(此部分没设计好，需要重构)
@History: NULL
************************************************************/
#pragma once
#ifndef DEPTH_CAMERA_H
#define DEPTH_CAMERA_H

//#include "ThreadPool.hpp"
#include "DetectionParams.h"
#include <opencv2/opencv.hpp>
#include <mutex>
#include <map>

namespace ht {
	/**
	 * @DepthCamera: 深度相机抽象类型（虚基类）
	 * 此类提供了一个通用的深度接口，用于从深度相机中获取XYZMap(深度坐标信息)，
	 * AMPMap（深度置信值），以及FlagMap。
	 * 任何相机都应该重写此类中对应的纯虚函数才能正常使用后续的一系列功能。
	 *
	 * @Type: Abstract
	 */
    class DepthCamera
    {
		// 下面的方法需要在其派生类中重写,才能使用相应的功能
    public:

		/**
		* 获取当前使用的相机型号或名字
		* @Descrip: 由实际使用的相机决定
		*
		* @return [std::string] 相机名或型号
		*/
        virtual const std::string getModelName() const;

		/**
		 * 获取深度图像的像素宽度
		 * @Descrip: 此宽度由相机的自带参数决定
		 *
		 * @return [int] 图像像素宽度
		 */
        virtual int getWidth() const = 0;

		/**
		 * 获取深度图像的像素高度
		 * @Descrip: 此高度由相机的自带参数决定
		 *
		 * @return [int] 图像像素高度
		 */
        virtual int getHeight() const = 0;
        
		/**
		 * 获取深度相机的默认检测参数
		 * @Descrip:
		 *
		 * @return [DetectionParams::Ptr] 相机检测参数的指针
		 */
        virtual const DetectionParams::Ptr & getDefaultParams() const;

		/**
		 * 虚析构函数
		 * @Descrip: 此不分由派生类自行完成，用作资源收，以及关闭相机等操作
		 *
		 * @return [void]
		 */
        virtual ~DepthCamera();

    protected:
		// 此函数必须在派生类中重写，否则无法运行算法

		/**
		 * 从相机中获得下一帧图像信息
		 * @Descrip: 从深度相机中获取下一帧的XYZ,RGB,IR等图像
		 * 通过调用此函数来刷新对应的图像信息，你可以在此函数中从相机中获取图像并转化为对应的map形式。
		 * 注意：获取的***_map的宽高必须与上面getHeight(),getWidth()函数获取到的一致，否则可能导致程序崩溃。
		 * 注意：如果在在后面的函数中has***Map()设置为flase，则对应的***_map则可以不用获取；
		 * 例如，如果你的相机能够获取的RGB图像且设置hasRGBMap()返回true,那么则必须要给参数rgb_map更新内容。
		 * 注意：此方法必须要在派生类中实现！！！
		 *
		 * @param [out] xyz_map XYZ map (projection point cloud). CV_32FC3
		 * @param [out] rgb_map RGB image. CV_8UC3
		 * @param [out] ir_map IR image. CV_8UC1
		 * @param [out] amp_map amplitude map. CV_32FC1
		 * @param [out] flag_map flag map. CV_8UC1
		 *
		 * @return [void]
		 */
        virtual void update(cv::Mat & xyz_map, cv::Mat & rgb_map, cv::Mat & ir_map, 
                            cv::Mat & amp_map, cv::Mat & flag_map) = 0;

    public:
		// 下面的函数可以根据你的需求进行重写

		/**
		 * 是否需要或者能够从相机中获取RGB图像
		 * @Descrip: 只需要设置函数的返回值即可;
		 * 如果你的相机不能获取RGB图像，请你务必设置为false；
		 * 如果设置为true后，必须到update()中实现获取该图像的方法
		 *
		 * @return [bool]
		 */
        virtual bool hasRGBMap() const;

		/**
		 * 是否需要或者能够从相机中获取IR图像(红外图像)
		 * @Descrip: 只需要设置函数的返回值即可;
		 * 如果你的相机不能获取IR图像，请你务必设置为false；
		 * 如果设置为true后，必须到update()中实现获取该图像的方法
		 *
		 * @return [bool]
		 */
        virtual bool hasIRMap() const;

		/**
		 * 是否需要或者能够从相机中获取AmpMap
		 * @Descrip: 只需要设置函数的返回值即可;
		 * 如果你的相机不能获取AmpMap图像，请你务必设置为false；
		 * 如果设置为true后，必须到update()中实现获取该图像的方法
		 *
		 * @return [bool]
		 */
        virtual bool hasAmpMap() const;

		/**
		 * 是否需要或者能够从相机中获取flagMap
		 * @Descrip: 只需要设置函数的返回值即可;
		 * 如果你的相机不能获取flagMap图像，请你务必设置为false；
		 * 如果设置为true后，必须到update()中实现获取该图像的方法
		 *
		 * @return [bool]
		 */
        virtual bool hasFlagMap() const;

		/**
		 * 确定相机AmpMap的点有效的值
		 * @Descrip:
		 *
		 * return [int]
		 */
        virtual int ampMapInvalidFlagValue() const;

		/**
		 * 确定点相对于相机的flagMap的有效性的值
		 * @Descrip:
		 *
		 * return [float]
		 */
        virtual float flagMapConfidenceThreshold() const;

		/**
		 * 检查相机的输入是否有效
		 * @Descrip: 此功能需要用户自己实现，且此功能依赖相机提供的SDK。
		 *
		 * @return [bool] : 当相机输入错误，断开连接等无法正常工作的情况时返回true，未发现异常时返回true。
		 */
        virtual bool badInput(); 

		// 相机可能会用到的通用方法或者变量

		/**
		 * 从深度相机中检索下一帧
		 * @Descrip: 调用派生类中的重写的update()函数重置并存储相机当前帧的信息，主要用在监测算法中
		 *
		 * @param [in]: 是否消除图像中的噪声，缺省值为true.
		 *
		 * @return [bool]: 返回true,更新成功;返回false,获取图像出错
		 */
        bool nextFrame(bool remove_noise = true);

		/**
		 * 开始让相机在一个并行的线程中捕获图像
		 * @Descrip: 以一定的最大帧率捕获图像;如果相机之前已开始工作，调用此函数会抛出异常。
		 *
		 * @param [in]: 相机捕获图像的最大帧率
		 * @param [in]: 是否除去图像噪声，缺省值为true
		 *
		 * @return [void]
		 */
        void beginCapture(int fps_cap = 50, bool remove_noise = true);

		/**
		 * 让相机停止捕获图像
		 * @Descrip: 调用此函数让相机停止工作后，你可以调用beginCapture()函数让相机再次工作。
		 * 注意：此方法会在本实例销毁时自动调用
		 *
		 * @return [void]
		 */
        void endCapture();

		/**
		 * 检查相机是否正在工作
		 * @Descrip: 用于检测相机当前的状态
		 *
		 * @return [bool]: 返回true，相机正在捕获图像;返回false,相机没有工作
		 */
        bool isCapturing();

		/**
		 * 为每帧图像获取后添加回调函数
		 * @Descrip: 绑定的回调函数会在获取到每帧图像后立即执行。
		 * 注意: 回调函数可能会从与添加回调的线程不同的线程调用。（异步执行）
		 *
		 * @param [std::function<DepthCamera &>]: 传入一个函数包装器（可以时函数指针，lambda表达式）,
		 * 参数必须为一个指向调用update()函数的引用（DepthCamera &）。
		 *
		 * @return [int]: 返回一个标识回调函数的id，用于去除回调函数等操作
		 */
        int addUpdateCallback(std::function<void(DepthCamera &)> func);

		/**
		 * 移除update()方法的回调函数
		 * @Descrip: 通过指定的id移除回调函数。
		 *
		 * @param [in]: 回调函数的id(在调用addUpdateCallback()时获得)
		 *
		 * @return [void]
		 */
        void removeUpdateCallback(int id);
        
		/**
		 * 获取相机的每一帧图像的大小
		 * @Descrip： 其大小等价于 getWidth() * getHeight()
		 *
		 * @return [cv::size]
		 */
        cv::Size getImageSize() const;

		/**
		 * 获取当前的XYZmap(有序点云)
		 * @Descrip: XYZ Map 即为包含屏幕上每个像素的XYZ位置(以米为单位)的容器,类型为CV_32FC3。
		 *
		 * @return [cv::Mat(CV_32FC3)]
		 */
        cv::Mat getXYZMap();

		/**
		 * 获取当前的RGB图像
		 * @Descrip: 如果能够获取RGB图像则返回RGB图像的矩阵,否则抛出错误
		 *
		 * @return [CV_8UC3]
		 */
        cv::Mat getRGBMap();

		/**
		 * 获取当前的IR图像
		 * @Descrip: 如果能够获取RGB图像则返回IR图像的矩阵,否则抛出错误
		 *
		 * @return [cv::Mat(CV_8UC1)]
		 */
        cv::Mat getIRMap();

		/**
		 * 获取图像当前的AmpMap图像
		 * @Descrip:
		 *
		 * @return [cv::Mat(CV_32FC1))]
		 */
        cv::Mat getAmpMap();

		/**
		 * 获取当前的FlagMap
		 * @Descrip
		 *
		 * @return [cv::Mat(CV_8UC1)]
		 */
        cv::Mat getFlagMap();

		/** 指向当前实例的智能指针 */
        typedef std::shared_ptr<DepthCamera> Ptr;

    protected:
		/**
		 * xyzMap
		 * 节点元素信息描述:
		 * 存储相机可观测世界中每个点的 (x,y,z) 数据的矩阵
		 * @Type: CV_32FC3
		 */
        cv::Mat xyzMap;

		/**
		 * 保存相机可观测世界中每个相应点的置信度的矩阵
		 * @Type: CV_8UC1
		 */
        cv::Mat ampMap;

		/**
		 * 保存相机可观测世界中每个相应点的置信度的矩阵
		 * @Type: CV_8UC1
		 */
        cv::Mat flagMap;

		/**
		 * 保存RGB信息的矩阵(如果可用)
		 * @Type: CV_8UC3
		 */
        cv::Mat rgbMap;

		/**
		 * 保存红外图像的矩阵(如果可用)
		 * @Type: CV_8UC1
		 */
        cv::Mat irMap;

		/**
		 * 值为True表示图像输入出错
		 * badInput()默认返回此值
		 * badInput()方法可以被重写
		 */
        bool badInputFlag;
        
		/** 线程锁，可确保更新图像时线程安全 */
        mutable std::mutex imageMutex;

		/** 图像缓冲 */
		//DataBuffer<cv::Mat> xyzBuffer;
		//DataBuffer<cv::Mat> ampBuffer;

    private:
		// 一些基础方法

		/**
		 * 用于初始化通用深度相机使用的图像的辅助函数
		 * @Descrip: 用于分配内存等操作
		 *
		 * @return [void]
		 */
        void initializeImages();

		/**
		 * 将单个图像的后台缓冲区交换到前台的辅助函数
		 * @Descrip: 如果图像不可用,则创建具有空值的矩阵
		 *
		 * @param [in] check_func: 成员函数指针函数,如果调用时为 true,则交换缓冲区(如果为 false),则创建空矩阵
		 * @param [in] img: 指向前景图像的指针
		 * @param [in] buf: 指向后台缓冲区的指针
		 */
        void swapBuffer(bool (DepthCamera::* check_func)() const, cv::Mat & img, cv::Mat & buf);

		/**
		 * 用于将所有后台缓冲区交换到前台的辅助函数。
		 * @Descrip:
		 *
		 * @return [void]
		 */
        void swapBuffers();

		/**
		 * 根据AMPMap和FlagMap中提供的置信度,消除 XYZMap 中的噪声
		 * @Descrip:
		 *
		 * @param [out] xyzMap: 要去噪的xyzMap
		 * @param [in] AMPMap: 深度置信值容器
		 * @param [in] confidence_thresh: 置信度
		 *
		 * @return [void]
		 */
        static void removeNoise(cv::Mat & xyzMap, cv::Mat & ampMap, float confidence_thresh);

		/** 存储每次更新后要调用的回调函数(ID、函数指针) */
        std::map<int, std::function<void(DepthCamera &)> > updateCallbacks;

		/**
		 * 相机默认捕获行为的辅助函数
		 * @Descrip:此函数会单独在一个线程中运行，用于不停更新
		 *
		 * @param [in] fps_cap: 最大帧率
		 * @param [in] interrupt: 指向中断的指针(当为 true 时,线程停止)
		 * @param [in] remove_noise: 如果为 true,则自动去噪
		 */
        void captureThreadingHelper(int fps_cap = 30, volatile bool * interrupt = nullptr,
                                    bool remove_noise = true);

        /** 用于终止捕获线程的信号量 */
        bool captureInterrupt = true;

        /**
         * 点的最小深度(以米为单位)。此深度下的点视为噪声。
         * (0.0表示禁用,位于DepthCamera.cpp)
         */
        static const float NOISE_FILTER_LOW;

        /**
         * 点的最大深度(以米为单位)。高于此深度的点视为噪声。
         * (0.0表示禁用,位于DepthCamera.cpp)
         */
        static const float NOISE_FILTER_HIGH;
		
		/** 线程池 */
		//ThreadPool thPool;

		
		/** 图像的后台缓冲区 */
        cv::Mat xyzMapBuf;
        cv::Mat rgbMapBuf;
        cv::Mat irMapBuf;
        cv::Mat ampMapBuf;
        cv::Mat flagMapBuf;
    };
}

#endif // DEPTH_CAMERA_H