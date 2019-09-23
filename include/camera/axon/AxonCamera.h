/***********************************************************
@File:PlaneDetector.h
@Author:lxz
@Date:2019-9-18
@Description: 根据axon相机驱动改写的相机类
@History: NULL
************************************************************/

#pragma once
#ifndef AXON_CAMERA_H
#define AXON_CAMERA_H

// STL Libraries
#include <memory>
#include <thread>
#include <atomic>

// OpenNI Libraries
#include <OpenNI.h>

// AXon camera Libraries
#include <AXonLink.h>

// Base class
#include "DepthCamera.h"



namespace ht {

	/**
	 * @AXonCamera: AXon相机驱动类
	 * 用于驱动AXon相机的派生类，重写了一些图像获取的方法。
	 *
	 * @Type: Normal
	 */
	class AXonCamera : public DepthCamera
	{
	public:
		/**
		 * 构造函数
		 * @Descrip:包含初始化并启动相机等操作
		 * @param [in] params: 平面检测器参数实例
		 */
		explicit AXonCamera();

		/** 销毁PMD相机实例 */
		~AXonCamera() override;

		/**
		 * 获取相机型号
		 * @Descrip: 重写DepthCamrea类中的函数
		 *
		 * @retun [std::string]: 相机的名称或型号
		 */
		const std::string getModelName() const override;

		/**
		 * 获取深度图像的像素宽度
		 * @Descrip: 此宽度由相机的自带参数决定
		 *
		 * @return [int] 图像像素宽度
		 */
		int getWidth() const override;

		/**
		 * 获取深度图像的像素高度
		 * @Descrip: 此宽度由相机的自带参数决定
		 *
		 * @return [int] 图像像素高度
		 */
		int getHeight() const override;

		/**
		 * 确定点相对于相机的flagMap的有效性的值
		 * @Descrip:
		 *
		 * return [float]
		 */
		float flagMapConfidenceThreshold() const override;

		/**
		 * 确定相机AmpMap的点有效的值
		 * @Descrip:
		 *
		 * return [int]
		 */
		int ampMapInvalidFlagValue() const override;

		/**
		 * 是否需要或者能够从相机中获取AmpMap
		 * @Descrip: 只需要设置函数的返回值即可;
		 * 如果你的相机不能获取AmpMap图像，请你务必设置为false；
		 * 如果设置为true后，必须到update()中实现获取该图像的方法
		 *
		 * @return [bool]
		 */
		bool hasAmpMap() const override;

		/**
		 * 是否需要或者能够从相机中获取flagMap
		 * @Descrip: 只需要设置函数的返回值即可;
		 * 如果你的相机不能获取flagMap图像，请你务必设置为false；
		 * 如果设置为true后，必须到update()中实现获取该图像的方法
		 *
		 * @return [bool]
		 */
		bool hasFlagMap() const override;

		/** 指向PMD相机实例 */
		typedef std::shared_ptr<AXonCamera> Ptr;

		//深度图像参数
		static const int Depth_Width = 640; // 深度图像宽度
		static const int Depth_Height = 480; // 深度图像高度

	protected:
		/**
		 * 从相机中获得下一帧图像信息
		 * @Descrip: 从深度相机中获取下一帧的XYZ,RGB,IR等图像
		 * 通过调用此函数来刷新对应的图像信息，你可以在此函数中从相机中获取图像并转化为对应的map形式。
		 * 注意：获取的***_map的宽高必须与上面getHeight(),getWidth()函数获取到的一致，否则可能导致程序崩溃。
		 * 注意：如果在在后面的函数中has***Map()设置为flase，则对应的***_map则可以不用获取；
		 * 例如，如果你的相机能够获取的RGB图像且设置hasRGBMap()返回true,那么则必须要给参数rgb_map更新内容。
		 * 注意：此方法必须要在派生类中实现！！！
		 *
		 * @param [out] xyz_map		深度图像中所有点的XYZ坐标. CV_32FC3
		 * @param [out] rgb_map		RGB图像. CV_8UC3
		 * @param [out] ir_map		红外图像. CV_8UC1
		 * @param [out] amp_map		置信参数. CV_32FC1
		 * @param [out] flag_map	其它图像. CV_8UC1
		 *
		 * @return [void]
		 */
		void update(cv::Mat& xyz_map, cv::Mat& rgb_map, cv::Mat& ir_map,
			cv::Mat& amp_map, cv::Mat& flag_map) override;
	private:
		/**
		 * 初始化相机
		 */
		void initCamera();
		/**
		 * 更新深度信息辅助函数
		 */
		void updateHelper();

		CamIntrinsicParam* GetDepthInstrinsicParamByResolution(int  nWidth, int nHeight, AXonLinkCamParam* allParam);
	
		bool waitAllStream(openni::VideoStream** streams, int allCount, int timeout);
	private:

		/** 设备管理器对象 */
		openni::Device device;
		/** 深度流对象 */
		openni::VideoStream depth;
		/** RGB流对象 */
		openni::VideoStream color;
		/** IR流对象 */
		openni::VideoStream ir;
		/** 深度数据帧对象 */
		//openni::VideoFrameRef depthFrame;
		/** 深度数据初始化参数 */
		//CamIntrinsicParam* depthIntParam;

		/** 图像捕获线程 */
		std::shared_ptr<std::thread> capThread;

		/** 信号量，用于控制图像捕获线程 */
		std::atomic_bool isCapture;

		/** 图像互斥锁 */
		std::mutex imageMutex;

		/** 包含三维点云信息的图像矩阵 */
		cv::Mat xyzMap;

		/** 图像矩阵缓冲 */
		cv::Mat xyzBuffer;
	};

}

#endif //AXON_CAMERA_H
