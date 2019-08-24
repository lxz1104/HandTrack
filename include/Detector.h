/***********************************************************
@File:Detector.h
@Author:lxz
@Date:2019-7-23
@Description:检测器的抽象类定以及实现，提供了检测器的一些共有的方法的接口
@History:
************************************************************/
#pragma once
#ifndef DETECTOR_H
#define DETECTOR_H
#include "DepthCamera.h"
#include "FrameObject.h"

namespace ht {
	/**
	 * 检测器的抽象类，提供不同检测器的公共接口
	 *
	 * @Type: Abstract
	 */
    class Detector {
    public:
		/**
		 * 构造函数
		 * @Descrip: 创造一个新的检测器实例
		 *
		 * @param [in]  params :检测器参数；如果没有额外指定，则使用默认参数
		 */
        Detector(DetectionParams::Ptr params = nullptr);

		/**
		 * 通过提供深度图像来更新检测器
		 * @Descrip：
		 *
		 * @param [in] image: 保存有深度图像坐标信息的三通道矩阵（xyz map）
		 *
		 * @return [void]
		 */
        void update(const cv::Mat & image);
        
		/**
		 * 通过调用相机更新图像来进行探测器更新
		 * @Descrip：调用给定的深度相机实例更新并获取xyz map
		 *
		 *  @param [in] camera: 深度相机实例化的引用
		 *
		 * @return [void]
		 */
        void update(DepthCamera & camera);

		/**
		 * 改变检测器的参数
		 * @Descrip：如果需要自定义检测器的参数，则可通过此函数完成
		 *
		 * @param [in] 检测参数：自定义的检测器参数实例
		 *
		 * @return [void]
		 */
        void setParams(const DetectionParams::Ptr params);

		/**
		 * 指向检测器实例的智能指针
		 * @Type:  std::shared_ptr<Detector>
		 */
        typedef std::shared_ptr<Detector> Ptr;

    protected:
		/**
		 * 目标检测器的主要功能
		 * @Descrip: 在每次更新图像后调用，必须在子类中实现相应的功能
		 *
		 * @param: 保存有深度图像坐标信息的三通道矩阵（xyz map）
		 *
		 * @return [void]
		 */
        virtual void detect(cv::Mat & image) = 0;

		/** 指向此探测器的目标检测参数的指针  */
        DetectionParams::Ptr params; 

    private:
		/**
		 * 存储当前帧的xyz map数据
		 * @Type: CV_32FC3
		 */
        cv::Mat image;

		/** 指向上次使用的深度相机实例的指针（捕获上一帧图像的相机）。  */
        DepthCamera * lastCamera = nullptr;

		/** 为True表示：如果相机的图像自上次更新调用以来未更新(因此我们不需要重新计算任何内容)  */
        bool onSameFrame = true;

		/** 前一个回调函数的 ID */
        int lastUpdateCallbackID = -1;

		/**  用于响应深度相机更新的回调函数的功能 */
        void callbackHelper(DepthCamera & camera);
        
		/** 回调函数实例 */
        std::function<void(DepthCamera &camera)> callback;
    };
}

#endif //DETECTOR_H