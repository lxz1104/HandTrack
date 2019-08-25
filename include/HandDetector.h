/***************************************************************************
 *@File:HandDetector.h
 *@Author:YQ
 *@Date:2019-07-26
 *@Description：该头文件主要实现了对手检测类HandDetector定义,说明了HandDetector类的继承来源，
				以及对相关函数进行了声明
 *@History:
 ***************************************************************************/
#pragma once
#ifndef HAND_DETECTOR_H
#define HAND_DETECTOR_H

#include "Detector.h"
#include "Hand.h"

namespace ht {
	/** 支持在深度数据（XYZMap）中检测多只手的手检测器类。
	 * @see PlaneDetector
	 */
    class HandDetector : public Detector {
    public:

		/**
		 * 构造一个新的手动探测器实例，使用提供的平面探测器删除每个帧中的平面。
		 * 如果您的应用程序需要检测到平面和手，
		 * 这减少了冗余，因为每个帧只检测一次平面。
		 * 警告: 假设每帧也更新了平面探测器。
		 * @param [out] plane_detector 指向用于查找平面的平面检测器实例的指针。
		 */
        explicit HandDetector(DetectionParams::Ptr params = nullptr);

		/**
		 * 从该探测器获取当前帧中的手列表
		 * @return 手的列表, 按SVM置信度的降序排序。
		 *         如果SVM置信度不可用，则按深度升序排序。
		 */
        const std::vector<Hand::Ptr> & getHands() const;

		/** 指向handDetector实例的共享指针 */
        typedef std::shared_ptr<HandDetector> Ptr;

    protected:
		/** 手部检测算法的实现 */
        void detect(cv::Mat & image) override;

    private:

		/** 存储当前检测到的手 */
        std::vector<Hand::Ptr> m_hands;
    };
}

#endif //HAND_DETECTOR_H