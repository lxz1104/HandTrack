/***********************************************************
@File:PlaneDetector.h
@Author:lxz
@Date:2019-7-01
@Description: 平面检测器部分，定义并实现了平面检测算法
@History:
************************************************************/
#pragma once
#ifndef PLANEDETECTOR_H
#define PLANEDETECTOR_H

#include "Detector.h"
#include "FramePlane.h"

namespace ark {
	/**
	 * @PlaneDetector: 平面探测器类
	 * 支持在深度投影图像(xyz map)中检测多个平面。
	 *
	 * @Type: Normal
	 */
    class PlaneDetector : public Detector {
    public:
		/**
		 * 构造新的平面探测器实例
		 * @param [in] params: 平面检测器参数实例
		 */
        PlaneDetector(DetectionParams::Ptr params = nullptr);

		/**
		 * 从此探测器获取当前帧中的平面列表。
		 * @Descrip:
		 *
		 * @retun [std::vector<FramePlane::Ptr> &]: 平面可见对象实例引用
		 */
        const std::vector<FramePlane::Ptr> & getPlanes() const;

		/** 指向PlaneDetector实例的共享指针  */
        typedef std::shared_ptr<PlaneDetector> Ptr;

		/**
		 * 从此平面探测器获取法线地图(如果可用)
		 * @Descrip:
		 *
		 * @return [cv::Mat]: 法线地图;如果一个不可用,则返回一个空图像
		 */
        cv::Mat getNormalMap();

    protected:
		/**
		 * 平面检测算法的实现
		 * @Descrip:
		 *
		 * @param [in] image: xyz map
		 *
		 * @return [void]
		 */
        void detect(cv::Mat & image) override;

    private:
		/** 存储当前检测到的平面 */
        std::vector<FramePlane::Ptr> planes;

		/**
		 * 保存在可观察世界中的每个点存储曲面法线矢量(facing viewer)的矩阵。
		 * 如果需要,从深度图自动计算。
		 * 子类的实现者根本不需要处理它。
		 *
		 * @Type: CV_32FC3
		 */
        cv::Mat normalMap;

		/**
		 * 用于获取给定xyz Map和法线图的平面方程的辅助函数
		 * @Descrip:
		 *
		 * @param [in]  xyz_map:  xyz map
		 * @param [in]  normal_map: 法线地图
		 * @param [out] output_equations: 要用平面方程填充系数的向量容器(形如: ax + by - z + c = 0)
		 * @param [out] output_points: 要在平面上填充 ij 坐标点的矢量容器
		 * @param [out] output_points_xyz：要用平面上的 xyz 坐标点矢量填充矢量
		 * @param [in]  params：平面检测参数
		 *
		 * @return [void]
		 */
        void detectPlaneHelper(const cv::Mat & xyz_map, const cv::Mat & normal_map, std::vector<Vec3f> & output_equations,
            std::vector<VecP2iPtr> & output_points, std::vector<VecV3fPtr> & output_points_xyz,
            DetectionParams::Ptr params = nullptr);
    };
}

#endif // !PLANEDETECTOR_H