/***********************************************************
@File:FramePlane.h
@Author:lxz
@Date:2019-7-24
@Description: 表示在当前帧中可见的平面对象的类,主要用于平面的分割
@History:
************************************************************/
#pragma once
#ifndef FRAME_PLANE_H
#define FRAME_PLANE_H

#include "FrameObject.h"

#include <vector>


namespace ht {
	/**
	 * @FramePlane: 表示在当前帧中可见的平面对象的类。
	 * 由其(reduced)方程的系数定义： ax + by - z + d = 0
	 * 模拟跟踪手和背景平面对象的示例。
	 *
	 * @Type: Normal
	 */
    class FramePlane : public FrameObject
    {
    public:
		/** 构造一个默认平面(其法线通过点(0,0,0)和(0,0,-1)）  */
        FramePlane();

		/**
		 * 使用给定的参数矢量和给定的 3D 对象信息构造平面
		 * @Descrip:
		 * 公式:v[0]x + v[1]y - z + v[2] = 0
		 * 即, z = v[0]x + v[1]y + v[2]
		 *
		 * @param [in] v 平面参数的矢量
		 */
        FramePlane(const Vec3f & v, const cv::Mat & cluster_depth_map, DetectionParams::Ptr params = nullptr);

		/**
		* 使用给定的参数矢量和给定的 3D 对象信息构造平面
		* @Descrip:
		* 公式:v[0]x + v[1]y - z + v[2] = 0
		* 即, z = v[0]x + v[1]y + v[2]
		*
		* @param [in] v: 平面参数的矢量
		* @param [in] points_ij: 容纳对象中的所有点坐标的向量容器(在屏幕坐标中)
		* @param [in] points_xyz: 容纳对象中的所有点坐标的向量容器(在世界坐标中)
		* @param [in] depth_map: 所有的点云数据。(可能包含此检测目标外部的点)
		* @param [in] params: 对象/手检测的参数参数(如果未指定,则使用默认参数)
		* @param [in] sorted:  点云是否已排序，如果为 true,则认为这些"点"已排序并跳过排序以节省时间
		* @param [in] points_to_use:  可选,要用于目标对象"点"中的点数。默认情况下,使用所有点。
		*/
        FramePlane(Vec3f v, VecP2iPtr points_ij, VecV3fPtr points_xyz,
            const cv::Mat & depth_map, DetectionParams::Ptr params = nullptr,
            bool sorted = false, int points_to_use = -1);

		/**
		 * 包含平面方程的系数
		 * 公式:v[0]x + v[1]y - z + v[2] = 0
		 * 即, z = v[0]x + v[1]y + v[2]
		 */
        const Vec3f equation;

		/**
		 * 返回此平面的单位化(长度为1.0)法线矢量,指向查看器。
		 */
        Vec3f getNormalVector();

		/**
		 * 查找平面上带有给定 x 和 y 坐标的点的 z 坐标
		 * @Descrip:
		 *
		 * @param [in] x: 点的x坐标
		 * @param [in] y: 点的y坐标
		 *
		 * @return [float]: 点的Z坐标
		 */
        float getZ(float x, float y);

		/**
		 * 从平面到给定点的欧氏规范
		 * @Descrip:
		 *
		 * @param point: 点的三维坐标
		 * @return [float]: 欧式距离
		 */
        float squaredDistanceToPoint(const Vec3f & point) const;

		/**
		 * 查找从平面到给定点的距离
		 * @Descrip:
		 *
		 * @param point: 点的三维坐标
		 * @return [float]: 点面距离
		 */
        float distanceToPoint(const Vec3f & point) const;

		/** 获取旋转矩形 (2D) 的顶点,该顶点绑定到平面的相关区域 */
        const std::vector<Point2f> & getPlaneBoundingRect() const;

		/** 指向FramePlane实例的共享指针 */
        typedef std::shared_ptr<FramePlane> Ptr;

    private:

		/** 边界(旋转)矩形的顶点 */
        void initializePlane();

        /** Vertices of bounding (rotated) rectangle */
        std::vector<Point2f> boundingRect;
    };
}

#endif //FRAME_PLANE_H