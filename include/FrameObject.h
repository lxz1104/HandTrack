/***********************************************************
@File:FrameObject.h
@Date:2019-7-24
@Description: 表示在单个帧中检测到的3D目标的类的声明部分
@History:
************************************************************/

#pragma once
#ifndef FRAME_OBJECT_H
#define FRAME_OBJECT_H

#include "stdafx.h"
#include "DetectionParams.h"

#include <opencv2/opencv.hpp>
#include <vector>



namespace ht {
	/**
	 * 表示在单个帧中观察到的3D目标的类
	 * @Type: Normal
	 */
    class FrameObject
    {
    public:
		/**
		 * 通过孤立的点云数据来构造FrameObject实例
		 * @Descrip：
		 * 注意：不在聚类上的点必须在点云中具有为0的z 坐标
		 *
		 * @param [in] cluster_depth_map：聚类的点云数据
		 * @param [in] params：检测器参数
		 */
        FrameObject();

		/**
		 * 通过孤立的点云数据来构造FrameObject实例
		 * @Descrip：
		 * 注意：不在聚类上的点必须在点云中具有为0的z 坐标
		 *
		 * @param [in] cluster_depth_map：聚类的点云数据
		 * @param [in] params：检测器参数
		 */
        explicit FrameObject(const cv::Mat & cluster_depth_map,
            const DetectionParams::Ptr params = nullptr);

		/**
		 * 通过点云的向量容器来构造 FrameObject 实例。
		 * @Descrip：
		 *
		 * @param [in] points_ij: 属于目标的所有点云集合坐标（(屏幕像素坐标)）的向量容器
		 * @param [in] depth_map:  所有的点云数据。(可能包含此检测目标外部的点)
		 * @param [in] params:  目标/手检测的参数(如果未指定,则使用默认参数)
		 * @param [in] sorted:  点云是否已排序，如果为 true,则认为这些"点"已排序并跳过排序以节省时间
		 * @param [in] points_to_use: 可选,要用于目标对象"点"中的点数。默认情况下,使用所有点。
		 */
        FrameObject(VecP2iPtr points_ij,
            VecV3fPtr points_xyz,
            const cv::Mat & depth_map,
            const DetectionParams::Ptr params = nullptr,
            bool sorted = false,
            int points_to_use = -1
        );

		/**
		 * 销毁 FrameObject 实例。
		 * @Descrip：
		 */
        ~FrameObject();

		/**
		 * 获取此对象中的点的列表,在屏幕坐标系中
		 * @Descrip：
		 *
		 * @return [std::vector<Point2i>&]
		 */
        const std::vector<Point2i> & getPointsIJ() const;

		/**
		 * 获取此对象中的点的列表,在世界坐标系中(不太确定)
		 * @Descrip：
		 *
		 * @return [std::vector<Vec3f>]
		 */
        const std::vector<Vec3f> & getPoints() const;

		/**
		 * 获取屏幕坐标中目标对象的近似质心的屏幕像素坐标（i，j）
		 * @Descrip：
		 *
		 * @return [Point2i]
		 */
        const Point2i & getCenterIJ();

		/**
		 * 获取屏幕坐标中目标对象的近似质心的世界坐标(x,y,z)
		 * @Descrip：
		 *
		 * @return [Vec3f]
		 */
        const Vec3f & getCenter();

		/**
		 * 获取屏幕坐标中目标对象的边界框
		 * @Descrip：
		 *
		 *  @return [cv::Rect]
		 */
        cv::Rect getBoundingBox() const;

		/**
		 * 获取目标对象的平均深度
		 * @Descrip：
		 *
		 * @return [float]
		 */
        float getDepth();

		/**
		 * 获取对象的可见表面积
		 * @Descrip：
		 *
		 * @return [double]
		 */
        double getSurfArea();

		/**
		 * 获取此对象可见部分的深度图像
		 * @Descrip：
		 *
		 * @return [cv::Mat]
		 */
        const cv::Mat & getDepthMap();

		/**
		 * 查找此点云聚类中最大的 2D 轮廓
		 * @Descrip：
		 *
		 * @return [std::vector<Point2i>&] ： 储存此群集中最大 2D 等高线的点向量容器
		 */
        const std::vector<Point2i> & getContour();

		/**
		 * 获取此对象的 2D 凸包
		 * @Descrip：
		 *
		 * @return [std::vector<Point2i>&] ： 将此对象的 2D 凸包存储为点的向量容器
		 */
        const std::vector<Point2i> & getConvexHull();

		/** 指向FrameObject的共享指针  */
        typedef std::shared_ptr<FrameObject> Ptr;

    protected:
		/**
		 * 存储此目标对象点云集群中点的 ij 坐标（屏幕坐标系）
		 * @Type:  std::shared_ptr<std::vector<Point2i>>
		 */
        std::shared_ptr<std::vector<Point2i>> points = nullptr;

		/**
		 * 存储此目标对象点云集群中点的  xyz 坐标（世界坐标系）
		 * @Type: std::shared_ptr<std::vector<Vec3f>>
		 */
        std::shared_ptr<std::vector<Vec3f>> points_xyz = nullptr;

		/**
		 * 表示在聚类点云中使用到的点云数量
		 */
        int num_points;

		/**
		 * 聚类点云边界框的左上角坐标(i,j)
		 * @Type: Point2i
		 */
        Point2i topLeftPt;

		/**
		 * 部分XYZ 坐标集合(xyz map)
		 * @Type: CV_32FC3
		 */
        cv::Mat xyzMap;

		/**
		 * 聚类点云的完整 XYZ 坐标集合(xyz map)
		 * @Type: CV_32FC3
		 */
        cv::Mat fullXyzMap;

		/**
		 * 包含来自常规 xyzMap (CV_8U) 的规范化深度 (z) 信息的灰度图像
		 * 注意:大小为xyzMap 的2倍
		 * @Type: CV_8U
		 */
        cv::Mat grayMap;

		/**
		 * 存储完整 xyz map的大小
		 * @Type: cv::Size
		 */
        cv::Size fullMapSize;

		/**
		 * 表面积(单位:平方米)
		 */
        double surfaceArea = -1;

		/**
		 * 对象中最大的轮廓
		 * @Type: std::vector<Point2i>
		 */
        std::vector<Point2i> contour;

		/**
		 * 对象的凸包的像素坐标
		 * @Type: std::vector<Point2i>
		 */
        std::vector<Point2i> convexHull;

		/**
		 * 此对象的凸包,其点存储为等值的索引,而不是 Point2i
		 * @Type: std::vector<int>
		 */
        std::vector<int> indexHull;

		/**
		 * 对象坐标中心的像素坐标
		 * @Type: Vec3f
		 */
        Point2i centerIj = Point2i(INT_MAX, 0);

		/**
		 * 对象在中心的真实坐标中的三维坐标
		 * @Type: Vec3f
		 */
        Vec3f centerXyz = Vec3f(FLT_MAX, 0.0f, 0.0f);

		/**
		 * 对象的平均深度
		 */
        double avgDepth = -1.0;

		/**
		 * 查找对象的质量中心
		 * @Descrip：
		 *
		 * @param 对象的轮廓的像素坐标
		 *
		 * @return [Point2i]: 质心的像素坐标
		 */
        static Point2i findCenter(const std::vector<Point2i> & contour);

		/**
		 * 在聚类的灰度图上执行膨胀和腐蚀形态运算
		 * @Descrip：
		 *
		 * @param [in] erode_sz: 侵蚀内核的大小
		 * @param [in] dilate_sz: dilate核的大小(默认情况下,取相同的值作为 erodeAmt)
		 * @param [in] dilate_first 如果为true,在侵蚀前执行稀释
		 *
		 * @return [void]
		 */
        void morph(int erode_sz, int dilate_sz = -1);

		/**
		 * 计算点云聚类的轮廓
		 * @param [in] xyzMap: 包含聚类边界框中点的输入深度图
		 * @param [in] points: 聚类中的点(绝对坐标)
		 * @param [in] points_xyz: 聚类中点的 xyz 坐标
		 * @param [out] topLeftPt:  群集左上角的点
		 * @param [in] num_points: 群集中的点数
		 * @param thresh the minimum z-coordinate of a point on the depth image below which the pixel will be zeroed out
		 *
		 * @return [void]
		 */
        void computeContour(const cv::Mat & xyzMap, 
            const std::vector<cv::Point> * points, 
            const std::vector<cv::Vec3f> * points_xyz,
            cv::Point topLeftPt, int num_points);

		/**
		 * 从 xyz map中计算此聚类的灰度 z 坐标图像
		 * @param [in] xyzMap: 包含聚类边界框中点的输入深度图
		 * @param [in] points: 聚类中的点(绝对坐标)
		 * @param [in] points_xyz: 聚类中点的xyz坐标
		 * @param [in] topLeftPt: 群集左上角的点
		 * @param [in] num_points: 群集中的点数
		 * @param thresh the minimum z-coordinate of a point on the depth image below which the pixel will be zeroed out
		 *
		 * @return [void]
		 */
        void computeGrayMap(const cv::Mat & xyzMap, 
            const std::vector<cv::Point> * points, 
            const std::vector<cv::Vec3f> * points_xyz,
            cv::Point topLeftPt, int num_points,  int thresh = 0);

		/** 对象/手检测参数 */
        DetectionParams::Ptr params = nullptr;

    private:
		/** 构造函数的辅助函数 */
        void initializeFrameObject(std::shared_ptr<std::vector<Point2i>> points_ij,
            std::shared_ptr<std::vector<Vec3f>> points_xyz,
            const cv::Mat & depth_map,
            DetectionParams::Ptr params = nullptr,
            bool sorted = false,
            int points_to_use = -1
        );
    };
}

#endif  //FRAME_OBJECT_H