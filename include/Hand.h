/***************************************************************************
 *@File:Hand.h
 *@Date:2019-07-26
 *@Description：该头文件主要实现了对手类Hand定义,说明了Hand类的继承来源，以及对相关函数进行了声明
 *@History:
 ***************************************************************************/
#pragma once
#ifndef HAND_H
#define HAND_H

#include "FrameObject.h"
#include <opencv2/video/tracking.hpp>
#include <boost/circular_buffer.hpp>
#include <map>

namespace ht {
	/**
	 * @Hand：表示当前帧中可见的手。
	 * 模拟跟踪手部和背景平面物体的示例：
	 * @include HandandPlane.cpp
	 */
    class Hand : public FrameObject
    {
    public:

		// 公共的构造函数

		/**
		 * 默认构造函数（警告：创建无效手）
		 */
        Hand();

		/**
		 * 基于孤立点云构造手实例。
		 * 注: 不在群集中的点在点云中必须有0个Z坐标。
		 * @param[in] cluster_depth_map 包含对象的簇深度点云
		 * @param[in] params            对象/手部检测的参数（如果未指定，则使用默认参数）
		 */
        explicit Hand(const cv::Mat & cluster_depth_map, DetectionParams::Ptr params = nullptr);

		/**
		 * 从点的向量构造手实例。
		 * @param [in] points_ij points_xyz 属于对象的所有点（屏幕坐标中）的点向量
		 * @param [in] depth_map  深度图参考点云。（可以包含此对象之外的点）
		 * @param [in] params     对象/手部检测的参数（如果未指定，则使用默认参数）
		 * @param [in] sorted     排序如果为真，则假定已排序“点”，跳过排序以节省时间.
		 * @param [in] points_to_use “points”中用于对象的点数。默认情况下，使用所有点。
		 */
        Hand(std::shared_ptr<std::vector<Point2i>> points_ij,
            std::shared_ptr<std::vector<Vec3f>> points_xyz,
            const cv::Mat & depth_map,
            DetectionParams::Ptr params = nullptr,
            bool sorted = false,
            int points_to_use = -1
        );

		/**
		 * 销毁手的实例
		 */
        ~Hand();

		// 公共变量以及公共方法

		/**
		 * 获取这只手上的手指数
		 */
        int getNumFingers() const;

		/**
		 * 检查物体是否是手
		 * @param 手部检测参数 (defaults to this->params)
		 */
        bool checkForHand();

		/**
		 * 得到手掌中心的（x，y，z）位置
		 */
        Vec3f getPalmCenter() const;

		/**
		 * 得到手掌中心的（i，j）位置
		 */
        Point2i getPalmCenterIJ() const;

		/**
		 * 获取所有检测到的指尖的（x，y，z）位置
		 */
        std::vector<Vec3f> getFingers() const;

		/**
		 * 获取所有检测到的指尖的（i，j）位置
		 */
        std::vector<Point2i> getFingersIJ() const;

		/**
		 * 获取所有检测到的缺陷（手指根部）的（X、Y、Z）位置
		 */
        std::vector<Vec3f> getDefects() const;;

		/**
		 * 获取所有检测到的缺陷（手指根部）的（i，j）位置
		 */
        std::vector<Point2i> getDefectsIJ() const;;

		/**
		 * 获取手腕两侧的（x，y，z）坐标（[0]左，[1]右）
		 */
        std::vector<Vec3f> getWrist() const;

		/**
		 * 得到手腕两侧的（i，j）坐标（[0]左，[1]右）
		 */
        std::vector<Point2i> getWristIJ() const;

		/**
		 * 获取手上最大（2d）内接圆的半径，以GetPalmCenter（）获取到的点为圆心。
		 */
        double getCircleRadius() const;

		/**
		 * 得到一个指向手的“主导”方向的单位向量，即从中心指向手指。
		 */
        Point2f getDominantDirection() const;

		/**
		 * 得到一个指向手的“主导”方向的三维单位向量
		 */
		Vec3f getDominantDirectionXYZ() const;

		/**
		 * 如果此对象是有效手，则为true (queryFrameObjects/queryFrameHands 将只返回有效手).
		 */
        bool isValidHand() const;

		/** 手对象实例的共享指针*/
        typedef std::shared_ptr<Hand> Ptr;

		// 设置内接圆半径
		void setRadius(const double  r) {
			this->circleRadius = r;
		}

		// 设置内接圆心
		void setCenter(const cv::Point& center) {
			this->centerIj = center;
		}

		/**
		 * 获取此对象的SVM置信度，[0.0，1.0]（较高=更可能是手）
		 */
		double getSVMConfidence() const;

	private:
		

		/**
		 * 如果“手簇”接触底部边缘，则为“真”，表示对象可能连接到用户的身体（手、手臂等）。
		 */
		bool touchingEdge() const;

		/**
		 * 如果手簇接触到屏幕左边缘的下半部分或屏幕下边缘的左半部分，则为true。
		 * 接触边缘意味着物体可能与使用者的身体（手、臂等）相连。
		 */
		bool touchingLeftEdge() const;

		/**
		* 如果手簇接触到屏幕左边缘的下半部分或屏幕下边缘的左半部分，则为true。
		* 接触边缘意味着物体可能与使用者的身体（手、臂等）相连。
		*/
		bool touchingRightEdge() const;

	public:
		/** 用于打标签 **/
		
		/**
		 * 获取指尖坐标按大拇指到小拇指排序,世界坐标
		 * @return [std::vector<Vec3f>]
		 */
		std::vector<Vec3f> getFingersSort() const;

		/**
		 * 获取指尖坐标按大拇指到小拇指排序,像素坐标坐标
		 * @return
		 */
		std::vector<Point2i> getFingersIJSort() const;

		/**
		 * 获取指尖与掌心的连线与手腕的夹角，从小到大排序
		 * @return
		 */
		std::vector<float> getinclude() const;

		/**
		 * 获取所有检测到的缺陷（手指根部）的（X、Y、Z）位置,按序排列
		 * @return
		 */
		std::vector<Vec3f> getDefectsSort() const;

		/**
		 * 获取所有检测到的缺陷（手指根部）的（i，j）位置,按序排列
		 */
		std::vector<Point2i> getDefectsIJSort() const;

		/**
		 * 获取手腕中点的世界坐标
		 * @return
		 */
		Vec3f getwristmid() const;

		/**
		 * 获取手臂最远端左侧的像素坐标
		 */
		Point2i getcontactL_ij() const;

		/**
		 * 获取手臂最远端右侧的像素坐标
		 */
		Point2i getcontactR_ij() const;

		/**
		 * 获取手臂最远端中心点
		 */
		Point2i getarmmidIJ() const;

		/**
		 * 找手腕中点的IJ坐标
		 */
		Point2i getwristmidIJ() const;

        /**
         * 获取左手标签的像素坐标
         */
        std::map<std::string, Point2i> getLHlabelij() const;

        /**
         * 获取左手标签的世界坐标
         */
        std::map<std::string, Vec3f> getLHlabelXYZ() const;

        /**
         * 获取右手标签的像素坐标
         */
        std::map<std::string, Point2i> getRHlabelij() const;

        /**
         * 获取右手标签的世界坐标
         */
        std::map<std::string, Vec3f> getRHlabelXYZ() const;

        /**
         * 获取左手掌心的像素坐标
         */
        std::map<std::string, Point2i> getLHcenterij() const;

        /**
         * 获取左手掌心的世界坐标
         */
        std::map<std::string, Vec3f> getLHcenterXYZ() const;

        /**
         * 获取右手掌心的像素坐标
         */
        std::map<std::string, Point2i> getRHcenterij() const;

        /**
         * 获取右手掌心的世界坐标
         */
        std::map<std::string, Vec3f> getRHcenterXYZ() const;
    private:
		/**
		 * 确定对象是否连接到边缘。
		 */
        void checkEdgeConnected();

		/**
		 * 手中心的（x,y,z）坐标位置
		 */
        Vec3f palmCenterXYZ;

		/**
		 * 手中心的（i,j）坐标位置
		 */
        Point2i palmCenterIJ;

		/**
		 * 检测到所有指尖的（x,y,z）坐标位置
		 */
        std::vector<Vec3f> fingersXYZ;

		/**
		 * 检测到所有指尖的(i,j)坐标位置
		 */
        std::vector<Point2i> fingersIJ;

		/**
		 * 检测到的缺陷点 (手指根部)(x,y,z)坐标位置
		 */
        std::vector<Vec3f> defectsXYZ;

		/**
		 *  检测到的缺陷点 (手指根部)(i,j)坐标位置
		 */
        std::vector<Point2i> defectsIJ;

		/**
		 * 手腕两侧 ([0] 代表左边, [1]代表右边)的(x,y,z)坐标位置
		 */
        std::vector<Vec3f> wristXYZ;

		/**
		 * 手腕两侧 ([0] 代表左边, [1]代表右边)的(i,j)坐标位置
		 */
        std::vector<Point2i> wristIJ;

        /**
         * 左手标签,手指加像素坐标
         */
        std::map<std::string, Point2i> LHlabelij;

        /**
         * 左手标签,手指加世界坐标
         */
        std::map<std::string, Vec3f> LHlabelXYZ;

        /**
         * 右手标签，手指加像素坐标
         */
        std::map<std::string, Point2i> RHlabelij;

        /**
         * 右手标签,手指加世界坐标
         */
        std::map<std::string, Vec3f> RHlabelXYZ;

        /**
         * 左手掌心加像素坐标
         */
        std::map<std::string, Point2i> LHcenterij;

        /**
         * 左手标心加世界坐标
         */
        std::map<std::string, Vec3f> LHcenterXYZ;

        /**
         * 右手心加像素坐标
         */
        std::map<std::string, Point2i> RHcenterij;

        /**
         * 右手心加世界坐标
         */
        std::map<std::string, Vec3f> RHcenterXYZ;

		/**
		 * 最大内接圆半径
		 */
        double circleRadius;

		/**
		 * 存储手的主导方向二维向量
		 */
        Point2f dominantDir;

		/**
		 * 存储手的主导方向三维向量
		 */
		Vec3f dominantDirXYZ;

		/**
		 * SVM分类器分配给这只手的置信值（[0，1]），
		 * 越高越可能是手
		 */
        double svmConfidence;

		/**
		 * 手对象是否有效
		 */
        bool isHand = false;

		/**
		 * 对象是否连接到框架的左边缘。
		 * 边缘连接意味着对象可能连接到用户的身体（手、臂等）
		 */
        bool leftEdgeConnected = false;

		/**
		 * 对象是否连接到框架的右边缘。
		 * 边缘连接意味着对象可能连接到用户的身体（手、臂等）
		 */
        bool rightEdgeConnected = false;

	private:
		/** 打标签所用数据 **/

		/**
		 * 检测到的指尖从大拇指到小拇指的顺序排序,世界坐标
		 */
		std::vector<Vec3f> fingersXYZSort;

		/**
		 * 检测到的指尖从大拇指到小拇指的顺序排序,像素坐标
		 */
		std::vector<Point2i> fingersIJSort;

		/**
		 * 指尖与掌心的连线与手腕的夹角，从小到大排序
		 */
		std::vector<float> included_angle;

		/**
		 * 检测到的缺陷点 (手指根部)(x,y,z)坐标位置,按序排列
		 */
		std::vector<Vec3f> defectsXYZSort;

		/**
		 * 检测到的缺陷点 (手指根部)(i,j)坐标位置,按序排列
		 */
		std::vector<Point2i> defectsIJSort;

		/**
		* 手腕中点的世界坐标
		*/
		Vec3f wristmidXYZ;

		/**
		 * 手腕中心点的像素坐标
		 */
		Point2i wristmidIJ;

		/**
		 * 手臂最远端的中点的像素坐标
		 */
		Point2i armmid;

		/**
		 * 获取手臂最远端左侧的像素坐标
		 */
		Point2i contactL_ij;

		/**
		 * 获取手臂最远端右侧的像素坐标
		 */
		Point2i contactR_ij;
    };
}

#endif //HAND_H