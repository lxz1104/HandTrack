/***********************************************************
 @File:DetectionParams.h
 @Author:lxz
 @Date:2019-7-23
 @Description: 定义了一些检测器的参数
 @History:
************************************************************/
#pragma once
#ifndef DETECTION_PARAMS_H
#define DETECTION_PARAMS_H

#include "stdafx.h"

namespace ht {
	/**
	 * @DetectionParams: 3D对象，平面，收部检测的一些参数
	 *
	 * @Type: Normal
	 */
    class DetectionParams {
    public:
		/**
		 * 默认构造函数
		 * @Descrip:
		 */
        DetectionParams() {}

		// 下面为一些通用参数

		/**
		 * 将图像坐标(i,j)转换为世界坐标(x,y,z)时,深度图像上每个点周围的像素数
		 *
		 * @default: 9
		 */
        int xyzAverageSize = 9;

		/**
		 * 深度图像左/右边缘的像素数量,其中点被视为连接到边缘(忽略边缘的指尖)
		 *
		 * @default: 10
		 */
        int bottomEdgeThresh = 10;

		/**
		 * 深度图像左/右边缘的像素数量,其中点被视为连接到边缘(忽略边缘的指尖)
		 *
		 * @default: 10
		 */
        int sideEdgeThresh = 10;

		// 手部检测用的一些参数

		/**
		 * 同一点云数据中各点之间的最大距离(用于手部检测时的漫水填充，以米为单位)
		 *
		 * @default: 0.004
		 */
        float handClusterMaxDistance = 0.004f;

		/**
		 * 手部结构面积占图像总面积的百分比，小于此数值的对象将被舍弃
		 * 设置0表示忽略
		 *
		 * @default: 0.0167
		 */
        float handClusterMinPoints = 0.0167f;

		/**
		 * 启动手部检测的漫水填充时连续种子点之间的像素数
		 *
		 * @default: 10
		 */
        int handClusterInterval = 20;

		/**
		 * 手部最小表面积(单位: 平方米)
		 *
		 * @default: 0.008
		 */
        double handMinArea = 0.008;

		/**
		 * 手的最大表面积(单位: 平方米)
		 *
		 * @default: 0.053
		 */
        double handMaxArea = 0.053;

		/**
		 * 如果为 true,则手对象必须触摸可见区域的下/左下/右下边缘
		 *
		 * @default: false
		 */
        bool handRequireEdgeConnected = false;

		/**
		 * 左右两侧的最小 y 坐标(作为图像高度的一小部分),以考虑连接到边缘的聚类
		 *
		 * @default: 0.66
		 */
        double handEdgeConnectMaxY = 0.66;

		/**
		 * 是否使用SVM进行分类
		 *
		 * @default: true
		 */
        bool handUseSVM = true;

		/**
		 * 第一个手部对象的最低 SVM 置信度值 (范围: [0,1])
		 *
		 * @default: 0.55
		 */
        float handSVMConfidenceThresh = 0.55f;

		/**
		 * 其他手部对象的最低 SVM 置信度值(范围: [0,1])(仅适用于查询手)
		 *
		 * @default: 0.56
		 */
        double handSVMHighConfidenceThresh = 0.56f;

		/**
		 * 手与相机的最大距离(单位: 米)
		 *
		 * @default: 1.6
		 */
        double handMaxDepth = 0.6;

		/**
		 * 腐蚀轮廓图像以移除小点的数量
		 *
		 * @default: 1
		 */
        int contourImageErodeAmount = 1;

		/**
		 * 分割轮廓图像以去除小间隙的数量
		 *
		 * @default: 4
		 */
        int contourImageDilateAmount = 4;

		/**
		 * 检测手掌中心时手部中心的点与手部范围内的点云数据中位于顶部的点之间的最大距离(单位：米)
		 *
		 * @default: 0.155
		 */
        float centerMaxDistFromTop = 0.155f;

		/**
		 * 从深度图底部边缘的像素数量,在检测接触点时,等值点被视为位于边缘
		 *
		 * @default: 8
		 */
        int contactBotEdgeThresh = 8;

		/**
		 * 从深度图的侧边的像素,其中等值点被视为在边缘 USD,同时检测接触点
		 *
		 * @default: 25
		 */
        int contactSideEdgeThresh = 25;

		/**
		 * 手腕的最小宽度（单位：米）
		 *
		 * @default: 0.030
		 */
        float wristWidthMin = 0.020f;

		/**
		 * 手腕的最大宽度（单位：米）
		 *
		 * @default: 0.085
		 */
        float wristWidthMax = 0.085f;

		/**
		 * 从手腕到手部中心的最大距离（单位：米）
		 *
		 * @default: 0.075
		 */
        double wristCenterDistThresh = 0.075;

		/**
		 * 最小手指长度（单位：米）
		 *
		 * @default: 0.014
		 */
        double fingerLenMin = 0.014;

		/**
		 * 最大手指长度（单位：米）
		 *
		 * @default: 0.13
		 */
        double fingerLenMax = 0.15;

		/**
		 * 两个指尖之间的最小距离
		 *
		 * @default: 0.01
		 */
        double fingerDistMin = 0.01;

		/**
		 * 任意手指的(值尖y值  -  缺陷点y值)/abs(值尖x值 - 缺陷点x值) 的最小值。
		 * 用于过滤掉一些部确定的手指
		 *
		 * @default: -1.0
		 */
        double fingerDefectSlopeMin = -1.0;

		/**
		 * 任意手指的(值尖y值  -  缺陷点y值)/abs(值尖x值 - 缺陷点x值) 的最大值。
		 * 用于过滤掉一些部确定的手指
		 *
		 * @default: -0.45
		 */
        double fingerCenterSlopeMin = -0.45;

		/**
		 * 手部点云轮廓的最小曲率,使用指尖旁边的点估计
		 *
		 * @default: 0.05
		 */
        double fingerCurveNearMin = 0.05;

		/**
		 * 集群轮廓的最小曲率,使用距离约为缺陷一半的点进行估计
		 *
		 * @default: 0.16
		 */
        double fingerCurveFarMin = 0.16;

		/**
		 * 仅检测到一根手指时使用的最小手指长度
		 *
		 * @default: 0.04
		 */
        double singleFingerLenMin = 0.04;

		/**
		 * 仅检测到一根手指时使用的最大手指长度
		 *
		 * @default: 0.11
		 */
        double singleFingerLenMax = 0.11;

		/**
		 * 由指尖和相邻缺陷形成的最小角度
		 *
		 * @default: 0.06
		 */
        double singleFingerAngleThresh = 0.06;

		/**
		 * 缺陷的最大角度
		 *
		 * @default: 0.70 * PI
		 */
        double defectMaxAngle = 0.70 * PI;

		/**
		 * 将当前缺陷的起始点视为手指候选值,与上一个缺陷的终点保持的最小距离
		 *
		 * @default: 0.02
		 */
        double defectMinDist = 0.02;

		/**
		 * 从缺陷的远点到中心的最小距离(单位：米)
		 *
		 * @default: 0.01
		 */
        double defectFarCenterMinDist = 0.01;

		/**
		 * 从缺陷的远点到中心的最大距离(单位：米)
		 *
		 * @default: 0.105
		 */
        double defectFarCenterMaxDist = 0.105;

		/**
		 * 缺陷起点和终点之间的最小距离 (单位：米)
		 *
		 * @default: 0.01
		 */
        double defectStartEndMinDist = 0.01;

		/**
		 * 缺陷点 y值低于中心点y值的最大值
		 *
		 * @default: 30
		 */
        int defectMaxYFromCenter = 30;

		/**
		 * 质心、缺陷和手指之间的最小角度
		 *
		 * @default: 0.20 * PI
		 */
        double centroidDefectFingerAngleMin = 0.20 * PI;

		/**
		 * 手与平面之间的最小距离平方(单位：平方米)。
		 * 在手部检测过程中不考虑靠近平面的点,以便将手与平面面隔离。
		 *
		 * @default: 0.000075
		 */
        double handPlaneMinSqrDist = 0.000075;

		// 平面检测用到的参数

		/**
		 * 平面检测中使用的法线图的分辨率
		 *
		 * @default: 3
		 */
        int normalResolution = 10;

		/**
		 * 两个相邻点的表面法向量之间的最大差值,以将它们视为位于同一平面上(在平面检测期间用于漫水填充)
		 *
		 * @default: 0.06
		 */
        float planeFloodFillThreshold = 0.60f;

		/**
		 * 执行回归之前从平面上移除的异常点分数
		 *
		 * @default: 0.2
		 */
        float planeOutlierRemovalThreshold = 0.02f;

		/**
		 * 组合平面上的最小值(# points / # total points on screen / normal resolution^2)。较小的平面被丢弃。
		 *
		 * @default: 0.0350
		 */
        float planeMinPoints = 0.0350f;

		/**
		 * 组合平面的最小表面积(单位：平方米)
		 *
		 * @default: 0.0120
		 */
        double planeMinArea = 0.0120;

		/**
		 * 不符合此条件的组合平面上的最小值(# equation inliers / # total points on screen * normal resolution^2)将被丢弃。
		 *
		 * @default: 0.0350
		 */
        float planeEquationMinInliers = 0.0350f;

		/**
		 * 较大平面组件中的最小值 (# points / # total points on screen * normal resolution^2)
		 *
		 * @default: 0.0050
		 */
        float subplaneMinPoints = 0.0050f;

		/**
		 * 较大平面组件的最小表面积(单位：平方米)
		 *
		 * @default: 0.005
		 */
        double subplaneMinArea = 0.005;

		/**
		 * 两个"子平面"方程之间的最小规范 (r^2),以考虑它们分开平面。
		 * 如果规范较低,则两者合并为一个较大的平面对象。
		 *
		 * @default: 0.0025
		 */
        double planeCombineThreshold = 0.0025;

		/** 指向 DetectionParams 实例的共享指针 */
        typedef std::shared_ptr<DetectionParams> Ptr;

		/** 使用默认参数初始化的 DetectionParams 实例 */
        static DetectionParams::Ptr DEFAULT;

		/** 创建新的 DetectionParams 实例,用智能指针包装并初始化使用默认参数值 */
        static DetectionParams::Ptr create() {
            return std::make_shared<DetectionParams>();
        }
    };
}

#endif //DETECTION_PARAMS_H