/***************************************************************************
 *@File:Visualizer.h
 *@Date:2019-07-26
 *@Description：该头文件主要实现了对可视化类Visualizer进行了定义，对相关函数进行了声明
 *@History:
 ***************************************************************************/
#pragma once
#ifndef VISUALIZER_H
#define VISUALIZER_H

#include "Hand.h"
#include "FramePlane.h"
#include <vector>
#include <opencv2/core.hpp>



namespace ht {
	/**
	 * 包含各种转换和可视化方法的类。
	 */
    class Visualizer
    {
    public:
		/**
		* 可视化xyz maps (每像素点云).
		* @param [in]  xyz_map 输入的点云矩阵
		* @param [out] output  输出图像
		* @param [in]  Max_Depth 最大深度
		* @return void 返回一个CV_8UC3格式的xyz图
		*/
        static void visualizeXYZMap(const cv::Mat &xyz_map, cv::Mat & output, float Max_Depth = 10.0);

		/**
		* 可视化法向图(各点归一化曲面法向量).
		* @param [in] normal_map 输入法向图
		* @param [out] output    输出图像
		* @param [in] resolution 正态图分辨率
		* @return void 返回一个CV_8UC3格式的法向图
		*/
        static void visualizeNormalMap(const cv::Mat &normal_map, cv::Mat & output, 
            int resolution = 3);

		/**
		* 手对象的可视化。
		* @param [in]  background 背景要绘制的基础图像
		* @param [out] output     输出图像
		* @param [in]  hand       手对象
		* @param [in]  display    显示要在手上显示的值; set to >= FLT_MAX to disable
		* @param [in]  touch_planes 可选择接触平面， 当前帧中手可能接触的平面
		* @return void a CV_8UC3 matrix with the hand drawn on it
		*/
        static void visualizeHand(const cv::Mat & background, cv::Mat & output, 
            Hand * hand, double display = FLT_MAX,
            const std::vector<std::shared_ptr<FramePlane> > * touch_planes = nullptr);

		/**
		* 平面回归可视化。
		* @param [in]  input_mat 用于绘制可视化效果的基础xyzmap
		* @param [out] output    输出图像
		* @param [in]  equation  平面方程
		* @param [in]  threshold 回归方程考虑的点允许的最大误差距离(mm)
		* @param [in]  clicked   单击手指当前是否接触回归方程。默认为否
		* @return void 绘制回归平面的矩阵的Cv_8uc3表示
		*/
        static void visualizePlaneRegression(const cv::Mat & input_mat, cv::Mat & output, 
                    std::vector<double> &equation, const double threshold, bool clicked = false);

		/**
		 * 可视化平面上的点。
		 * @param [in] input_mat 输入点云
		 * @param [in] indicies  表示平面上各点的坐标（i，j）
		 */
        static void visualizePlanePoints(cv::Mat &input_mat, std::vector<Point2i> indicies);

    private:
		/**
		 * 可视化是通用矩阵.
		 * @param [in]  input  要被可视化的矩阵
		 * @param [out] output 输出图像
		 * @return void 输入矩阵的cv uc3表示法
		 */
        static void visualizeMatrix(const cv::Mat & input, cv::Mat & output);

    };
}

#endif //VISUALIZER_H
