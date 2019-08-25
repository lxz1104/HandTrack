/**
 *      ┌─┐       ┌─┐
 *   ┌──┘ ┴───────┘ ┴──┐
 *   │                 │
 *   │       ───       │
 *   │  ─┬┘       └┬─  │
 *   │                 │
 *   │       ─┴─       │
 *   │                 │
 *   └───┐         ┌───┘
 *       │         │
 *       │         │
 *       │         │
 *       │         └──────────────┐
 *       │                        │
 *       │                        ├─┐
 *       │                        ┌─┘
 *       │                        │
 *       └─┐  ┐  ┌───────┬──┐  ┌──┘
 *         │ ─┤ ─┤       │ ─┤ ─┤
 *         └──┴──┘       └──┴──┘
 *                工具函数
 *                请勿乱动
 */
 /***********************************************************
 @File: Util.h
 @Author: LXZ
 @Date: 2019-07-01
 @Description: 几何计算类库
 @History: NULL
 ************************************************************/
#pragma once
#ifndef UTIL_H
#define UTIL_H

#include "stdafx.h"

#include <opencv2/opencv.hpp>
#include <boost/smart_ptr.hpp>
#include <vector>
#include <string>


namespace ht {
    /**
     * 工具函数类库
     */
    namespace util
    {
		/**
		 * 计算两个向量之间的夹角
		 * @param [in] pt1 向量1的起点
		 * @param [in] pt2 向量1的终点
		 * @param [in] pt3 向量2的起点
		 * @param [in] pt4 向量2的终点
		 * @return [float]
		 */
		double getAngelOfTwoVector(const Point2i& pt1, const Point2i& pt2, const Point2i& pt3, const Point2i& pt4);

		/**
		 * 计算两个向量之间的夹角
		 * @param [in] pt1 向量1的起点
		 * @param [in] pt2 向量1的终点
		 * @return [float] 角度
		 */
		double distanOfTwoPointIJ(const Point2i& pt1, const Point2i& pt2);

        /**
		 * 使用分隔符将字符串拆分为多个部分
		 * @param string_in 要拆分的字符串
		 * @param delimiters 分隔符，默认为空格
		 * @param ignore_empty 如果为真，则忽略空字符串
		 * @param trim 如果为真，则在拆分后清除每个字符串中的空格
		 * @return 保存拆分结果的向量容器
		 */
        std::vector<std::string> split(const std::string & string_in,
            char const * delimiters = " ", bool ignore_empty = false, bool trim = false);

		/**
		 * 使用分隔符将字符串拆分为多个部分
		 * @param string_in 要拆分的字符串
		 * @param delimiters 分隔符，默认为空格
		 * @param ignore_empty 如果为真，则忽略空字符串
		 * @param trim 如果为真，则在拆分后清除每个字符串中的空格
		 * @return 保存拆分结果的向量容器
		 */
        std::vector<std::string> split(const char * string_in, char const * delimiters = " ",
            bool ignore_empty = false, bool trim = false);


		/** 修剪字符串左端的空格，换行符等 */
        void ltrim(std::string & s);

        /** 修剪字符串右端的空格，换行符等 */
        void rtrim(std::string & s);

        /** 修剪字符串两端的空格，换行符等 */
        void trim(std::string & s);

        /** 将字符串转换为大写 */
        void upper(std::string & s);

        /** 将字符串转换为小写 */
        void lower(std::string & s);

		/**
         * 根据给定的数量自动将字符串复数（末尾添加's'）。
         * @param str [in]			字符串（不会被修改）
         * @param num [in]			数量（如果不是1，则添加“s”）。
         * @return [std::string]	复数字符串
         */
        template<class T>
        std::string pluralize(std::string str, T num);

        /**
        * 生成随机的RGB颜色
        * @return Vec3b格式的随机RGB颜色
        */
        Vec3b randomColor();

		/*
		* 得到（x1，y1）和（x2，y2）之间的L2范数（欧氏距离）
		* @param [in] pt1 (x1, y1)
		* @param [in] pt2 (x2, y2)
		* @return [float] 两点之间的欧几里得距离
		*/
        template<class T>
        float euclideanDistance(const cv::Point_<T> & pt1, const cv::Point_<T> & pt2);

        /**
        * 得到两个三维点之间的欧氏距离
        * @param pt1 第一个点
        * @param pt2 第二个点
        * @return [template_type] 两点之间的欧氏距离
        */
        template<class T>
        T euclideanDistance(const cv::Vec<T, 3> & pt1, const cv::Vec<T, 3> & pt2);

		/**
		 * 计算点与直线上任意点之间的距离平方
		 * @param v the 输入点
		 * @param a, b 直线上两点，用于确定直线方程
		 * @param cv_norm_type 要使用的标准化类型（cv::NORM_XYZ）；默认为L2范数的平方
		 *
         */
        template<class Param_T>
        float pointLineDistance(const cv::Point_<Param_T> & p, const cv::Point_<Param_T> & a, const cv::Point_<Param_T> & b, int cv_norm_type = cv::NORM_L2SQR);

		/**
		 * 计算点与直线段上任意点之间的平方距离
		 * @param v 输入点
		 * @param a, b 线段的两端点
		 * @param [float] cv_norm_type 要使用的标准化类型（cv::NORM_XYZ）；默认为L2范数的平方
		 */
        template<class Param_T>
        Param_T pointLineDistance(const cv::Vec<Param_T, 3> & p, const cv::Vec<Param_T, 3> & a, const cv::Vec<Param_T, 3> & b, int cv_norm_type = cv::NORM_L2SQR);

		/**
		 * 计算点与直线段上任意点之间的平方距离
		 * @param v 输入点
		 * @param a, b 线段的两端点
		 * @param [float] cv_norm_type 要使用的标准化类型（cv::NORM_XYZ）；默认为L2范数的平方
		 */
        template<class Param_T>
        float pointLineSegmentDistance(const cv::Point_<Param_T> & p, const cv::Point_<Param_T> & a, const cv::Point_<Param_T> & b, int cv_norm_type = cv::NORM_L2SQR);

		/**
		 * 计算点与直线段上任意点之间的平方距离
		 * @param v 输入点
		 * @param a, b 线段的两端点
		 * @param cv_norm_type 要使用的标准化类型（cv::NORM_XYZ）；默认为L2范数的平方
		 */
        template<class Param_T>
        Param_T pointLineSegmentDistance(const cv::Vec<Param_T, 3> & p, const cv::Vec<Param_T, 3> & a, const cv::Vec<Param_T, 3> & b, int cv_norm_type = cv::NORM_L2SQR);

		/**
		* 计算平面定义为：ax+by-z+c=0 的点与平面之间的平方距离。
		* @param pt		三维点
		* @param eqn	平面方程参数：[A、B、C]
		* @return		L2范数平方（m^2）
		*/
        template<class T> T pointPlaneDistance(const cv::Vec<T, 3> & pt, const cv::Vec<T, 3> & eqn);

		/**
		 * 计算点与平面之间的欧几里得距离，平面定义为：ax+by-z+c=0
		 * @param pt 三维点
		 * @param a, b, c 平面方程参数
		 * @return 欧几里得距离（单位米）
		 */
        template<class T> T pointPlaneDistance(const cv::Vec<T, 3> & pt, T a, T b, T c);

        /**
        * 计算平面定义为：ax+by-z+c=0的点与平面之间的欧几里得距离平方
        * @param pt		三维点
        * @param eqn	平面方程参数：[A、B、C]
        * @return		L2范数平方（m^2）
        */
        template<class T> T pointPlaneSquaredDistance(const cv::Vec<T, 3> & pt, const cv::Vec<T, 3> & eqn);

		/**
		 * 计算平面定义为：ax+by-z+c=0 的点与平面之间的欧几里得距离平方
		 * @param pt 三维点
		 * @param a  平面方程参数
		 * @param b	 平面方程参数
		 * @param c  平面方程参数
		 * @return	 L2范数平方（m^2）
		 */
        template<class T> T pointPlaneSquaredDistance(const cv::Vec<T, 3> & pt, T a, T b, T c);

		/**
         * 估计深度图上一点周围每像素的欧几里得距离
         * @param xyz_map	输入深度图
         * @param pt		给定点二维坐标
         * @param radius	查找半径
         * @return			每像素距离
         */
        double euclideanDistancePerPixel(cv::Mat xyz_map, Point2i pt, int radius);

		/**
		 * 删除具有给定标记的img上的点
		 * @param [out] img		要操作的图像
		 * @param [in]  points	要设置为0的（i，j）坐标
		 */
        void removePoints(cv::Mat & img, const std::vector<Point2i> & points);

		/**
		 * 将“image”上对应于“ref_cloud”拟合“plane_equation”方程上的点的所有像素归零
		 * @param [in, out]				xyz_map 输入点云
		 * @param [in] plane_equation	平面方程
		 * @param threshold				平面的厚度，即从平面到删除点的最大距离
		 * @param mask					可选,掩膜矩阵，用于加快运算速度
		 * @param mask_color			可选,其值必须等于要从点云中删除的点的对应索引处的“mask_color”
		 */
        template <class T>
        void removePlane(const cv::Mat & ref_cloud, cv::Mat & image, const Vec3f & plane_equation,
                         double threshold, cv::Mat * mask = nullptr, uchar mask_color = 0);

		/**
		* 在一个点周围计算所有非零值的平均
		* @param	img 要使用的基础图像
		* @param	pt 兴趣点
		* @param	radius 用于计算平均值的相邻点数
		* @return	兴趣点周围的平均值（x，y，z）
		*/
        Vec3f averageAroundPoint(const cv::Mat & img, const Point2i & pt, int radius = 5);

		/**
		 * 通过计算两个向量对邻近点的交叉积，求出深度图像上某一点的近似曲面法向量。
		 * @param img		基础深度图像
		 * @param pt		兴趣点
		 * @param radius	用于计算交叉积的向量长度
		 * @return [Vec3f]	兴趣点处的归一化后的法向量（面向观察者）
		 */
        Vec3f normalAtPoint(const cv::Mat & img, const Point2i & pt, int radius = 3);

		/**
		 * 通过考虑每个点的“influence”，消除点云中的异常值
		 * @param [in]  data		输入点云
		 * @param [out] output		输出点（应提前初始化为空容器)
		 * @param [in]  thresh		要消除的阈值
		 * @param [in]  data_aux	可选，辅助二维输入向量与“data”一起重新排序
		 * @param [out] output_aux	可选，辅助二维输出向量与“data”一起重新排序
		 * @param [in]  num_points	要使用的“数据”中的点数，默认为全部。
		 * @return [int] 输出点数 = floor('num_points' * (1.0 - 'thresh'))
		 */
        int removeOutliers(const std::vector<Vec3f> & data, 
                               std::vector<Vec3f> & output, double thresh = 0.3, 
                               const std::vector<Point2i> * data_aux = nullptr,
                               std::vector<Point2i> * output_aux = nullptr,
                               int num_points = -1);

		/**
		 * 对一组点执行基于RANSAC的鲁棒平面回归
		 * @param points [in]		三维点的向量容器（必须至少包含3个点）
		 * @param thresh			作为一个考虑点的最大L2范数平方（r^2）
		 * @param iterations		RANSAC迭代次数
		 * @param num_points		要使用的输入点数（最小值3）。默认情况下，使用全部。
		 * @return [cv::Vec<T, 3>]	通过给定数据点的最佳拟合平面，形式为（平面方程）：[a,b,c]: 0 = ax + by - z + c
		 */
        template<class T>
        cv::Vec<T, 3> ransacFindPlane(const std::vector<cv::Vec<T, 3> > & points,
            T thresh, int iterations = 300, int num_points = -1);

		/**
		* 计算与点云中每个点相关联的曲面法向量
		* @param [in]  xyz_map	   输入点云
		* @param [out] output_mat  输出正态矩阵
		* @param [in]  normal_dist 用来采样表面向量以计算法向的距离
		* @param [in]  resolution  输出法向矩阵的像素分辨率
		* @param [in]  fill_in     如果为真，则通过复制填充输出矩阵的所有像素，否则，只填充间隔为“分辨率”的像素。
		*/
        void computeNormalMap(const cv::Mat & xyz_map, cv::Mat & output_mat,
            int normal_dist = 6, int resolution = 2, bool fill_in = true);

		/**
		 * 确定（x，y）是否是矩阵中的非零点
		 * @param xyz_map 深度数据
		 * @param x 该点的X坐标
		 * @param y 该点的Y坐标
		 * @return true if (x.y) is non-zero
		 */
        bool isMember(cv::Mat xyz_map, int x, int y);

		/**
		 * 找到深度图像的平均深度
		 * @param xyz_map 深度数据
		 * @return [double] 平均深度（单位：米）
		 */
        double averageDepth(cv::Mat xyz_map);

		/**
		 * 找到点云的质心
		 * @param xyz_map 输入点云 
		 * @return [Point2i] 质心的（X,Y）坐标
		 */
        Point2i findCentroid(cv::Mat xyz_map);

		 /**
		  * 从种子点（x，y）开始在深度图上执行单个漫水填充
		  * @param [in]      xyz_map			输入点云
		  * @param [in]  	 seed				种子点
		  * @param [in]      thresh				邻近点之间允许的最大欧几里得距离
		  * @param [out]     output_ij_points  （可选）指向向量的指针，用于存储洪泛填充访问的点的二维坐标。
		  * @param [out]     output_xyz_points （可选）指向矢量的指针，用于存储洪泛填充访问的点的三维坐标。
		  * @param [out]     output_mask		与“xyz_map”类型相同的可选输出图像，其中由漫水填充访问的所有像素从“xyz_map”复制，
		  *											其他像素设置为黑色（设置为空以禁用）
		  * @param [in]      interval1			以像素为单位到图像上相邻位置的距离（例如，如果为2，则相邻位置为（-2、0）、（0、2）等）；默认为1
		  * @param [in]      interval2		   （高级选项）可选附加间隔（设置0表示未使用），仅适用于向上/向下填充
		  * @param [in]		 interval2_thresh  （高级选项）可选间隔,更换参数"thresh"的距离
		  * @param [in, out] visited_map       （高级选项）辅助矩阵（CV_8U）
		  *											对于正在访问的点（1）、已访问的点（0）或默认情况下尚未访问的点（255）的记录，
		  *											分配一个临时矩阵以在洪水填充期间使用。
		  * @param [in]	     cosine				如果为真，则使用余弦相似性而不是欧几里得距离
		  *
		  * @return [int] 组件中的点数
		  */
        int floodFill(const cv::Mat & xyz_map, const Point2i & seed,
            float thresh = 0.005f, 
            std::vector <Point2i> * output_ij_points = nullptr,
            std::vector <Vec3f> * output_xyz_points = nullptr,
            cv::Mat * output_mask = nullptr,
            int interval1 = 1, int interval2 = 0, float interval2_dist = 0.05f, 
            cv::Mat * visited_map = nullptr, bool cosine = false);

		/**
		 * 如果y轴朝上，计算以弧度表示的角度“pointij”是从原点开始的，从（0，1）开始逆时针移动。
		 * @param pointij 输入点坐标
		 * @return angle 从（0，1）到原点到输入点的角度
		 */
        double pointToAngle(const Point2f & pointij);

		/**
		 * 如果Y轴朝上，则从（0，1）点使用单位大小的“angle”弧度逆时针计算该点。
		 * @param angle 输入角度
		 * @return [Point2f] 单位幅度“angle”弧度的点，（0，1）的逆时针方向
		 */
        Point2f angleToPoint(float angle);

		/**
		* 计算像素坐标中两点之间的弧度角度（如果没有设置顶点，默认为原点）
		* @param a 第一个点坐标
		* @param b 第二个点坐标
		* @param center 可选，夹角的顶点，缺省值为原点( a-center-b )
		* @return [double] a,b两点之间的夹角
		*/
        double angleBetweenPoints(const Point2f & a, const Point2f & b, const Point2f & center = Point2f(0, 0));

		/**
		* 用一个点的大小除它的模长，使它标准化
		* @param [in] pt （x,y）坐标值
		* @return [Point2f] 标准化之后的点（如果pt是（0，0），返回pt而不修改它）
		*/
        Point2f normalize(const Point2f & pt);

        /**
        * 标准化化矢量并确保它指向查看器（-Z）
        * @param vec (x,y,z)向量
        * @return normalized vector
        */
        Vec3f normalize(const Vec3f & vec);

		/**
         * 计算一个点的大小。
         * @param pt 输入点
         * @return [double] 点的大小
         */
        template <class T>
        double magnitude(cv::Point_<T> pt);

		/**
         * 计算一个点的大小
         * @param pt 输入点
         * @return [double] 点的大小
         */
        template <class T>
        double magnitude(cv::Point3_<T> pt);

		/**
		 * 计算一个点的大小
		 * @param pt 输入点
		 * @return [double] 点的大小
		 */
        template <class T, int n>
        double magnitude(cv::Vec<T, n> pt);

		/**
		* 计算向量的范数
		* @param [in] pt 输入点
		* @param [in] cv_norm_type 要使用的范数类型（cv::NORM_XYZ）。默认值为L2的平方。
		* @return [double] 点的范数
		*/
        template <class Param_T>
        double norm(const cv::Point_<Param_T> & pt, int cv_norm_type = cv::NORM_L2SQR);

		/**
		* 计算向量的范数
		* @param [in] pt 输入点
		* @param [in] cv_norm_type 要使用的范数类型（cv::NORM_XYZ）。默认值为L2的平方。
		* @return [double] 点的范数
		*/
        template <class Param_T>
        double norm(const cv::Point3_<Param_T> & pt, int cv_norm_type = cv::NORM_L2SQR);

		/**
		* 计算向量的范数
		* @param [in] pt 输入点
		* @param [in] cv_norm_type 要使用的范数类型（cv::NORM_XYZ）。默认值为L2的平方。
		* @return [double] 点的范数
		*/
        template <class Param_T, int n>
        double norm(const cv::Vec<Param_T, n> & pt, int cv_norm_type = cv::NORM_L2SQR);

        /**
         * 计算两个三维矢量之间的角度
         * @param a 三维矢量a
		 * @param b 三维矢量b
         * @param center 可选，在计算角度之前用向量“center”减去矢量a和矢量b,缺省值为(0, 0, 0)
         * @return [double] 矢量a和矢量b的夹角
         */
        double angleBetween3DVec(Vec3f a, Vec3f b, Vec3f center = Vec3f(0, 0, 0));

		/**
         * 检查点是否在图像内
         * @param img 图像
         * @param pt 要检测的点
		 * @return [bool] : true，位于图像内；flase，不在图像内
         */
        bool pointInImage(const cv::Mat & img, const Point2i pt);

        /**
         * 检查点是否在矩形边界内
         * @param rect 矩形框
         * @param pt 所检查的点
		 * @return [bool] : true，位于矩形框内；flase，不在矩形框内
         */
        bool pointInRect(const cv::Rect & rect, const Point2i pt);

		/**
         * 检查点是否位于矩形（左上角点为(0,0)，大小为[size]）的边缘，
         * @param size 矩形的大小
         * @param pt 待检查的点
         * @param margin_tb 上边缘/底部边缘被视为边界的最大像素数
         * @param margin_lr 左/右边缘被视为边界的最大像素数
         */
        bool pointOnEdge(const cv::Size size, const Point2i pt,
            int margin_tb = 30, int margin_lr = 30);

		/**
         * 检查点是否位于矩形边缘
         * @param rect 矩形边框
         * @param pt 要检查的点
         * @param margin_tb 边缘上被视为靠近边框上/下边缘的最大像素数
         * @param margin_lr 边缘上被视为靠近边框左/右边缘的最大像素数
         * @param scale 点坐标相对于图像比例的比例
         */
        bool pointOnEdge(const cv::Rect rect, const Point2i pt,
            int margin_tb = 30, int margin_lr = 30);

		/**
         * 检查点是否位于图像边缘
         * @param [in] img 图像
         * @param pt the 要检查的点
         * @param margin_tb 边缘上被视为靠近边框上/下边缘的最大像素数
         * @param margin_lr 边缘上被视为靠近边框左/右边缘的最大像素数
         */
        bool pointOnEdge(const cv::Mat & img, const Point2i pt,
            int margin_tb = 30, int margin_lr = 30);

		/**
         * 计算由三个顶点定义的三角形的面积,三维坐标
         * @param a 第一个顶点
         * @param b 第二个顶点
         * @param c 第三个顶点
         * @return [float] 三角形面积，以平方米为单位
         */
        float triangleArea(Vec3f a, Vec3f b, Vec3f c = Vec3f(0, 0, 0));

		/**
         * 计算由四个顶点定义的四边形的面积
         * @param pts 储存四个顶点的容器
         * @return [float] 四边形面积，以平方米为单位
         */
        float quadrangleArea(Vec3f pts[4]);

		/**
         * 计算深度图上所有可见集群的近似表面积。
         * @param [in] 深度图像输入深度图像。将排除Z坐标为0的所有点。
         * @return [double] 表面积，平方米为单位
         */
        double surfaceArea(const cv::Mat & depthMap);

		/**
         * 计算包含指定点的集群的近似表面积
         * @param [in] frame_size 该帧图像的大小
         * @param [in] points_ij 集群中的所有IJ坐标点(默认判定为已排序)
         * @param [in] points_xyz 集群中的所有XYZ坐标点(默认判定为已排序)
         * @param [in] sorted 如果为真，对所有点进行重新排序
         * @param [in] cluster_size 此群集中的点数。默认情况下，使用“集群”向量中的所有点。
         * @return [double]表面积，单位：平方米
         */
        double surfaceArea(const cv::Size & frame_size,
            const std::vector<Point2i> & points_ij,
            const std::vector<Vec3f> & points_xyz,
            int cluster_size = -1);

		/**
         * 使用相邻点之间半径近似最小的圆来近似深度图簇的表面积。
         * @param shape 深度数据
         * @return [double] 表面积，单位：平方米
         */
        double surfaceAreaCircle(cv::Mat shape);

		/**
         * 用相邻点之间使用三角法近似深度图簇的表面积
         * @param shape 深度数据
         * @return [double] 表面积，单位：平方米
         */
        double surfaceAreaTriangulate(cv::Mat shape);

		/**
		* 计算一组二维点的直径
		* 输出最远点的索引并返回它们之间的距离
		* @param [in] points 输入点集
		* @param [out] a 最远点的索引
		* @param [out] b 最远点的索引
		* @return [double] 两点之间的二维欧几里得范数（距离的平方）
		*/
        double diameter(const std::vector<cv::Point> & points, int & a, int & b);

		/**
         * 对基数点按y和x坐标对点进行排序
         * @param points[in] 待排序点的容器
         * @param width 深度图像的整体宽度
		 * @param height 深度图像的整体高度
         * @param num_points 要使用的“points”中的点数。默认情况下，使用全部。
         * @param points_xyz 指向关联XYZ点的向量容器的指针。如果提供，与IJ点一起排序
         */
        void radixSortPoints(std::vector<Point2i> & points,
            int width, int height,
            int num_points = -1,
            std::vector<Vec3f> * points_xyz = nullptr);

		 /**
		  * 在起始点的螺旋中寻找一个接近[starting_point]的集群中的非零点
		  * 用于捕捉计算质心等，用于确定点是否位于集群上.
		  * 如果[starting_point]未非零点，直接返回[starting_point]的值
		  * @param cluster 代表集群的xyz Map
		  * @param starting_point 搜索起始点
		  * @param max_attempts 在中断搜索并简单地返回起始点之前要查找的最大点数
		  * @return 在螺旋查找时遇到的第一个接近[starting_point]的非零点且位于[cluster]上，
		  */
        Point2i nearestPointOnCluster(const cv::Mat cluster, Point2i starting_point, int max_attempts = 500);

		 /**
		  * 查找轮廓内最大内接圆的中心和半径
		  * @param[in] contour 集群轮廓点（像素坐标）
		  * @param[in] xyz_map 输入点云
		  * @param bounds 对象的边界
		  * @param top_point 顶点，在 xyz 坐标或集群中
		  * @param top_dist_thresh 圆的中心与集群顶点之间的最大距离
		  * @param[out] radius 圆半径的输出
		  * @param samples 从轮廓中采样的近似点数
		  * @return 圆的中心点
		  */
         Point2f largestInscribedCircle(const std::vector<Point2i> & contour,
            const cv::Mat & xyz_map, 
            const cv::Rect bounds,
            const Vec3f top_point = Vec3f(0, 0,0),
            float top_dist_thresh = FLT_MAX,
            double * radius = nullptr, int samples = 100);

		 /**
		 * 求指定点附近轮廓的近似曲率（1/R）
		 * @param[in] contour 输入轮廓
		 * @param index 轮廓内目标点的索引
		 * @param float 从目标点到采样导数点的二维欧几里德距离
		 * @param max_tries  查找边点的最大尝试次数。设置为-1以禁用。
		 * @return [float] 指定点的弧度
		 */
        float contourCurvature(const std::vector<Point2i> & contour, int index,
            float radius = 30.0, int max_tries = 60);

		/**
         * 在指定点附近以弧度求轮廓的曲率角
         * @param[in] contour 输入轮廓
         * @param index 轮廓内目标点的索引
         * @param start 从目标点到起始点平均曲率的点数
         * @param end 从目标点到结束点平均曲率的点数
         * @return angle 点的弧度
         */
        float contourLocalAngle(const std::vector<Point2i> & contour, int index,
            int start = 2, int end = 5);

		/** 
		 * 在XYZMap上找到给定方向上给定点和最远点之间的二维距离（以像素为单位）。
         * @param xyz_map XYZMap（点云数据）
         * @param center 中心点
         * @param angle 如果Y轴朝上，则由与（0，1）的逆时针角度指定的方向所成的角度。
         * @param angle_offset 要添加到角度的偏移角度（默认为0.0）
         */
        float radiusInDirection(const cv::Mat & xyz_map, const Point2i & center,
                             double angle, double angle_offset = 0.0);


        /** 查找分类器路径 */
        std::string resolveRootPath(const std::string & root_path);

		/**
         * 比较两个点（Point、Point2f、Vec3i、Vec3f）的大小；比较顺序：首先是x，然后是y，然后是z（如果可用）
		 * 用于对点进行排序。
         */
        template<class T>
        class PointComparer {
        public:
            PointComparer(bool reverse = false, bool compare_y_then_x = false)
                : reverse(reverse), compare_y_then_x(compare_y_then_x){ }

            // 比较两点的大小。如果a小于b，则返回true。
            bool operator()(T a, T b);
        private:
            bool reverse = false, compare_y_then_x = false;
        };
		template<class T>
		inline bool PointComparer<T>::operator()(T a, T b)
		{
			return a >= b;
		}
};
}

#endif // !UTIL_H