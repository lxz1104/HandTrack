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
		 * @return [float]
		 */
		double distanOfTwoPointIJ(const Point2i& pt1, const Point2i& pt2);

        /**
        * Splits a string into components based on a delimiter
        * @param string_in string to split
        * @param delimiters c_str of delimiters to split at
        * @param ignore_empty if true, ignores empty strings
        * @param trim if true, trims whitespaces from each string after splitting
        * @return vector of string components
        */
        std::vector<std::string> split(const std::string & string_in,
            char const * delimiters = " ", bool ignore_empty = false, bool trim = false);

        /**
        * Splits a string into components based on a delimiter
        * @param string_in string to split
        * @param delimiters c_str of delimiters to split at
        * @param ignore_empty if true, ignores empty strings
        * @param trim if true, trims whitespaces from each string after splitting
        * @return vector of string components
        */
        std::vector<std::string> split(const char * string_in, char const * delimiters = " ",
            bool ignore_empty = false, bool trim = false);


        /** Trims whitespaces (space, newline, etc.) in-place from the left end of the string */
        void ltrim(std::string & s);

        /** Trims whitespaces (space, newline, etc.) in-place from the right end of the string */
        void rtrim(std::string & s);

        /** Trims whitespaces (space, newline, etc.) in-place from both ends of the string */
        void trim(std::string & s);

        /** Convert a string to upper case in-place */
        void upper(std::string & s);

        /** Convert a string to lower case in-place */
        void lower(std::string & s);

        /**
         * automatically pluralize a string (add 's') base on a given quantity.
         * @param str [in] the string (will not be modified)
         * @param num the quantity (adds 's' if this is not 1)
         * @return pluralized string
         */
        template<class T>
        std::string pluralize(std::string str, T num);

        /**
        * Generates a random RGB color.
        * @return random RGB color in Vec3b format
        */
        Vec3b randomColor();

        /**
        * Get the color at index 'index' of the built-in palette
        * Used to map integers to colors.
        * @param color_index index of color
        * @param bgr if true, color is returned in BGR order instead of RGB (default true)
        * @return color in Vec3b format
        */
        Vec3b paletteColor(int color_index, bool bgr = true);

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
        * Compute the squared distance between a point and any point on a  line
        * @param v the point
        * @param a, b two points on the line
        * @param cv_norm_type type of norm to use (cv::NORM_XYZ); defaults to square of L2
        */
        template<class Param_T>
        float pointLineDistance(const cv::Point_<Param_T> & p, const cv::Point_<Param_T> & a, const cv::Point_<Param_T> & b, int cv_norm_type = cv::NORM_L2SQR);

        /**
        * Compute the squared distance between a point and any point on a line
        * @param v the point
        * @param a, b two points on the line
        * @param cv_norm_type type of norm to use (cv::NORM_XYZ); defaults to square of L2
        */
        template<class Param_T>
        Param_T pointLineDistance(const cv::Vec<Param_T, 3> & p, const cv::Vec<Param_T, 3> & a, const cv::Vec<Param_T, 3> & b, int cv_norm_type = cv::NORM_L2SQR);

        /**
        * Compute the squared distance between a point and any point on a line segment
        * @param v the point
        * @param a, b end points of the line segment
        * @param cv_norm_type type of norm to use (cv::NORM_XYZ); defaults to square of L2
        */
        template<class Param_T>
        float pointLineSegmentDistance(const cv::Point_<Param_T> & p, const cv::Point_<Param_T> & a, const cv::Point_<Param_T> & b, int cv_norm_type = cv::NORM_L2SQR);

        /*
        * Compute the squared distance between a point and any point on a line segment
        * @param v the point
        * @param a, b end points of the line segment
        * @param cv_norm_type type of norm to use (cv::NORM_XYZ); defaults to square of L2
        */
        template<class Param_T>
        Param_T pointLineSegmentDistance(const cv::Vec<Param_T, 3> & p, const cv::Vec<Param_T, 3> & a, const cv::Vec<Param_T, 3> & b, int cv_norm_type = cv::NORM_L2SQR);

        /**
        * Compute the squared distance between a point and a plane
        * Where the plane is defined as: ax + by - z + c = 0
        * @param pt the point
        * @param eqn equation of plane in form: [a, b, c]
        * @return euclidean distance in meters
        */
        template<class T> T pointPlaneDistance(const cv::Vec<T, 3> & pt, const cv::Vec<T, 3> & eqn);

        /**
        * Compute the euclidean distance between a point and a plane
        * Where the plane is defined as: ax + by - z + c = 0
        * @param pt the point
        * @param a, b, c parameters of plane
        * @return euclidean distance in meters
        */
        template<class T> T pointPlaneDistance(const cv::Vec<T, 3> & pt, T a, T b, T c);

        /**
        * Compute the squared euclidean distance between a point and a plane
        * Where the plane is defined as: ax + by - z + c = 0
        * @param pt the point
        * @param eqn equation of plane in form: [a, b, c]
        * @return squared L2 norm in m^2
        */
        template<class T> T pointPlaneSquaredDistance(const cv::Vec<T, 3> & pt, const cv::Vec<T, 3> & eqn);

        /**
        * Compute the squared euclidean distance between a point and a plane
        * Where the plane is defined as: ax + by - z + c = 0
        * @param pt the point
        * @param a, b, c parameters of plane
        * @return squared L2 norm in m^2
        */
        template<class T> T pointPlaneSquaredDistance(const cv::Vec<T, 3> & pt, T a, T b, T c);

        /**
        * Estimate the euclidean distance per pixel around a point on a depth map
        * @param xyz_map the input depth map
        * @param pt the point
        * @param radius radius to average distance per pixel
        * @return distance per pixel
        */
        double euclideanDistancePerPixel(cv::Mat xyz_map, Point2i pt, int radius);

        /**
        * Removes points on img with the given indicies
        * @param [out] img the image to be operated on
        * @param points list of indicies (i,j) to be set to 0
        */
        void removePoints(cv::Mat & img, const std::vector<Point2i> & points);

		 /**
		  * 将“image”上对应于“ref_cloud”拟合“plane_equation”方程上的点的所有像素归零
		  * @param [in, out] xyz_map 输入点云
		  * @param [in] plane_equation 平面方程
		  * @param threshold 平面的厚度，即从平面到删除点的最大距离
		  * @param mask, mask_color optional mask matrix whose value must equal 'mask_color'
		  *                         at the correspoinding index
		  *                         for a point to be removed from the point cloud
		  */
        template <class T>
        void removePlane(const cv::Mat & ref_cloud, cv::Mat & image, const Vec3f & plane_equation,
                         double threshold, cv::Mat * mask = nullptr, uchar mask_color = 0);

		/**
		* 在一个点周围计算所有非零值的平均
		* @param img 要使用的基础图像
		* @param pt 兴趣点
		* @param radius 用于计算平均值的相邻点数
		* @return 兴趣点周围的平均值（x，y，z）
		*/
        Vec3f averageAroundPoint(const cv::Mat & img, const Point2i & pt, int radius = 5);

        /**
        * Find the approximate surface normal vector at a point on an XYZ map by computing
        * the cross product of two vectors to nearby points.
        * @param img base image to use
        * @param pt the point of interest
        * @param radius length of vectors to use for computing the cross product
        * @return normalized normal vector (the one facing viewer) at the point of interest
        */
        Vec3f normalAtPoint(const cv::Mat & img, const Point2i & pt, int radius = 3);

        /**
        * Eliminate outliers in a point cloud by considering the 'influence' of each point
        * @param data input points
        * @param output output points (should be empty)
        * @param thresh fraction of points to eliminate
        * @param data_aux optionally, auxilliary 2D input vector to reorder along with 'data'
        * @param output_aux optionally, auxilliary 2D output vector to reorder along with 'data'
        * @param num_points number of points in 'data' to use, defaults to all.
        * @return number of output points = floor('num_points' * (1.0 - 'thresh'))
        */
        int removeOutliers(const std::vector<Vec3f> & data, 
                               std::vector<Vec3f> & output, double thresh = 0.3, 
                               const std::vector<Point2i> * data_aux = nullptr,
                               std::vector<Point2i> * output_aux = nullptr,
                               int num_points = -1);

        /**
          * Perform RANSAC-based robust plane regression on a set of points
          * @param points [in] vector of 3D points (must contain at least 3 points)
          * @param thresh maximum squared L2 norm (r^2) from a point for consideration as an inlier
          * @param iterations number of RANSAC iterations
          * @param num_points number of input points to use (min 3). By default, uses all.
          * @return best-fitting plane through the given data points, in the form 
          *         [a,b,c]: 0 = ax + by - z + c
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
        * Determine whether (x,y) is a non-zero point in the matrix.
        * @param xyz_map Input image
        * @param x, y coordinates of the point
        * @return true if (x.y) is non-zero
        */
        bool isMember(cv::Mat xyz_map, int x, int y);

        /**
        * Find the average depth of a depth image
        * @param xyz_map depth image
        * @return average depth in meters
        */
        double averageDepth(cv::Mat xyz_map);

        /**
        * Find the centroid of the point cloud
        * @param xyz_map input point cloud
        * @return (x,y) coordinate of the centroid
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
        * Compute the angle in radians 'pointij' is at from the origin, going CCW starting from (0, 1), if y-axis is facing up.
        * @param pointij input point in ij coordinates
        * @return angle from origin, CCW from (0, 1)
        */
        double pointToAngle(const Point2f & pointij);

        /**
        * Compute the point with unit magnitude 'angle' radians CCW from (0, 1), if y-axis is facing up
        * @param angle input angle
        * @return point with unit magnitude 'angle' radians CCW from (0, 1)
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
         * Compute the magnitude of a point.
         * @param pt input point
         * @return magnitude of point
         */
        template <class T>
        double magnitude(cv::Point_<T> pt);

        /**
         * Compute the magnitude of a point.
         * @param pt input point
         * @return magnitude of point
         */
        template <class T>
        double magnitude(cv::Point3_<T> pt);

        /**
         * Compute the magnitude of a point.
         * @param pt input point
         * @return magnitude of point
         */
        template <class T, int n>
        double magnitude(cv::Vec<T, n> pt);

        /**
        * Compute the norm of a vector.
        * @param pt input point
        * @param cv_norm_type type of norm to use (cv::NORM_XYZ). default is square of L2.
        * @return norm of point
        */
		/**
		* 计算向量的范数
		* @param [in] pt input point
		* @param [in] cv_norm_type 要使用的范数类型（cv::NORM_XYZ）。默认值为L2的平方。
		* @return [double] 点的范数
		*/
        template <class Param_T>
        double norm(const cv::Point_<Param_T> & pt, int cv_norm_type = cv::NORM_L2SQR);

        /**
        * 计算向量的范数
        * @param pt input point
        * @param cv_norm_type type of norm to use (cv::NORM_XYZ). default is square of L2.
        * @return norm of point
        */
        template <class Param_T>
        double norm(const cv::Point3_<Param_T> & pt, int cv_norm_type = cv::NORM_L2SQR);

        /**
        * 计算向量的范数
        * @param pt input point
        * @param cv_norm_type type of norm to use (cv::NORM_XYZ). default is square of L2.
        * @return norm of point
        */
        template <class Param_T, int n>
        double norm(const cv::Vec<Param_T, n> & pt, int cv_norm_type = cv::NORM_L2SQR);

        /**
         * Compute the angle between two 3D vectors.
         * @param a, b the vectors
         * @param center optionally, vector to subtract both a and b by before computing angle
         * @return angle between vectors
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
         * Checks if a point is on the edge of a rectangle
         * @param rect the rectangle
         * @param pt the point
         * @param margin_tb max number of pixels from the top/bottom edges to be considered on edge
         * @param margin_lr max number of pixels from the left/right edges to be considered on edge
         * @param scale the scale of the point's coordinates relative to the scale of the image
         */
        bool pointOnEdge(const cv::Rect rect, const Point2i pt,
            int margin_tb = 30, int margin_lr = 30);

        /**
         * Checks if a point is on the edge of an image
         * @param [in] img the image
         * @param pt the point
         * @param margin_tb max number of pixels from the top/bottom edges to be considered on edge
         * @param margin_lr max number of pixels from the left/right edges to be considered on edge
         */
        bool pointOnEdge(const cv::Mat & img, const Point2i pt,
            int margin_tb = 30, int margin_lr = 30);

        /**
         * Computes the area of the triangle defined by three vertices
         * @param a first vertex
         * @param b second vertex
         * @param c third vertex
         * @return area of triangle, in real meters squared
         */
        float triangleArea(Vec3f a, Vec3f b, Vec3f c = Vec3f(0, 0, 0));

        /**
         * Computes the area of the quadrangle defined by four vertices
         * @param pts the vertices of the quadrangle
         * @return area of quadrangle, in real meters squared
         */
        float quadrangleArea(Vec3f pts[4]);

        /**
         * Computes the approximate surface area of all visible clusters on a depth map.
         * @param [in] depthMap the input depth map. All points with 0 z-coordinate will be excluded.
         * @return surface area, in meters squared
         */
        double surfaceArea(const cv::Mat & depthMap);

        /**
         * Computes the approximate surface area of a cluster containing the specified points
         * @param [in] frame_size size of the image frame
         * @param [in] points_ij ij coords of points in the cluster. Assumed to be ordered by y, then by x.
         * @param [in] points_xyz xyz coords of points in the cluster. Assumed to be in the same order as points_ij.
         * @param [in] sorted if true, assumes that 'cluster' is sorted and does not sort it again
         * @param [in] cluster_size number of points in this cluster. By default, uses all points in the 'cluster' vector.
         * @return surface area, in meters squared
         */
        double surfaceArea(const cv::Size & frame_size,
            const std::vector<Point2i> & points_ij,
            const std::vector<Vec3f> & points_xyz,
            int cluster_size = -1);

        /**
          * Approximates the surface area of a depth map cluster, using circles
          * of the smallest possible radius among adjacent points.
          * @param shape the depth map input
          * @return surface area, meters squared
          */
        double surfaceAreaCircle(cv::Mat shape);

        /**
          * Approximates the surface area of a depth map cluster by triangulation
          * among adjacent points
          * @param shape the depth map input
          * @return surface area, meters squared
          */
        double surfaceAreaTriangulate(cv::Mat shape);

        /**
        * compute the diameter of a set of 2D points. 
        * outputs the indices of the furthest points and returns the distance between them.
        * @param[in] points the input points
        * @param[out] a, b outputs the indices of the furthest points
        * @return 2D squred euclidean norm (distance^2) between the two points
        */
        double diameter(const std::vector<cv::Point> & points, int & a, int & b);

        /**
          * Sort points by y and then x coordinate using radix sort
          * @param points[in] vector of points to sort
          * @param width, height width and height of overall depth image
          * @param num_points number of points in'points' to use. By default, uses all.
          * @param points_xyz pointer to vector of associated xyz points. If provided, sorted along with ij points
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
         * Find the angle of curvature of a contour in radians near the specified point
         * @param[in] contour the input contour
         * @param index the index of the target point within the contour
         * @param start the number of points from the target point to begin averaging curvature
         * @param end the number of points from the target point to stop averaging curvature
         * @return angle in radians at the point
         */
        float contourLocalAngle(const std::vector<Point2i> & contour, int index,
            int start = 2, int end = 5);

        /** find the 2D distance, in pixels, between a given point and the farthest point in a given direction
          * that has a nonzero value on an xyz map.
          * @param xyz_map the XYZ map (point cloud)
          * @param center the center point
          * @param angle the direction, specified by an angle CCW from (0, 1) if y-axis is facing up
          * @param angle_offset the offset angle to be added to angle (default 0.0)
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