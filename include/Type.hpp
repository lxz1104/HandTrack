#pragma once
#ifndef TYPE_HPP
#define TYPE_HPP

#include <opencv2/opencv.hpp>

// 预定义
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#ifndef PI
#define PI 3.14159265358979323846
#endif

namespace ht {

	// 基础类型
	typedef cv::Point Point;
	typedef cv::Point2i Point2i;
	typedef cv::Point2f Point2f;
	typedef cv::Point2d Point2d;
	typedef cv::Vec2f Vec2f;
	typedef cv::Vec2d Vec2d;
	typedef cv::Vec2i Vec2i;
	typedef cv::Vec3b Vec3b;
	typedef cv::Vec3f Vec3f;
	typedef cv::Vec3d Vec3d;
	typedef cv::Vec3i Vec3i;

	// 代码中常用的智能指针
	typedef std::shared_ptr<std::vector<Point2i> > VecP2iPtr;
	typedef std::shared_ptr<std::vector<Vec3f> > VecV3fPtr;
}
#endif // !TYPE_HPP