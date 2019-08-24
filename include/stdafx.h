#pragma once
#ifndef STDAFX_H
#define STDAFX_H



// C++ Libraries
#include <cctype>
#include <cfloat>
#include <climits>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <utility>
#include <thread>
#include <mutex>

// Boost
#include <boost/filesystem.hpp>
#include <boost/function.hpp>
#include <boost/polygon/voronoi.hpp>
#include <boost/polygon/point_data.hpp>
#include <boost/polygon/segment_data.hpp>

// OpenCV Libraries
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/ml.hpp>

// Boost log Libraries
#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
//#include <boost/log/expressions.hpp>

// OpenMP Libraries
#ifdef _OPENMP
#include <omp.h>
#endif

// 预定义
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#ifndef PI
#define PI 3.14159265358979323846
#endif

//#define DEBUG

namespace ht {

	// Typedefs for common types
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

#endif // !STDAFX_H