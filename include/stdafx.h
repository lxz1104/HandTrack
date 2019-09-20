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
//#include <opencv2/ml.hpp>

// Boost log Libraries
#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
//#include <boost/log/expressions.hpp>

// OpenMP Libraries
#ifdef _OPENMP
#include <omp.h>
#endif

//#define DEBUG

#endif // !STDAFX_H