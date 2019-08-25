#pragma once
#ifndef PMDCAMERA_PMDCAMREALISTENER_H
#define PMDCAMERA_PMDCAMREALISTENER_H

#include "DataBuffer.hpp"

#include <opencv2/opencv.hpp>
#include <royale.hpp>
#include <memory>
#include <mutex>

namespace ht {
	/** 原始数据宽度 */
	constexpr int DATA_WIDTH = 224;
	/** 原始数据高度 */
	constexpr int DATA_HEIGHT = 171;


	class CamListener : public royale::IDepthDataListener
	{

	public:

		CamListener();
		virtual ~CamListener() = default;
		void onNewData(const royale::DepthData* data) override;

		//设置相机参数
		void setLensParameters(royale::LensParameters lensParameters);

		void getXYZMap(cv::Mat& xyzMap) {
			std::lock_guard<std::mutex> lock(this->imgMutex);
			this->xyz_map.copyTo(xyzMap);
			//return this->xyz_map;
		}
		void getAmpMap(cv::Mat& ampMap) {
			std::lock_guard<std::mutex> lock(this->imgMutex);
			this->confidence_map.copyTo(ampMap);
			// this->confidence_map;
		}
	private:
		// 图像矩阵
		cv::Mat zImage;
		cv::Mat grayImage;
		cv::Mat xyz_map;
		cv::Mat confidence_map;

		// 相机内参矩阵
		cv::Mat cameraMatrix;
		cv::Mat distortionCoefficients;

		// 去噪参数
		const float NOISE_FILTER_LOW = 0.10f;
		const float NOISE_FILTER_HIGH = 0.80f;

		// 线程锁
		std::mutex imgMutex;
	};
}



#endif //PMDCAMERA_PMDCAMREALISTENER_H

