#include "camera/pmd/PMDCamreaListener.h"

namespace ht {

	void CamListener::onNewData(const royale::DepthData* data) {
		std::lock_guard<std::mutex> lock(this->imgMutex);

		// 将图像矩阵置零
		this->zImage = cv::Scalar::all(0);
		this->grayImage = cv::Scalar::all(0);
		this->xyz_map = cv::Scalar::all(0);

		cv::Vec3f* xyzptr = nullptr;
		float* grayptr = nullptr;
		uint8_t* confptr = nullptr;
		size_t k = 0;
		for (int y = 0; y < this->xyz_map.rows; y++) {
			xyzptr = this->xyz_map.ptr<cv::Vec3f>(y);
			grayptr = this->grayImage.ptr<float>(y);
			confptr = this->confidence_map.ptr<uint8_t>(y);
			for (int x = 0; x < this->xyz_map.cols; x++, k++) {
				auto curPoint = data->points.at(k);
				confptr[x] = curPoint.depthConfidence;
				if (curPoint.z == 0.0f) {
					continue; //错误的深度信息
				}
				if (curPoint.z < NOISE_FILTER_LOW || curPoint.z > NOISE_FILTER_HIGH || curPoint.depthConfidence < 180) {
					continue;
				}
				xyzptr[x][0] = curPoint.x;
				xyzptr[x][1] = curPoint.y;
				xyzptr[x][2] = curPoint.z;
				grayptr[x] = curPoint.grayValue;
			}
		}
		//this->xyzBuffer.Put(this->pXYZImage.release());
		/*this->xyzBuffer->Put(this->xyz_map.clone());
		this->ampBuffer->Put(this->confidence_map.clone());*/
		/*cv::imshow("Gray", this->grayImage);
		cv::waitKey(1);*/
	}

	CamListener::CamListener() {
		// 创建两个图像,之后将填充每个包含一个 32Bit 通道的图像
		this->zImage.create(cv::Size(DATA_WIDTH, DATA_HEIGHT), CV_32FC1);
		this->grayImage.create(cv::Size(DATA_WIDTH, DATA_HEIGHT), CV_32FC1);
		this->xyz_map.create(cv::Size(DATA_WIDTH, DATA_HEIGHT), CV_32FC3);
		this->confidence_map.create(cv::Size(DATA_WIDTH, DATA_HEIGHT), CV_8UC1);
	}

	void CamListener::setLensParameters(royale::LensParameters lensParameters)
	{
		// 构造相机矩阵
		// (fx   0    cx)
		// (0    fy   cy)
		// (0    0    1 )
		this->cameraMatrix = (cv::Mat1d(3, 3) << lensParameters.focalLength.first, 0, lensParameters.principalPoint.first,
			0, lensParameters.focalLength.second, lensParameters.principalPoint.second,
			0, 0, 1);

		// 构造失真系数
		// k1 k2 p1 p2 k3
		this->distortionCoefficients = (cv::Mat1d(1, 5) << lensParameters.distortionRadial[0],
			lensParameters.distortionRadial[1],
			lensParameters.distortionTangential.first,
			lensParameters.distortionTangential.second,
			lensParameters.distortionRadial[2]);
	}
}

