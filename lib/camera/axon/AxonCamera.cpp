//
// Created by lxz on 19-9-19.
//

// Header
#include "camera/axon/AxonCamera.h"

// Boost.log Libraries
#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>

namespace ht {
	
	namespace camera {
		//初始化相机
		void AXonCamera::initCamera() {
			// 定义状态变量
			openni::Status rc = openni::STATUS_OK;
			/** 初始化OpenNI */
			rc = openni::OpenNI::initialize();
			if (rc != openni::STATUS_OK) {
				BOOST_LOG_TRIVIAL(error) << "Initialize failed: " << openni::OpenNI::getExtendedError();
				exit(EXIT_FAILURE);
			}
			BOOST_LOG_TRIVIAL(info) << "Initialize OpenNI success.";

			/** 使用OpenNI尝试打开设备 */
			rc = this->device.open(openni::ANY_DEVICE);
			if (rc != openni::STATUS_OK)
			{
				BOOST_LOG_TRIVIAL(error) << "Device open failed!!\b" << openni::OpenNI::getExtendedError();
				openni::OpenNI::shutdown();
				exit(EXIT_FAILURE);
			}

			/** 查看驱动版本号 */
			OniVersion driver;
			int nsize = sizeof(driver);
			this->device.getProperty(ONI_DEVICE_PROPERTY_DRIVER_VERSION, &driver, &nsize);	// 获取驱动版本号
			BOOST_LOG_TRIVIAL(info) << "AXon driver version: " << driver.major << "."
				<< driver.minor << "." << driver.maintenance << "." << driver.build;

			///** 设置RGB和Depth对齐  */
			//openni::ImageRegistrationMode mode = this->device.getImageRegistrationMode();
			//if (mode != openni::IMAGE_REGISTRATION_COLOR_TO_DEPTH) {
			//	this->device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_COLOR_TO_DEPTH); //对齐方式color to depth
			//}
			//AXonLinkCamParam camParam;
			//int dataSize = sizeof(AXonLinkCamParam);
			//rc = this->device.getProperty(AXONLINK_DEVICE_PROPERTY_GET_CAMERA_PARAMETERS,
			//	&camParam, &dataSize);

			/** 创建深度流 */
			if (this->device.getSensorInfo(openni::SENSOR_DEPTH) != nullptr) {
				// 有深度Sensor则创建深度流
				rc = this->depth.create(device, openni::SENSOR_DEPTH);
				if (rc == openni::STATUS_OK)
				{
					/** 开启深度流 */
					rc = this->depth.start();
					if (rc != openni::STATUS_OK)
					{
						BOOST_LOG_TRIVIAL(error) << "Couldn't start depth stream: " << openni::OpenNI::getExtendedError();
						this->depth.destroy();
						exit(EXIT_FAILURE);
					}
				}
				else
				{
					BOOST_LOG_TRIVIAL(error) << "Couldn't find depth stream: " << openni::OpenNI::getExtendedError();
					exit(EXIT_FAILURE);
				}
			}

			/** 检查深度流是否合法 */
			if (!this->depth.isValid())
			{
				BOOST_LOG_TRIVIAL(error) << "No valid streams. Exiting\n";
				openni::OpenNI::shutdown();
				exit(EXIT_FAILURE);
			}

			/** 开启深度图像镜像 */
			this->depth.setMirroringEnabled(true);


		}

		// 等待流同步
		bool AXonCamera::waitAllStream(openni::VideoStream** streams, int allCount, int timeout)
		{
			int streamCount = allCount;
			while (1)
			{
				if (streamCount == 0)
					return true;
				int readyStreamIndex = 0;
				if (openni::STATUS_OK != openni::OpenNI::waitForAnyStream(streams, streamCount, &readyStreamIndex, timeout))
				{
					printf("Error: Device open failed:\n%s\n", openni::OpenNI::getExtendedError());
					return false;
				}

				streams[readyStreamIndex] = NULL;

				streamCount--;
			}
		}
		/**
		 * 构造函数
		 */
		AXonCamera::AXonCamera() :isCapture(true)
		{
			BOOST_LOG_TRIVIAL(info) << "Begin init " << this->getModelName() << " Camera...";
			//初始化相机
			this->initCamera();
			// 初始化图像矩阵
			this->xyzMap.create(cv::Size(AXonCamera::Depth_Width, AXonCamera::Depth_Width), CV_32FC3);
			this->xyzBuffer.create(cv::Size(AXonCamera::Depth_Width, AXonCamera::Depth_Width), CV_32FC3);
			// 初始化图像捕获线程
			this->capThread = std::make_shared<std::thread>(&AXonCamera::updateHelper, this);
		}

		AXonCamera::~AXonCamera()
		{
			// 关闭相机
			BOOST_LOG_TRIVIAL(info) << "Close " << this->getModelName() << " Camera...";
			/** 关闭图像捕获线程 */
			this->isCapture = false;
			this->capThread->join();
			/** 停止所有流 */
			this->depth.stop();
			/** 销毁所有流 */
			this->depth.destroy();
			/** 关闭设备 */
			this->device.close();
			/** 关闭OpenNI */
			openni::OpenNI::shutdown();
		}

		/**
		 * 更新深度信息辅助函数
		 */
		void AXonCamera::updateHelper() {
			BOOST_LOG_TRIVIAL(info) << "检测设备连接状态...";
			if (!this->device.isValid())
				return;
			BOOST_LOG_TRIVIAL(info) << "设备连接状态: OK";

			// 记录帧数据
			openni::Status rc = openni::STATUS_OK;
			openni::VideoFrameRef depthFrame;

			int temp;
			BOOST_LOG_TRIVIAL(info) << "图像捕获线程启动.";
			while (this->isCapture)
			{
				openni::VideoStream* pStream = &this->depth;
				rc = openni::OpenNI::waitForAnyStream(&pStream, 1, &temp, 2000);    /*等待流*/
				if (rc != openni::STATUS_OK)
				{
					BOOST_LOG_TRIVIAL(error) << "Wait failed! (timeout is 2000 ms): "
						<< openni::OpenNI::getExtendedError();
					return;
				}

				/*读深度流帧数据*/
				rc = this->depth.readFrame(&depthFrame);
				if (rc != openni::STATUS_OK)
				{
					BOOST_LOG_TRIVIAL(error) << "Read failed: " << openni::OpenNI::getExtendedError();
					return;
				}

				/** 验证深度流数据格式是否合法 */
				if (depthFrame.getVideoMode().getPixelFormat() != openni::PIXEL_FORMAT_DEPTH_1_MM &&
					depthFrame.getVideoMode().getPixelFormat() != openni::PIXEL_FORMAT_DEPTH_100_UM &&
					depthFrame.getVideoMode().getPixelFormat() != openni::PIXEL_FORMAT_DEPTH_1_3_MM)
				{
					BOOST_LOG_TRIVIAL(error) << "Unexpected frame format!!";
					return;
				}

				/** 当前帧的深度数据 */
				const openni::DepthPixel* depthImage = (const openni::DepthPixel*)depthFrame.getData();

				// 将图像矩阵置零
				this->xyzBuffer = cv::Scalar::all(0);
				cv::Vec3f* xyzptr = nullptr;


				for (int i = 0; i < depthFrame.getWidth(); i++)
				{
					//xyzptr = this->xyzBuffer.ptr<cv::Vec3f>(i);
					for (int j = 0; j < depthFrame.getHeight(); j++)
					{
						float pz = 0, px = 0, py = 0;
						openni::CoordinateConverter::convertDepthToWorld(this->depth, j, i, depthImage[j * depthFrame.getWidth() + i], &px, &py, &pz);
						xyzBuffer.ptr<cv::Vec3f>(j)[i][0] = px / 2000.0;
						xyzBuffer.ptr<cv::Vec3f>(j)[i][1] = py / 2000.0;
						xyzBuffer.ptr<cv::Vec3f>(j)[i][2] = pz / 2000.0;
						if (xyzBuffer.ptr<cv::Vec3f>(j)[i][2] < NOISE_FILTER_LOW || xyzBuffer.ptr<cv::Vec3f>(j)[i][2] > NOISE_FILTER_HIGH) {
							xyzBuffer.ptr<cv::Vec3f>(j)[i][2] = 0;
						}
						//printf("(%f,%f,%f)\n", xyzptr[j][2], xyzptr[j][2], xyzptr[j][2]);
						//printf("with:%f, heigth: %f\n", depthFrame.getWidth(), depthFrame.getHeight());
					}
				}
			
				// 将后台缓冲交换到前台
				std::lock_guard<std::mutex> lock(this->imageMutex);
				cv::swap(this->xyzMap, this->xyzBuffer);

			}
			BOOST_LOG_TRIVIAL(info) << "图像捕获线程关闭...";

		}

		CamIntrinsicParam* AXonCamera::GetDepthInstrinsicParamByResolution(int  nWidth, int nHeight, AXonLinkCamParam* allParam)
		{
			// Find Color Sensor Intrinsic Parameters
			CamIntrinsicParam* pstParam = NULL;
			for (int i = 0; i < AXON_LINK_SUPPORTED_PARAMETERS; i++)
			{
				pstParam = &allParam->astDepthParam[i];
				if (pstParam->ResolutionX == nWidth
					&& pstParam->ResolutionY == nHeight)
					return pstParam;
			}

			return NULL;
		}


		/********************* 重写的基类方法 *****************/
		const std::string AXonCamera::getModelName() const {
			return "AXon";
		}

		int AXonCamera::getWidth() const {
			return AXonCamera::Depth_Width;
		}

		int AXonCamera::getHeight() const {
			return AXonCamera::Depth_Height;
		}

		float AXonCamera::flagMapConfidenceThreshold() const {
			return (60.0f / 255.0f * 500.0f);
		}

		int AXonCamera::ampMapInvalidFlagValue() const {
			return  0u;
		}

		bool AXonCamera::hasAmpMap() const
		{
			return false;
		}

		bool AXonCamera::hasFlagMap() const
		{
			return false;
		}

		/** !!此方法一定要重写 **/
		void AXonCamera::update(cv::Mat& xyz_map, cv::Mat& rgb_map, cv::Mat& ir_map,
			cv::Mat& amp_map, cv::Mat& flag_map)
		{
			// 一定要执行拷贝操作，不能直接使用等号运算符赋值

			std::lock_guard<std::mutex> lock(this->imageMutex);
			this->xyzMap.copyTo(xyz_map);
		}
	}
}
