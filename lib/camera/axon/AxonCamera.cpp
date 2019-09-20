//
// Created by lxz on 19-9-19.
//
#include "camera/axon/AxonCamera.h"

// Boost.log Libraries
#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>

namespace ht {

	//初始化相机
	void AXonCamera::initCamera() {
		// 状态变量
		openni::Status rc = openni::STATUS_OK;
		/** 初始化OpenNI */
		rc = openni::OpenNI::initialize();
		if (rc != openni::STATUS_OK) {
			BOOST_LOG_TRIVIAL(error) << "Initialize failed: " << openni::OpenNI::getExtendedError();
			exit(EXIT_FAILURE);
		}
		BOOST_LOG_TRIVIAL(info) << "Initialize OpenNI success.";
	    
		/** 使用OpenNI尝试打开设备 */
		rc = device.open(openni::ANY_DEVICE);
		if (rc != openni::STATUS_OK)
		{
			BOOST_LOG_TRIVIAL(error) << "Device open failed: " << openni::OpenNI::getExtendedError();
			openni::OpenNI::shutdown();
			exit(EXIT_FAILURE);
		}

		/** 查看驱动版本号 */
		OniVersion driver;
		int nsize = sizeof(driver);
		this->device.getProperty(ONI_DEVICE_PROPERTY_DRIVER_VERSION,&driver,&nsize);	// 获取驱动版本号
		BOOST_LOG_TRIVIAL(info) << "AXon driver version: " << driver.major << "." 
								<< driver.minor << "." << driver.maintenance << "." << driver.build;

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
					depth.destroy();
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
		if (!depth.isValid())
		{
			BOOST_LOG_TRIVIAL(error) << "No valid streams. Exiting\n";
			openni::OpenNI::shutdown();
			exit(EXIT_FAILURE);
		}
	}
	/**
	 * 构造函数
	 */
	AXonCamera::AXonCamera()
	{
		BOOST_LOG_TRIVIAL(info) << "Begin init " << this->getModelName() << " Camera...";
		//初始化相机
		this->initCamera();
		// 初始化图像矩阵
		this->xyzMap.create(cv::Size(AXonCamera::Depth_Width, AXonCamera::Depth_Width), CV_32FC3);
	}

	AXonCamera::~AXonCamera()
	{
		// 关闭相机
		BOOST_LOG_TRIVIAL(info) << "Close " << this->getModelName() << " Camera...";
		/** 停止深度流 */
		this->depth.stop();  
		/** 销毁深度流 */
		this->depth.destroy();
		/** 关闭设备 */
		this->device.close(); 
		/** 关闭OpenNI */
		openni::OpenNI::shutdown();
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
		
		// 记录帧数据
		openni::VideoFrameRef frame;
		// 状态变量
		openni::Status rc = openni::STATUS_OK;

		int changedStreamDummy;
		openni::VideoStream* pStream = &depth;
		rc = openni::OpenNI::waitForAnyStream(&pStream, 1, &changedStreamDummy,
			2000);    /*等待流*/
		if (rc != openni::STATUS_OK)
		{
			BOOST_LOG_TRIVIAL(error) << "Wait failed! (timeout is 2000 ms): "
									 << openni::OpenNI::getExtendedError();
			return;
		}

		/*读深度流帧数据*/
		rc = depth.readFrame(&frame); 
		if (rc != openni::STATUS_OK)
		{
			BOOST_LOG_TRIVIAL(error) << "Read failed: " <<  openni::OpenNI::getExtendedError();
			return;
		}

		if (frame.getVideoMode().getPixelFormat() != openni::PIXEL_FORMAT_DEPTH_1_MM &&
			frame.getVideoMode().getPixelFormat() != openni::PIXEL_FORMAT_DEPTH_100_UM &&
			frame.getVideoMode().getPixelFormat() != openni::PIXEL_FORMAT_DEPTH_1_3_MM)
		{
			BOOST_LOG_TRIVIAL(error) << "Unexpected frame format!!";
			return;
		}

		openni::DepthPixel* pDepthRow = (openni::DepthPixel*)frame.getData(); /*当前帧的深度数据*/

		// 将图像矩阵置零
		this->xyzMap = cv::Scalar::all(0);
		cv::Vec3f* xyzptr = nullptr;

		for (int y = 0; y < frame.getHeight(); ++y)
		{
			xyzptr = this->xyzMap.ptr<cv::Vec3f>(y);

			const openni::DepthPixel* pDepth = pDepthRow;

			for (int x = 0; x < frame.getWidth(); ++x, ++pDepth)
			{
				xyzptr[x][0] = x;
				xyzptr[x][1] = y;
				xyzptr[x][2] = *pDepth;
			}
		}


		this->xyzMap.copyTo(xyz_map);
		//int middleIndex = (frame.getHeight() + 1) * frame.getWidth() / 2;
		///*打印当前帧的时间戳和中心深渡值*/
		//printf("[%08llu]%8d\n", (long long)frame.getTimestamp(), pDepth[middleIndex]);


	}
}
