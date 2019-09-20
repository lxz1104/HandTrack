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
		if (this->device.getSensorInfo(openni::SENSOR_DEPTH) != nullptr) {\
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
		

		if (!depth.isValid())// || !color.isValid())
		{
			printf("SimpleViewer: No valid streams. Exiting\n");
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
		return AXonCamera::depth_width;
	}

	int AXonCamera::getHeight() const {
		return AXonCamera::depth_height;
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
	}
}
