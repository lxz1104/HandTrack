//
// Created by lxz on 19-9-19.
//
#include "camera/axon/AxonCamera.h"

// Boost.log Libraries
#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>

namespace ht {

	//��ʼ�����
	void AXonCamera::initCamera() {
		// ״̬����
		openni::Status rc = openni::STATUS_OK;
		/** ��ʼ��OpenNI */
		rc = openni::OpenNI::initialize();
		if (rc != openni::STATUS_OK) {
			BOOST_LOG_TRIVIAL(error) << "Initialize failed: " << openni::OpenNI::getExtendedError();
			exit(EXIT_FAILURE);
		}
		BOOST_LOG_TRIVIAL(info) << "Initialize OpenNI success.";
	    
		/** ʹ��OpenNI���Դ��豸 */
		rc = device.open(openni::ANY_DEVICE);
		if (rc != openni::STATUS_OK)
		{
			BOOST_LOG_TRIVIAL(error) << "Device open failed: " << openni::OpenNI::getExtendedError();
			openni::OpenNI::shutdown();
			exit(EXIT_FAILURE);
		}

		/** �鿴�����汾�� */
		OniVersion driver;
		int nsize = sizeof(driver);
		this->device.getProperty(ONI_DEVICE_PROPERTY_DRIVER_VERSION,&driver,&nsize);	// ��ȡ�����汾��
		BOOST_LOG_TRIVIAL(info) << "AXon driver version: " << driver.major << "." 
								<< driver.minor << "." << driver.maintenance << "." << driver.build;

		/** ��������� */
		if (this->device.getSensorInfo(openni::SENSOR_DEPTH) != nullptr) {\
			// �����Sensor�򴴽������
			rc = this->depth.create(device, openni::SENSOR_DEPTH);
			if (rc == openni::STATUS_OK)
			{
				/** ��������� */
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
	 * ���캯��
	 */
	AXonCamera::AXonCamera()
	{
		BOOST_LOG_TRIVIAL(info) << "Begin init " << this->getModelName() << " Camera...";
		//��ʼ�����
		this->initCamera();
	}

	AXonCamera::~AXonCamera()
	{
		// �ر����
		BOOST_LOG_TRIVIAL(info) << "Close " << this->getModelName() << " Camera...";
		/** ֹͣ����� */
		this->depth.stop();  
		/** ��������� */
		this->depth.destroy();
		/** �ر��豸 */
		this->device.close(); 
		/** �ر�OpenNI */
		openni::OpenNI::shutdown();
	}


	/********************* ��д�Ļ��෽�� *****************/
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

	/** !!�˷���һ��Ҫ��д **/
	void AXonCamera::update(cv::Mat& xyz_map, cv::Mat& rgb_map, cv::Mat& ir_map,
			cv::Mat& amp_map, cv::Mat& flag_map)
	{
		// һ��Ҫִ�п�������������ֱ��ʹ�õȺ��������ֵ
	}
}
