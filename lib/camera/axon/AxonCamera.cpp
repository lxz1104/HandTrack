//
// Created by lxz on 19-9-19.
//

// Header
#include "camera/axon/AxonCamera.h"

// Boost.log Libraries
#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>

namespace ht {

	//��ʼ�����
	void AXonCamera::initCamera() {
		// ����״̬����
		openni::Status rc = openni::STATUS_OK;
		/** ��ʼ��OpenNI */
		rc = openni::OpenNI::initialize();
		if (rc != openni::STATUS_OK) {
			BOOST_LOG_TRIVIAL(error) << "Initialize failed: " << openni::OpenNI::getExtendedError();
			exit(EXIT_FAILURE);
		}
		BOOST_LOG_TRIVIAL(info) << "Initialize OpenNI success.";
	    
		/** ʹ��OpenNI���Դ��豸 */
		rc = this->device.open(openni::ANY_DEVICE);
		if (rc != openni::STATUS_OK)
		{
			BOOST_LOG_TRIVIAL(error) << "Device open failed!!\b" << openni::OpenNI::getExtendedError();
			openni::OpenNI::shutdown();
			exit(EXIT_FAILURE);
		}

		/** �鿴�����汾�� */
		OniVersion driver;
		int nsize = sizeof(driver);
		this->device.getProperty(ONI_DEVICE_PROPERTY_DRIVER_VERSION,&driver,&nsize);	// ��ȡ�����汾��
		BOOST_LOG_TRIVIAL(info) << "AXon driver version: " << driver.major << "." 
								<< driver.minor << "." << driver.maintenance << "." << driver.build;

		/** ����RGB��Depth����  */
		openni::ImageRegistrationMode mode = this->device.getImageRegistrationMode();
		if (mode != openni::IMAGE_REGISTRATION_COLOR_TO_DEPTH) {
			this->device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_COLOR_TO_DEPTH); //���뷽ʽcolor to depth
		}
		AXonLinkCamParam camParam;
		int dataSize = sizeof(AXonLinkCamParam);
		rc = this->device.getProperty(AXONLINK_DEVICE_PROPERTY_GET_CAMERA_PARAMETERS,
			&camParam, &dataSize);

		/** ��������� */
		if (this->device.getSensorInfo(openni::SENSOR_DEPTH) != nullptr) {
			// �����Sensor�򴴽������
			rc = this->depth.create(device, openni::SENSOR_DEPTH);
			if (rc == openni::STATUS_OK)
			{
				/** ��������� */
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

		/** ����RGB�� */
		if (this->device.getSensorInfo(openni::SENSOR_COLOR) != nullptr)
		{
			// ��RGB Sensor�򴴽�RGB��
			rc = this->color.create(device, openni::SENSOR_COLOR);
			if (rc == openni::STATUS_OK)
			{
				/** ����RGB�� */
				openni::VideoMode mode = this->color.getVideoMode();
				mode.setPixelFormat(openni::PIXEL_FORMAT_RGB888);
				this->color.setVideoMode(mode);
				rc = this->color.start();
				if (rc != openni::STATUS_OK)
				{
					BOOST_LOG_TRIVIAL(error) << "Couldn't start RGB stream: " << openni::OpenNI::getExtendedError();
					this->color.destroy();
					exit(EXIT_FAILURE);
				}
			}
			else
			{
				BOOST_LOG_TRIVIAL(error) << "Couldn't find RGB stream: " << openni::OpenNI::getExtendedError();
				exit(EXIT_FAILURE);
			}
		}

		/** ����IR�� */
		if (this->device.getSensorInfo(openni::SENSOR_IR) != nullptr)
		{
			// ��IR Sensor�򴴽�IR��
			rc = this->ir.create(device, openni::SENSOR_IR);
			if (rc == openni::STATUS_OK)
			{
				/** ����IR�� */
				rc = this->ir.start();
				if (rc != openni::STATUS_OK)
				{
					BOOST_LOG_TRIVIAL(error) << "Couldn't start IR stream: " << openni::OpenNI::getExtendedError();
					this->ir.destroy();
					exit(EXIT_FAILURE);
				}
			}
			else
			{
				BOOST_LOG_TRIVIAL(error) << "Couldn't find IR stream: " << openni::OpenNI::getExtendedError();
				exit(EXIT_FAILURE);
			}
		}
		
		/** ���������Ƿ�Ϸ� */
		if (!(this->depth.isValid()) || (!this->color.isValid()) || (!this->ir.isValid()))
		{
			BOOST_LOG_TRIVIAL(error) << "No valid streams. Exiting\n";
			openni::OpenNI::shutdown();
			exit(EXIT_FAILURE);
		}
		
		/** �������ͼ���� */
		//this->depth.setMirroringEnabled(true);
		/** ������ͬ�� */
		this->device.setDepthColorSyncEnabled(true); 
		/** ���ö��뷽ʽ */
		this->device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_COLOR_TO_DEPTH);
	
		/** �����л�ȡ֡���� */
		openni::VideoStream* allStream[3] = { NULL };
		allStream[0] = &this->depth;
		allStream[1] = &this->color;
		allStream[2] = &this->ir;

		while (!this->waitAllStream(allStream, 1, 200)) //�ȴ����������õ�֡
		{
			;
		}
		openni::VideoFrameRef colorFrame, depthFrame, irFrame;
		this->color.readFrame(&colorFrame);//ȡ����������֡��ע�⣺����ͬ��֮�󣬱�����������֡��������ߣ�����ͬ�����ϣ��ᵼ����Զ�ڵȴ�
		this->depth.readFrame(&depthFrame);
		this->ir.readFrame(&irFrame);
	}

	// �ȴ���ͬ��
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
	 * ���캯��
	 */
	AXonCamera::AXonCamera():isCapture(true)
	{
		BOOST_LOG_TRIVIAL(info) << "Begin init " << this->getModelName() << " Camera...";
		//��ʼ�����
		this->initCamera();
		// ��ʼ��ͼ�����
		this->xyzMap.create(cv::Size(AXonCamera::Depth_Width, AXonCamera::Depth_Width), CV_32FC3);
		this->xyzBuffer.create(cv::Size(AXonCamera::Depth_Width, AXonCamera::Depth_Width), CV_32FC3);
		// ��ʼ��ͼ�񲶻��߳�
		this->capThread = std::make_shared<std::thread>(&AXonCamera::updateHelper, this);
	}

	AXonCamera::~AXonCamera()
	{
		// �ر����
		BOOST_LOG_TRIVIAL(info) << "Close " << this->getModelName() << " Camera...";
		/** �ر�ͼ�񲶻��߳� */
		this->isCapture = false;
		this->capThread->join();
		/** ֹͣ������ */
		this->depth.stop();  
		this->color.stop();
		this->ir.stop();
		/** ���������� */
		this->depth.destroy();
		this->color.destroy();
		this->ir.destroy();
		/** �ر��豸 */
		this->device.close(); 
		/** �ر�OpenNI */
		openni::OpenNI::shutdown();
	}

	/**
	 * ���������Ϣ��������
	 */
	void AXonCamera::updateHelper() {
		BOOST_LOG_TRIVIAL(info) << "����豸����״̬...";
		if (!this->device.isValid())
			return;
		if (!device.getDepthColorSyncEnabled())
			device.setDepthColorSyncEnabled(true);
		BOOST_LOG_TRIVIAL(info) << "�豸����״̬: OK";

		openni::VideoMode colorVideoMode = this->color.getVideoMode();
		colorVideoMode.setResolution(640, 480);  //������������ͬ�ֱ���
		this->color.setVideoMode(colorVideoMode);
		openni::VideoMode depthVideoMode = this->depth.getVideoMode();
		depthVideoMode.setResolution(640, 480);
		this->depth.setVideoMode(depthVideoMode);
		openni::VideoMode irVideoMode = this->ir.getVideoMode();
		irVideoMode.setResolution(640, 480);
		this->ir.setVideoMode(irVideoMode);

		openni::ImageRegistrationMode mode = this->device.getImageRegistrationMode();
		if (mode != openni::IMAGE_REGISTRATION_COLOR_TO_DEPTH)
			this->device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_COLOR_TO_DEPTH); //���뷽ʽcolor to depth


		AXonLinkCamParam camParam;
		int dataSize = sizeof(AXonLinkCamParam);
		openni::Status rc = openni::STATUS_OK;
		rc = this->device.getProperty(AXONLINK_DEVICE_PROPERTY_GET_CAMERA_PARAMETERS,
			&camParam, &dataSize);
		CamIntrinsicParam* depthIntParam = GetDepthInstrinsicParamByResolution(640, 480, &camParam);// ���֡����


		// ��¼֡����
		openni::VideoFrameRef colorFrame, depthFrame, irFrame;
		
		int temp;
		BOOST_LOG_TRIVIAL(info) << "ͼ�񲶻��߳�����.";
		while (this->isCapture)
		{
			openni::VideoStream* pStream = &this->depth;
			rc = openni::OpenNI::waitForAnyStream(&pStream, 1, &temp,2000);    /*�ȴ���*/
			if (rc != openni::STATUS_OK)
			{
				BOOST_LOG_TRIVIAL(error) << "Wait failed! (timeout is 2000 ms): "
					<< openni::OpenNI::getExtendedError();
				return;
			}

			/*�������֡����*/
			rc = this->depth.readFrame(&depthFrame);
			rc = this->color.readFrame(&colorFrame);
			rc = this->ir.readFrame(&irFrame);
			if (rc != openni::STATUS_OK)
			{
				BOOST_LOG_TRIVIAL(error) << "Read failed: " << openni::OpenNI::getExtendedError();
				return;
			}

			/** ��֤��������ݸ�ʽ�Ƿ�Ϸ� */
			if (depthFrame.getVideoMode().getPixelFormat() != openni::PIXEL_FORMAT_DEPTH_1_MM &&
				depthFrame.getVideoMode().getPixelFormat() != openni::PIXEL_FORMAT_DEPTH_100_UM &&
				depthFrame.getVideoMode().getPixelFormat() != openni::PIXEL_FORMAT_DEPTH_1_3_MM)
			{
				BOOST_LOG_TRIVIAL(error) << "Unexpected frame format!!";
				return;
			}

			openni::PixelFormat format = this->depth.getVideoMode().getPixelFormat();
			float depth_unit = openni::OpenNI::getDepthValueUnit_mm(format);

			/** ��ǰ֡��������� */
			const openni::DepthPixel* depthImage = (const openni::DepthPixel*)depthFrame.getData();

			// ��ͼ���������
			this->xyzBuffer = cv::Scalar::all(0);
			cv::Vec3f* xyzptr = nullptr;
			for (int y = 0; y < depthFrame.getHeight(); ++y)
			{
				xyzptr = this->xyzBuffer.ptr<cv::Vec3f>(y);
				const openni::DepthPixel* depthRowHead = depthImage;
				for (int x = 0; x < depthFrame.getWidth(); ++x)
				{
					int depthValue = (int)(*(depthRowHead + x)) * depth_unit;
					if (depthValue > 0)
					{

						float pz = depthValue * depth_unit;
						float px = (y - depthIntParam->cx) * pz / depthIntParam->fx;
						float py = (depthIntParam->cy - x) * pz / depthIntParam->fy;

						xyzptr[x][0] = px;
						xyzptr[x][1] = py;
						xyzptr[x][2] = -pz;
						printf("(%f,%f,%f)\n", xyzptr[x][2], xyzptr[x][2], xyzptr[x][2]);
					}
				}
			}

			{
				// ����̨���彻����ǰ̨
				std::lock_guard<std::mutex> lock(imageMutex);
				cv::swap(this->xyzMap, this->xyzBuffer);
			}
			
		}
		BOOST_LOG_TRIVIAL(info) << "ͼ�񲶻��̹߳ر�...";
		
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


	/********************* ��д�Ļ��෽�� *****************/
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

	/** !!�˷���һ��Ҫ��д **/
	void AXonCamera::update(cv::Mat& xyz_map, cv::Mat& rgb_map, cv::Mat& ir_map,
			cv::Mat& amp_map, cv::Mat& flag_map)
	{
		// һ��Ҫִ�п�������������ֱ��ʹ�õȺ��������ֵ
		
		std::lock_guard<std::mutex> lock(imageMutex);
		this->xyzMap.copyTo(xyz_map);

	}
}
