//
// Created by lxz on 19-9-19.
//

// Header
#include "camera/axon/AxonCamera.h"

// Boost.log Libraries
#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>

namespace ht {

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
		this->device.getProperty(ONI_DEVICE_PROPERTY_DRIVER_VERSION,&driver,&nsize);	// 获取驱动版本号
		BOOST_LOG_TRIVIAL(info) << "AXon driver version: " << driver.major << "." 
								<< driver.minor << "." << driver.maintenance << "." << driver.build;

		/** 设置RGB和Depth对齐  */
		openni::ImageRegistrationMode mode = this->device.getImageRegistrationMode();
		if (mode != openni::IMAGE_REGISTRATION_COLOR_TO_DEPTH) {
			this->device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_COLOR_TO_DEPTH); //对齐方式color to depth
		}
		AXonLinkCamParam camParam;
		int dataSize = sizeof(AXonLinkCamParam);
		rc = this->device.getProperty(AXONLINK_DEVICE_PROPERTY_GET_CAMERA_PARAMETERS,
			&camParam, &dataSize);

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

		/** 创建RGB流 */
		if (this->device.getSensorInfo(openni::SENSOR_COLOR) != nullptr)
		{
			// 有RGB Sensor则创建RGB流
			rc = this->color.create(device, openni::SENSOR_COLOR);
			if (rc == openni::STATUS_OK)
			{
				/** 开启RGB流 */
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

		/** 创建IR流 */
		if (this->device.getSensorInfo(openni::SENSOR_IR) != nullptr)
		{
			// 有IR Sensor则创建IR流
			rc = this->ir.create(device, openni::SENSOR_IR);
			if (rc == openni::STATUS_OK)
			{
				/** 开启IR流 */
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
		
		/** 检查深度流是否合法 */
		if (!(this->depth.isValid()) || (!this->color.isValid()) || (!this->ir.isValid()))
		{
			BOOST_LOG_TRIVIAL(error) << "No valid streams. Exiting\n";
			openni::OpenNI::shutdown();
			exit(EXIT_FAILURE);
		}
		
		/** 开启深度图像镜像 */
		//this->depth.setMirroringEnabled(true);
		/** 设置流同步 */
		this->device.setDepthColorSyncEnabled(true); 
		/** 设置对齐方式 */
		this->device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_COLOR_TO_DEPTH);
	
		/** 从流中获取帧数据 */
		openni::VideoStream* allStream[3] = { NULL };
		allStream[0] = &this->depth;
		allStream[1] = &this->color;
		allStream[2] = &this->ir;

		while (!this->waitAllStream(allStream, 1, 200)) //等待三个流都拿到帧
		{
			;
		}
		openni::VideoFrameRef colorFrame, depthFrame, irFrame;
		this->color.readFrame(&colorFrame);//取走三个流的帧，注意：设置同步之后，必须所有流的帧都必须读走，否则同步不上，会导致永远在等待
		this->depth.readFrame(&depthFrame);
		this->ir.readFrame(&irFrame);
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
	AXonCamera::AXonCamera():isCapture(true)
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
		this->color.stop();
		this->ir.stop();
		/** 销毁所有流 */
		this->depth.destroy();
		this->color.destroy();
		this->ir.destroy();
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
		if (!device.getDepthColorSyncEnabled())
			device.setDepthColorSyncEnabled(true);
		BOOST_LOG_TRIVIAL(info) << "设备连接状态: OK";

		openni::VideoMode colorVideoMode = this->color.getVideoMode();
		colorVideoMode.setResolution(640, 480);  //设置三个流相同分辨率
		this->color.setVideoMode(colorVideoMode);
		openni::VideoMode depthVideoMode = this->depth.getVideoMode();
		depthVideoMode.setResolution(640, 480);
		this->depth.setVideoMode(depthVideoMode);
		openni::VideoMode irVideoMode = this->ir.getVideoMode();
		irVideoMode.setResolution(640, 480);
		this->ir.setVideoMode(irVideoMode);

		openni::ImageRegistrationMode mode = this->device.getImageRegistrationMode();
		if (mode != openni::IMAGE_REGISTRATION_COLOR_TO_DEPTH)
			this->device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_COLOR_TO_DEPTH); //对齐方式color to depth


		AXonLinkCamParam camParam;
		int dataSize = sizeof(AXonLinkCamParam);
		openni::Status rc = openni::STATUS_OK;
		rc = this->device.getProperty(AXONLINK_DEVICE_PROPERTY_GET_CAMERA_PARAMETERS,
			&camParam, &dataSize);
		CamIntrinsicParam* depthIntParam = GetDepthInstrinsicParamByResolution(640, 480, &camParam);// 深度帧参数


		// 记录帧数据
		openni::VideoFrameRef colorFrame, depthFrame, irFrame;
		
		int temp;
		BOOST_LOG_TRIVIAL(info) << "图像捕获线程启动.";
		while (this->isCapture)
		{
			openni::VideoStream* pStream = &this->depth;
			rc = openni::OpenNI::waitForAnyStream(&pStream, 1, &temp,2000);    /*等待流*/
			if (rc != openni::STATUS_OK)
			{
				BOOST_LOG_TRIVIAL(error) << "Wait failed! (timeout is 2000 ms): "
					<< openni::OpenNI::getExtendedError();
				return;
			}

			/*读深度流帧数据*/
			rc = this->depth.readFrame(&depthFrame);
			rc = this->color.readFrame(&colorFrame);
			rc = this->ir.readFrame(&irFrame);
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

			openni::PixelFormat format = this->depth.getVideoMode().getPixelFormat();
			float depth_unit = openni::OpenNI::getDepthValueUnit_mm(format);

			/** 当前帧的深度数据 */
			const openni::DepthPixel* depthImage = (const openni::DepthPixel*)depthFrame.getData();

			// 将图像矩阵置零
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
				// 将后台缓冲交换到前台
				std::lock_guard<std::mutex> lock(imageMutex);
				cv::swap(this->xyzMap, this->xyzBuffer);
			}
			
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
		
		std::lock_guard<std::mutex> lock(imageMutex);
		this->xyzMap.copyTo(xyz_map);

	}
}
