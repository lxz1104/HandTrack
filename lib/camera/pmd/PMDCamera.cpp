//
// Created by lxz on 19-6-23.
//
#include "camera/pmd/PMDCamera.h"
#include "stdafx.h"


using namespace std;

namespace ht {

    //初始化相机
    void PMDCamera::initCamera(){
		BOOST_LOG_TRIVIAL(info) << "Trying to open pmd pico flexx...";
        //查询并连接相机
        {
            royale::CameraManager manager;
            //记录已连接的相机列表
            royale::Vector<royale::String> camList(manager.getConnectedCameraList());
			BOOST_LOG_TRIVIAL(info) << "Detected <" << camList.size() << "> " << " camera(s).";

            if (!camList.empty())
            {
				BOOST_LOG_TRIVIAL(info) << "CamID for first device: " << camList.at(0).c_str() << " with a length of (" << camList.at(0).length() << ")";
                this->cameraDevice = manager.createCamera (camList[0]);
            }
            else
            {
				BOOST_LOG_TRIVIAL(error) << "No suitable camera device detected."
										 << "Please make sure that a supported camera is plugged in, all drivers are "
										 << "installed, and you have proper USB permission";
                exit(EXIT_FAILURE);
            }

            camList.clear();
        }

        //检查相机状态：是否连接成功
        if(this->cameraDevice == nullptr){
            //相机不可用
			BOOST_LOG_TRIVIAL(error) << "Cannot create the camera device";
            exit(EXIT_FAILURE);
        }

		BOOST_LOG_TRIVIAL(info) << "Connect camera <0> successful!";

        //初始化相机
        auto status = this->cameraDevice->initialize();

        if(royale::CameraStatus::SUCCESS != status){
			BOOST_LOG_TRIVIAL(error) << "Cannot initialize the camera device, error string : "
                      << royale::getErrorString (status);
            exit(EXIT_FAILURE);
        }

		BOOST_LOG_TRIVIAL(info) << "Trying to update sensor...";

        //设置相机操作模式
        royale::Vector<royale::String> useCases;
        status = cameraDevice->getUseCases (useCases);  //获取相机操作模式
        if (status != royale::CameraStatus::SUCCESS || useCases.empty())
        {
			BOOST_LOG_TRIVIAL(error) << "No use cases are available";
			BOOST_LOG_TRIVIAL(error) << "getUseCases() returned: " << royale::getErrorString (status);
            exit(EXIT_FAILURE);
        }

        //默认操作模式
        auto selectedUseCaseIdx = 5u;

        //设置操作模式
        if (this->cameraDevice->setUseCase(useCases.at(selectedUseCaseIdx)) != royale::CameraStatus::SUCCESS)
        {
			BOOST_LOG_TRIVIAL(error) << "Error setting use case";
            exit(EXIT_FAILURE);
        }

		BOOST_LOG_TRIVIAL(info) << "Use case: " << useCases.at(selectedUseCaseIdx);

        // 就收数据流的ID
        royale::Vector<royale::StreamId> streamIds;
        if (this->cameraDevice->getStreams (streamIds) != royale::CameraStatus::SUCCESS)
        {
			BOOST_LOG_TRIVIAL(error) << "Error retrieving streams";
            exit(EXIT_FAILURE);
        }

		BOOST_LOG_TRIVIAL(info) << "Stream IDs : " << streamIds.at(0);

        // 设置相机参数
        royale::LensParameters lensParameters;
        status = this->cameraDevice->getLensParameters(lensParameters);
        if (status != royale::CameraStatus::SUCCESS)
        {
            BOOST_LOG_TRIVIAL(error) << "Can't read out the lens parameters";
            exit(EXIT_FAILURE);
        }

        this->listener.setLensParameters (lensParameters);

        //注册数据监听器
        if (cameraDevice->registerDataListener (&this->listener) != royale::CameraStatus::SUCCESS)
        {
			BOOST_LOG_TRIVIAL(error) << "Error registering data listener";
            exit(EXIT_FAILURE);
        }
        //开启相机捕获模式
        if (cameraDevice->startCapture() !=  royale::CameraStatus::SUCCESS)
        {
			BOOST_LOG_TRIVIAL(error) << "Error starting the capturing";
            exit(EXIT_FAILURE);
        }
    }
    /**
     * 构造函数
     */
    PMDCamera::PMDCamera()
    {
		BOOST_LOG_TRIVIAL(info) << "Init " << this->getModelName() << " Camera...";
        //初始化相机
        this->initCamera();
    }

    PMDCamera::~PMDCamera()
    {
        //停止捕获模式
        if (this->cameraDevice->stopCapture() != royale::CameraStatus::SUCCESS)
        {
			BOOST_LOG_TRIVIAL(error) << "Error starting the capturing";
			std::abort();
        }
    }


	/********************* 重写的基类方法 *****************/
	const std::string PMDCamera::getModelName() const {
		return "PMD";
	}

	int PMDCamera::getWidth() const {
		return PMDCamera::Depth_Width;
	}

	int PMDCamera::getHeight() const {
		return PMDCamera::Depth_Width;
	}

	float PMDCamera::flagMapConfidenceThreshold() const {
		return (60.0f / 255.0f * 500.0f);
	}

	int PMDCamera::ampMapInvalidFlagValue() const {
		return  0u;
	}

	bool PMDCamera::hasAmpMap() const
	{
		return false;
	}

	bool PMDCamera::hasFlagMap() const
	{
		return false;
	}

	/** !!此方法一定要重写 **/
    void PMDCamera::update(cv::Mat & xyz_map, cv::Mat & rgb_map, cv::Mat & ir_map,
                           cv::Mat & amp_map, cv::Mat & flag_map)
    {
		// 一定要执行拷贝操作，不能直接使用等号运算符赋值
		this->listener.getXYZMap(xyz_map);
		this->listener.getAmpMap(amp_map);
    }
}
