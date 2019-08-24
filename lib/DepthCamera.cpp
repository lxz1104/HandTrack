#include "stdafx.h"
#include "DepthCamera.h"

namespace ht {

	/**  点的最小深度(以米为单位)。小于此深度的点视为噪声。(0.0表示禁用) */
    const float DepthCamera::NOISE_FILTER_LOW = 0.15f;

	/** 点的最大深度(以米为单位)。大于此深度的点视为噪声。(0.0 表示禁用) */
    const float DepthCamera::NOISE_FILTER_HIGH = 0.6f;

    DepthCamera::~DepthCamera()
    {
        badInputFlag = true;
        endCapture();
    }

    void DepthCamera::beginCapture(int fps_cap, bool remove_noise)
	{
        assert(captureInterrupt == true);
        captureInterrupt = false;

		BOOST_LOG_TRIVIAL(info) << "正在启动深度图像获取线程...";
		/*this->thPool.enqueue(&DepthCamera::captureThreadingHelper, this, fps_cap,
			&captureInterrupt, remove_noise);*/
		/*std::thread thd(&DepthCamera::captureThreadingHelper, this, fps_cap,
			&captureInterrupt, remove_noise);
		thd.detach();*/

    }

    void DepthCamera::endCapture()
    {
        captureInterrupt = true;
    }

    bool DepthCamera::nextFrame(bool removeNoise)
    {
		// 初始化后台缓冲
        //initializeImages();

		// 使用后台缓冲区图像调用更新(允许在前端继续操作，防止出现数据冲突)
        this->update(this->xyzMap, this->rgbMap, this->irMap, this->ampMap, this->flagMap);

       /* if (!badInput() && xyzMapBuf.data) {
            if (removeNoise) {
                this->removeNoise(xyzMapBuf, ampMapBuf, flagMapConfidenceThreshold());
            }
        }*/

	
		// 交换时锁定所有缓冲区
		//std::lock_guard<std::mutex> lock(imageMutex);
		// 更新完成后,将后台缓冲区交换到前端
		//swapBuffers();
		
		

		//调用绑定的回调函数
        for (auto & callback : updateCallbacks) {
            callback.second(*this);
        }

        return !badInput();
    }


	/** 返回此深度摄像机实例的默认检测参数 */
    const DetectionParams::Ptr & DepthCamera::getDefaultParams() const {
        return DetectionParams::DEFAULT;
    }

	/** 在错误输入时返回 true */
    bool DepthCamera::badInput()
    {
        return badInputFlag;
    }

	/**
	 * 消除 zMap 和 xyzMap 上的噪声
	 */
    void DepthCamera::removeNoise(cv::Mat & xyz_map, cv::Mat & amp_map, float confidence_thresh)
    {
        for (int r = 0; r < xyz_map.rows; ++r)
        {
            Vec3f * ptr = xyz_map.ptr<Vec3f>(r);

            const float * ampptr = nullptr;
            if (amp_map.data) ampptr = amp_map.ptr<float>(r);

            for (int c = 0; c < xyz_map.cols; ++c)
            {
                if (ptr[c][2] > 0.0f) {
                    if (ptr[c][2] < NOISE_FILTER_LOW &&
                        (ptr[c][2] > NOISE_FILTER_HIGH || ptr[c][2] == 0.0) &&
                        (ampptr == nullptr || amp_map.data == nullptr ||
                            ampptr[c] < confidence_thresh)) {
                        ptr[c][0] = ptr[c][1] = ptr[c][2] = 0.0f;
                    }
                }
            }
        }
    }

    bool DepthCamera::isCapturing()
    {
        return !captureInterrupt;
    }

    int DepthCamera::addUpdateCallback(std::function<void(DepthCamera&)> func)
    {
        int id;
        if (updateCallbacks.empty()) {
            id = 0;
        }
        else {
            id = updateCallbacks.rbegin()->first + 1;
        }

        updateCallbacks[id] = func;
        return id;
    }

    void DepthCamera::removeUpdateCallback(int id)
    {
        updateCallbacks.erase(id);
    }

    cv::Size DepthCamera::getImageSize() const
    {
        return cv::Size(getWidth(), getHeight());
    }


    const std::string DepthCamera::getModelName() const {
        return "DepthCamera";
    }

    void DepthCamera::initializeImages()
    {
        cv::Size sz = getImageSize();

		// 初始化后台缓冲区
        xyzMapBuf.release();
        xyzMapBuf.create(sz, CV_32FC3);

        if (hasRGBMap()) {
            rgbMapBuf.release();
            rgbMapBuf.create(sz, CV_8UC3);
        }

        if (hasIRMap()) {
            irMapBuf.release();
            irMapBuf.create(sz, CV_8U);
        }

        if (hasAmpMap()) {
            ampMapBuf.release();
            ampMapBuf.create(sz, CV_32F);
        }

        if (hasFlagMap()) {
            flagMapBuf.release();
            flagMapBuf.create(sz, CV_8U);
        }
    }

	/** 交换单个缓冲区 */
    void DepthCamera::swapBuffer(bool (DepthCamera::* check_func)() const, cv::Mat & img, cv::Mat & buf)
    {
        if ((this->*check_func)()) {
            cv::swap(img, buf);
        }
        else {
            img.data = nullptr;
        }
    }

	/** 交换所有缓冲区 */
    void DepthCamera::swapBuffers()
    {
        cv::swap(xyzMap, xyzMapBuf);
		if (this->hasRGBMap()) {
			swapBuffer(&DepthCamera::hasRGBMap, rgbMap, rgbMapBuf);
		}
		if (this->hasIRMap()) {
			swapBuffer(&DepthCamera::hasIRMap, irMap, irMapBuf);
		}
		if (this->hasAmpMap())
		{
			swapBuffer(&DepthCamera::hasAmpMap, ampMap, ampMapBuf);
		}
		if (this->hasFlagMap()) {
			swapBuffer(&DepthCamera::hasFlagMap, flagMap, flagMapBuf);
		}
    }

    cv::Mat DepthCamera::getXYZMap()
    {
        /*std::lock_guard<std::mutex> lock(imageMutex);*/
		this->nextFrame(false);
        if (xyzMap.data == nullptr) return cv::Mat::zeros(getHeight(), getWidth(), CV_32FC3);
        return xyzMap;
    }

    cv::Mat DepthCamera::getAmpMap()
    {
        if (!hasAmpMap()) throw;

        /*std::lock_guard<std::mutex> lock(imageMutex);
        if (ampMap.data == nullptr) return cv::Mat::zeros(getHeight(), getWidth(), CV_32F);*/

		this->nextFrame(false);
		if (ampMap.data == nullptr) return cv::Mat::zeros(getHeight(), getWidth(), CV_32F);
        return ampMap;
    }

    cv::Mat DepthCamera::getFlagMap()
    {
        if (!hasFlagMap()) throw;

        //std::lock_guard<std::mutex> lock(imageMutex);
		this->nextFrame(false);
        if (flagMap.data == nullptr) return cv::Mat::zeros(getHeight(), getWidth(), CV_8U);
        return flagMap;
    }

    cv::Mat DepthCamera::getRGBMap() {
        if (!hasRGBMap()) throw;

        //std::lock_guard<std::mutex> lock(imageMutex);
		this->nextFrame(false);
        if (rgbMap.data == nullptr) return cv::Mat::zeros(getHeight(), getWidth(), CV_8UC3);
        return rgbMap;
    }

    cv::Mat DepthCamera::getIRMap()
    {
        if (!hasIRMap()) throw;

        //std::lock_guard<std::mutex> lock(imageMutex);
		this->nextFrame(false);
        if (irMap.data == nullptr) return cv::Mat::zeros(getImageSize(), CV_8U);
        return irMap;
    }

    bool DepthCamera::hasAmpMap() const
    {
		// 默认为false，可以重写
        return false;
    }

    bool DepthCamera::hasFlagMap() const
    {
		// 默认为false，可以重写
        return false;
    }

    bool DepthCamera::hasRGBMap() const {
		// 默认为false，可以重写
        return false;
    }

    bool DepthCamera::hasIRMap() const
    {
		// 默认为false，可以重写
        return false;
    }

	// 深度摄像机必须具有 XYZ map，否则无法使用算法

    int DepthCamera::ampMapInvalidFlagValue() const{
        return -1;
    }

    float DepthCamera::flagMapConfidenceThreshold() const{
        return 0.5;
    }

    void DepthCamera::captureThreadingHelper(int fps_cap, volatile bool * interrupt, bool remove_noise)
    {
        using namespace std::chrono;
        steady_clock::time_point lastTime;
        steady_clock::time_point currTime;
        float timePerFrame;
        if (fps_cap > 0) {
            timePerFrame = 1e9f / fps_cap;
            lastTime = steady_clock::now();
        }
		BOOST_LOG_TRIVIAL(info) << "深度图像获取线程已启动";
        while (interrupt == nullptr || !(*interrupt)) {
            this->nextFrame(remove_noise);
            // cap FPS
            if (fps_cap > 0) {
                currTime = steady_clock::now();
                steady_clock::duration delta = duration_cast<microseconds>(currTime - lastTime);

                if (delta.count() < timePerFrame) {
                    long long ms = (long long)(timePerFrame - delta.count()) / 1e6f;
                    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
                }
                lastTime = currTime;
            }
        }
		BOOST_LOG_TRIVIAL(info) << "深度图像获取线程已关闭";
    }
}
