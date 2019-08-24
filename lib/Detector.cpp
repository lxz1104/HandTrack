#include "stdafx.h"
#include "Detector.h"
#include "DetectionParams.h"

namespace ht {
    Detector::Detector(DetectionParams::Ptr params)
        : params(params ? params : DetectionParams::DEFAULT) {
		BOOST_LOG_TRIVIAL(info) << "初始化探测器: Detector::Detector(DetectionParams::Ptr params)";
		// 将回调函数的方法与实例绑定
		callback = std::bind(&Detector::callbackHelper, this, std::placeholders::_1);
    } 

    void Detector::update(const cv::Mat & image)
    {
		// 更新图像并对图像进行目标检测
        this->image = image;
        detect(this->image);
        lastCamera = nullptr;
        onSameFrame = false;
    }

    void Detector::update(DepthCamera & camera)
    {
		// 如果相机机仍位于与之前相同的帧上（即图像的内容未更新）,则直接返回
        if (onSameFrame && lastCamera == &camera) return;
        this->image = camera.getXYZMap();
		// 执行目标检测
        this->detect(this->image);

		// 如果绑定的相机发生改变，则更新相机的回调函数
        if (lastCamera != &camera) {
            if (lastCamera) lastCamera->removeUpdateCallback(lastUpdateCallbackID);
            lastUpdateCallbackID = camera.addUpdateCallback(callback);
        }

        lastCamera = &camera;
    }

    void Detector::setParams(const DetectionParams::Ptr params)
    {
        this->params = params;
    }

    void Detector::callbackHelper(DepthCamera & camera)
    {
        onSameFrame = false;
    }
}
