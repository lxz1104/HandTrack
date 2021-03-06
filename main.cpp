﻿#include "Core.h"
#include "camera/pmd/PMDCamera.h"
#include "camera/axon/AxonCamera.h"

using namespace ht;

int main() {
	// 初始化相机实例
	//DepthCamera::Ptr camera = std::make_shared<ht::camera::PMDCamera>();	// PMD camera
	DepthCamera::Ptr camera = std::make_shared<ht::camera::AXonCamera>();	// AXon camera

	// 初始化检测参数
	DetectionParams::Ptr params = camera->getDefaultParams(); //使用默认参数

	// 初始化检测器
	HandDetector::Ptr handDetector = std::make_shared<HandDetector>();

	// 操作选项
	bool showBackground = true, useSVM = false, useEdgeConn = false;

	cv::Mat scaledZImage;

	// fps信息
	const int FPS_CYCLE_FRAMES = 8; // 平均多少帧更新一次帧率
	using ms = std::chrono::duration<float, std::milli>;
	using time_point = std::chrono::high_resolution_clock::time_point;

	std::chrono::high_resolution_clock timer = std::chrono::high_resolution_clock();
	time_point currCycleStartTime = timer.now(); // 当前周期的开始时间

	float currFPS = 0.0;	// 当前的帧率

	int currFrame = 0;		// 当前位于第几帧

	char chr[64] = { '0' };

	// 主进程
	while (true)
	{
		++currFrame;
		// 从相机获取最新的XYZMap
		cv::Mat xyzMap = camera->getXYZMap();

		/**** 平面检测部分 ****/

		// 查询当前帧中的检测对象
		params->handUseSVM = useSVM;
		params->handRequireEdgeConnected = useEdgeConn;

		// 手部对象
		std::vector<Hand::Ptr> hands;
		
		// 设置检测参数
		handDetector->setParams(params);

	
		handDetector->update(*camera);
		hands = handDetector->getHands();
		

		/**** 可视化部分 ****/

		// 构可视化图像
		cv::Mat handVisual;

		// 可视化背景
		//xyzMap = cv::Scalar::all(0);
		if (showBackground) {
			Visualizer::visualizeXYZMap(xyzMap, handVisual, 1.5f);
		}
		else
		{
			cv::Mat clearBackground(cv::Size(xyzMap.cols, xyzMap.rows), CV_32FC3);
			clearBackground = cv::Scalar::all(0);
			Visualizer::visualizeXYZMap(clearBackground, handVisual, 1.5f);
		}
		
		Visualizer::visualizeXYZMap(xyzMap, xyzMap, 1.0f);
		
		const cv::Scalar WHITE(255, 255, 255);

		// 或者检测到的手的特征
		int i = 0;
		for (Hand::Ptr & hand : hands) {
			if (!handVisual.empty()) {
					// 可视化检测到的手
					Visualizer::visualizeHand(handVisual, handVisual, hand.get());
					std::map<std::string, Vec3f> LMap = hand->getLHlabelXYZ();
					
					// 输出三维坐标
					/*for (auto item : LMap)
					{
						BOOST_LOG_TRIVIAL(info) << item.first << ": " << item.second;
					}*/
			}
			++i;
			if (i >= 2) {
				break;
			}
		}


		if (hands.size() > 0) {
			// 显示检测到的手的数量
			cv::putText(handVisual, std::to_string(hands.size()) +
				util::pluralize(" Hand", hands.size()),
				Point2i(10, 25), 0, 0.5, WHITE);
		}

		// update FPS
		if (currFrame % FPS_CYCLE_FRAMES == 0) {
			time_point now = timer.now();
			currFPS = (float)FPS_CYCLE_FRAMES * 1000.0f / std::chrono::duration_cast<ms>(now - currCycleStartTime).count();
			currCycleStartTime = now;
		}
		
		sprintf(chr, "FPS: %02.3f", currFPS);
		cv::putText(handVisual, chr, Point2i(handVisual.cols - 120, 25), 0, 0.5, cv::Scalar(0, 255, 255));
		
		// 可视化手、平面、XYZMap
		if (!xyzMap.empty()) {
			scaledZImage.create(cv::Size(ht::camera::AXonCamera::Depth_Width * 1, ht::camera::AXonCamera::Depth_Height * 1), CV_8UC1);
			resize(xyzMap, scaledZImage, scaledZImage.size());
			cv::imshow(camera->getModelName() + " Depth Map", scaledZImage);
			if (!handVisual.empty()) {
				scaledZImage.create(cv::Size(ht::camera::AXonCamera::Depth_Width * 1, ht::camera::AXonCamera::Depth_Height * 1), CV_8UC1);
				resize(handVisual, scaledZImage, scaledZImage.size());
				cv::imshow("Demo Output", scaledZImage);
			}
			
		}

		/**** 模式切换部分 ****/
		int c = cv::waitKey(1);

		// 不区分大小写
		if (c >= 'a' && c <= 'z') c &= 0xdf;

		// 按下ESC或者Q
		if (c == 'Q' || c == 27) {
			/*** 退出程序 ***/
			break;
		}
		// 切换模式
		switch (c) {
		case 'H':
			showBackground ^= 1;
			BOOST_LOG_TRIVIAL(info) << "Show Background: " << (showBackground ? "True" : "False");
			break;
		case 'S':
			useSVM ^= 1; 
			BOOST_LOG_TRIVIAL(info) << "USE SVM: " << (useSVM ? "True" : "False");
			break;
		case 'U':
			useEdgeConn ^= 1;
			break;
		}
	}
	cv::destroyAllWindows();
	return 0;
}