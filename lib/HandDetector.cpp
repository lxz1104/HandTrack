#include "stdafx.h"
#include "HandDetector.h"
#include "Util.h"

namespace ht {
    HandDetector::HandDetector(DetectionParams::Ptr params)
        : Detector(params) {
       
    }

    const std::vector<Hand::Ptr> & HandDetector::getHands() const {
        return this->m_hands;
    }

    void HandDetector::detect(cv::Mat & image)
    {
		this->m_hands.clear();
        // 1. 初始化元素
        const int R = image.rows, C = image.cols;
        cv::Mat floodFillMap(R, C, CV_8U);

        const Vec3f * ptr;
        uchar * visPtr;

        for (int r = 0; r < R; ++r)
        {
            visPtr = floodFillMap.ptr<uchar>(r);
            ptr = image.ptr<Vec3f>(r);
            for (int c = 0; c < C; ++c)
            {
                visPtr[c] = ptr[c][2] > 0 ? 255 : 0;
            }
        }
		
		
        // 3. 漫水填充点云
        std::shared_ptr<Hand> bestHandObject;
        float closestHandDist = FLT_MAX;

        std::vector<Point2i> allIJPoints;
        std::vector<Vec3f> allXYZPoints;

        allIJPoints.reserve(size_t(R) * C);
        allXYZPoints.reserve(size_t(R) * C);

#ifdef DEBUG
        cv::Mat floodFillVis = cv::Mat::zeros(R, C, CV_8UC3);
        int compID = 0;
#endif

		// 使用检测参数计算集群中的点的最小数量
        const int CLUSTER_MIN_POINTS = (int)(params->handClusterMinPoints * R * C);
        for (int r = 0; r < R; r += params->handClusterInterval)
        {
            auto * ptr = image.ptr<Vec3f>(r);
			auto * visPtr = floodFillMap.ptr<uchar>(r);

            for (int c = 0; c < C; c += params->handClusterInterval)
            {
                if (visPtr[c] > 0 && ptr[c][2] > 0 && ptr[c][2] <= params->handMaxDepth)
                {
                    int points_in_comp = util::floodFill(image, Point2i(c, r),
                        params->handClusterMaxDistance,
                        &allIJPoints, &allXYZPoints, nullptr, 1, 7,
                        params->handClusterMaxDistance * 8, &floodFillMap);

                    if (points_in_comp >= CLUSTER_MIN_POINTS)
                    {
                        VecP2iPtr ijPoints = std::make_shared<std::vector<Point2i>>(allIJPoints);
                        VecV3fPtr xyzPoints = std::make_shared<std::vector<Vec3f>>(allXYZPoints);

						// 4. 对于每个集群检测是否为手

						// 如果满足所需条件，则构造3D手部对象
                        Hand::Ptr handPtr = std::make_shared<Hand>(ijPoints, xyzPoints, image,
                            params, false, points_in_comp);
						
                        if (ijPoints->size() < CLUSTER_MIN_POINTS) continue;
						 
#ifdef DEBUG
                        cv::Vec3b color = util::paletteColor(compID++);
                        for (uint i = 0; i < points_in_comp; ++i) {
                            floodFillVis.at<Vec3b>(allIJPoints[i]) = color;
                        }

                        if (handPtr->getWristIJ().size() >= 2) {
                            cv::circle(floodFillVis, handPtr->getWristIJ()[0], 5, cv::Scalar(100, 255, 255));
                            cv::circle(floodFillVis, handPtr->getWristIJ()[1], 5, cv::Scalar(100, 255, 255));
                            cv::circle(floodFillVis, handPtr->getPalmCenterIJ(), handPtr->getCircleRadius(), cv::Scalar(100, 50, 255));
                        }
#endif

                        if (handPtr->isValidHand()) {
                            float distance = handPtr->getDepth();

                            if (distance < closestHandDist) {
                                bestHandObject = handPtr;
                                closestHandDist = distance;
                            }

#ifdef DEBUG
                            cv::polylines(floodFillVis, handPtr->getContour(), true, cv::Scalar(255, 255, 255));
#endif
                            if (handPtr->getSVMConfidence() >
                                params->handSVMHighConfidenceThresh ||
                                !params->handUseSVM) {
								// 避免重复的手
                                if (bestHandObject == handPtr) bestHandObject = nullptr;
								this->m_hands.push_back(handPtr);
                            }
                        }
                    }
                }
            }
        }

        if (bestHandObject != nullptr) {
            // if no hands surpass 'high confidence threshold', at least add one hand
			// 如果没有手超过好置信值，则至少增加一只手
			this->m_hands.push_back(bestHandObject);
        }

		// 对找到的手部对象进行排序
        if (params->handUseSVM) {
			// 按置信度大小排序
            std::sort(this->m_hands.begin(), this->m_hands.end(), [](Hand::Ptr a, Hand::Ptr b) {
                return a->getSVMConfidence() > b->getSVMConfidence();
            });
        }
        else {
			// 按手的远近排序
            std::sort(this->m_hands.begin(), this->m_hands.end(), [](Hand::Ptr a, Hand::Ptr b) {
                return a->getDepth() < b->getDepth();
            });
        }
#ifdef DEBUG
        cv::imshow("[Hand Flood Fill Debug]", floodFillVis);
#endif
    }
}
