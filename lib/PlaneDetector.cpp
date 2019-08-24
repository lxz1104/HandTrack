#include "stdafx.h"
#include "Util.h"
#include "PlaneDetector.h"

namespace ark {
    PlaneDetector::PlaneDetector(DetectionParams::Ptr params) : Detector(params) { }

    cv::Mat PlaneDetector::getNormalMap()
    {
        return normalMap;
    }

    void PlaneDetector::detect(cv::Mat & image)
    {
        planes.clear();

        std::vector<Vec3f> equations;
        std::vector<VecP2iPtr> points;
        std::vector<VecV3fPtr> pointsXYZ;

        util::computeNormalMap(image, normalMap, 8, params->normalResolution, false);
        detectPlaneHelper(image, normalMap, equations, points, pointsXYZ, params);

        for (uint i = 0; i < equations.size(); ++i) {
            FramePlane::Ptr planePtr = std::make_shared<FramePlane>
                (equations[i], points[i], pointsXYZ[i], image, params);

            if (planePtr->getSurfArea() > params->planeMinArea) {
                planes.emplace_back(planePtr);
            }
        }

        std::sort(planes.begin(), planes.end(), [](const FramePlane::Ptr & a, const FramePlane::Ptr & b) {
            return a->getPoints().size() > b->getPoints().size();
        });

        // 已检测完平面，如果启用调试标志，则显示可视化效果
#ifdef DEBUG
        cv::Mat planeDebugVisual =
            cv::Mat::zeros(image.size() / params->normalResolution, CV_8UC3);

        for (int i = 0; i < planes.size(); ++i) {
            Vec3b color = util::paletteColor(i);
            const std::vector<Point2i> & points = planes[i]->getPointsIJ();

            for (uint j = 0; j < points.size(); ++j) {
                planeDebugVisual.at<Vec3b>(points[j] / params->normalResolution) = color;
            }
            const std::vector<Point2f> & rect = planes[i]->getPlaneBoundingRect();
            for (int i = 0; i < 4; ++i) {
                cv::line(planeDebugVisual, rect[i] / params->normalResolution,
                    rect[(i + 1) & 3] / params->normalResolution, cv::Scalar(255, 0, 0));
            }
        }

        cv::resize(planeDebugVisual, planeDebugVisual, planeDebugVisual.size() * params->normalResolution,
            0, 0, cv::INTER_NEAREST);

        for (int i = 0; i < planes.size(); ++i) {
            cv::putText(planeDebugVisual, std::to_string(planes[i]->getSurfArea()), planes[i]->getCenterIJ(), 0, 0.5, cv::Scalar(255, 255, 255));
        }

        cv::imshow("[Plane Debug]", planeDebugVisual);
#endif
    }

    void PlaneDetector::detectPlaneHelper(const cv::Mat & xyz_map, const cv::Mat & normal_map,
        std::vector<Vec3f> & output_equations, 
        std::vector<VecP2iPtr> & output_points, std::vector<VecV3fPtr> & output_points_xyz, 
        DetectionParams::Ptr params)
    { 
        // 1. 初始化
        const int R = xyz_map.rows, C = xyz_map.cols, N = R * C;

        // 初始化保存漫水填充的矩阵
        cv::Mat floodFillMap(R, C, CV_8U);
        const Vec3f * ptr; uchar * visPtr;
        for (int r = 0; r < R; ++r)
        {
            visPtr = floodFillMap.ptr<uchar>(r);
            ptr = xyz_map.ptr<Vec3f>(r);
            for (int c = 0; c < C; ++c)
            {
                visPtr[c] = ptr[c][2] > 0 ? 255 : 0;
            }
        }

        int compId = -1;
        std::vector<Point2i> allIndices;
        allIndices.reserve(N);

		// 2. 寻找“子平面”，即每一帧中可见的所有平面对象，并组合相似的对象。
        // 储存每个平面上的点（像素坐标）
        std::vector<std::shared_ptr<std::vector<Point2i> > > planePointsIJ;

        // 储存每个平面上的点（以三维坐标表示）
        std::vector<std::shared_ptr<std::vector<Vec3f> > > planePointsXYZ;

        // 平面方程：ax+by-z+c=0
        std::vector<Vec3f> planeEquation;

		// 计算一些常量
        const int SUBPLANE_MIN_POINTS = static_cast<const int>(params->subplaneMinPoints * N /
            (params->normalResolution * params->normalResolution));

        const int PLANE_MIN_POINTS = static_cast<const int>(params->planeMinPoints * N /
            (params->normalResolution * params->normalResolution));

        const int PLANE_MIN_INLIERS = static_cast<const int>(params->planeEquationMinInliers * N /
            (params->normalResolution * params->normalResolution));

        for (int r = 0; r < R; r += params->normalResolution) {
            visPtr = floodFillMap.ptr<uchar>(r);

            for (int c = 0; c < C; c += params->normalResolution) {
                if (visPtr[c] == 0) continue;

                Point2i pt(c, r);
				// 漫水填充法线
                int numPts = util::floodFill(normal_map, pt, params->planeFloodFillThreshold,
                                             &allIndices, nullptr, nullptr,
                                             params->normalResolution, 0, 0.0f, &floodFillMap, true);
				//std::cout << numPts << endl;
                if (numPts >= SUBPLANE_MIN_POINTS) {
                    std::vector<Vec3f> allXyzPoints(numPts);

                    for (int k = 0; k < numPts; ++k) {
                        allXyzPoints[k] = xyz_map.at<Vec3f>(allIndices[k]);;
                    }

                    // find surface area
					// 查找表面积
                    util::radixSortPoints(allIndices, C, R, numPts, &allXyzPoints);
                    double surfArea = util::surfaceArea(normal_map.size(), allIndices,
                        allXyzPoints, numPts);

                    if (surfArea < params->subplaneMinArea) {
                        continue;
                    }


					// 查找平面方程
                    Vec3f eqn = util::linearRegression(allXyzPoints, numPts);

					// 合并相似的子平面
                    uint i;
                    for (i = 0; i < planeEquation.size(); ++i) {
                        if (util::norm(planeEquation[i] - eqn) < params->planeCombineThreshold) {
                            // 找到相似的子平面，所以把它们组合起来
                            break;
                        }
                    }

					// 指向储存平面IJ/XYZ坐标点容器的指针
                    std::shared_ptr<std::vector<Point2i>> pointStore;
                    std::shared_ptr<std::vector<Vec3f>> pointStoreXyz;

                    if (i >= planeEquation.size()) {
                        // 未发现类似平面
                        planeEquation.push_back(eqn);
                        planePointsIJ.emplace_back(std::make_shared<std::vector<Point2i> >());
                        planePointsXYZ.emplace_back(std::make_shared<std::vector<Vec3f> >());
                        pointStore = *planePointsIJ.rbegin();
                        pointStoreXyz = *planePointsXYZ.rbegin();
                    }
                    else {
                        // 发现类似平面
                        pointStore = planePointsIJ[i];
                        pointStoreXyz = planePointsXYZ[i];
                    }

                    // 保存要存储的平面点
                    size_t start = pointStore->size();
                    pointStore->resize(start + numPts);
                    pointStoreXyz->resize(start + numPts);

                    for (size_t i = 0; i < numPts; ++i) {
                        pointStore->at(start + i) = allIndices[i];
                        pointStoreXyz->at(start + i) = allXyzPoints[i];
                    }
                }
            }
        }

        // 3. find equations of the combined planes and construct Plane objects with the data
		// 3.求组合平面的方程并用数据构造平面对象
        for (unsigned i = 0; i < planeEquation.size(); ++i) {
            size_t SZ = planePointsIJ[i]->size();
            if (SZ < PLANE_MIN_POINTS) continue;

            std::vector<Vec3f> pointsXYZ;
            util::removeOutliers(*planePointsXYZ[i], pointsXYZ, params->planeOutlierRemovalThreshold);

            planeEquation[i] = util::linearRegression(pointsXYZ);

            int goodPts = 0;

            for (size_t j = 0; j < SZ; ++j) {
                float norm = util::pointPlaneSquaredDistance((*planePointsXYZ[i])[j], planeEquation[i]);
                if (norm < params->handPlaneMinSqrDist) {
                    ++goodPts;
                }
            }
            if (goodPts < PLANE_MIN_INLIERS) continue;

			// 添加到结果
            output_points.push_back(planePointsIJ[i]);
            output_points_xyz.push_back(planePointsXYZ[i]);
            output_equations.push_back(planeEquation[i]);
        }
    }

    const std::vector<FramePlane::Ptr> & PlaneDetector::getPlanes() const {
        return planes;
    }
}
