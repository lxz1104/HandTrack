#include "stdafx.h"
#include "Util.h"
#include "Hand.h"
#include "HandClassifier.h"

// 仅限用于本文件范围
namespace {
	/**
	* 按角度顺序排列缺陷的比较器
	*/
	class DefectComparer {
	public:
		/**
		* 创建一个新的缺陷比较器
		* @param contour 计算缺陷的轮廓
		* @param defects 储存缺陷的容器
		* @param center 计算坡度的中心点
		*/
		DefectComparer(const std::vector<ht::Point2i>& contour,
			const std::vector<cv::Vec4i>& defects, const ht::Point2i& center) {
			angle.resize(contour.size());

			for (unsigned i = 0; i < defects.size(); ++i) {
				ht::Point2i pt = contour[defects[i][ATTR_USED]] - center;
				angle[defects[i][ATTR_USED]] = ht::util::pointToAngle(pt);
			}
		}

		/**
		* 比较两个缺陷（从底部逆时针方向最小的缺陷判定为较小）
		*/
		bool operator()(cv::Vec4i a, cv::Vec4i b) const {
			int idxA = a[ATTR_USED], idxB = b[ATTR_USED];
			return angle[idxA] > angle[idxB];
		}

	private:
		/**
		* 用于比较的vec4i索引
		*/
		const int ATTR_USED = 2;

		/**
		* 存储轮廓上所有点的坡度
		*/
		std::vector<double> angle;

		// 已禁用默认构造函数
		DefectComparer() {};
	};
}

namespace ht {

	static const std::string SVM_PATH = "config/hand-svm";
	// 初始化SVM手验证程序
	static const classifier::SVMHandValidator& handValidator = classifier::SVMHandValidator(SVM_PATH);

	Hand::Hand()
	{

	}

	Hand::Hand(const cv::Mat& cluster_depth_map, DetectionParams::Ptr params)
		: FrameObject(cluster_depth_map, params)
	{
		// 确定群集是否是一只手
		isHand = checkForHand();
	}

	Hand::Hand(VecP2iPtr points_ij, VecV3fPtr points_xyz, const cv::Mat& depth_map, DetectionParams::Ptr params, bool sorted, int points_to_use)
		: FrameObject(points_ij, points_xyz, depth_map, params, sorted, points_to_use)
	{
		// 确定群集是否是一只手
		isHand = checkForHand();
	}

	Hand::~Hand() { }

	int Hand::getNumFingers() const {
		return (int)fingersXYZ.size();
	}

	bool Hand::checkForHand()
	{
		checkEdgeConnected();

		 // 如果未连接到边缘，则停止
		if (params->handRequireEdgeConnected && !leftEdgeConnected && !rightEdgeConnected) {
			BOOST_LOG_TRIVIAL(debug) << "未连接到边缘";
			return false;
		}

		if (points->size() == 0 || num_points == 0) {
			BOOST_LOG_TRIVIAL(debug) << "集群大小为零";
			return false;
		}
		this->computeContour(xyzMap, points.get(), points_xyz.get(), topLeftPt, num_points);


		/** 寻找集群的中心 */
		Point2i centroid = findCenter(contour) - topLeftPt;

		// 确保中心点在集群内
		centroid = util::nearestPointOnCluster(xyzMap, centroid);

		// 查找中心上方最大内接圆的半径和中心点
		Vec3f topPt = util::averageAroundPoint(xyzMap, (*points)[0] - topLeftPt,
			params->xyzAverageSize);

		// 最大内接圆半径
		double cirrad;
		// 查找最大内接圆的中心（完整深度图左上角的坐标）
		Point2i circen = util::largestInscribedCircle(contour, xyzMap, getBoundingBox(), topPt,
			params->centerMaxDistFromTop, &cirrad);

		Point2i center = circen - topLeftPt;
		this->palmCenterIJ = circen;
		this->palmCenterXYZ = util::averageAroundPoint(xyzMap, center, params->xyzAverageSize);
		this->circleRadius = cirrad;

		/** 寻找手腕点位置 */
		int wristL = -1, wristR = -1;
		Point2i wristL_ij, wristR_ij;
		Vec3f wristL_xyz, wristR_xyz;

		// 1.获取用于手腕检测的种子点
		//    - 如果连接到边缘，则最左侧和右侧连接最紧密的等值点
		//    - 否则，获取最低等值点
		int contactL = -1, contactR = -1, direction = 1;

		const int lMargin = params->contactSideEdgeThresh,
			rMargin = fullMapSize.width - params->contactSideEdgeThresh;

		for (size_t i = 0; i < contour.size(); ++i) {
			Point2i& pt = contour[i];

			if (touchingEdge()) {
				if (pt.y > fullMapSize.height * params->handEdgeConnectMaxY &&
					util::pointOnEdge(fullMapSize, pt, params->contactBotEdgeThresh,
						params->contactSideEdgeThresh)) {

					if (contactL == -1) {
						contactL = contactR = (int)i;
						continue;
					}

					const Point2i& ccl = contour[contactL], & ccr = contour[contactR];

					if (pt.x <= lMargin) {
						if (ccl.x > lMargin || ccl.y > pt.y) contactL = (int)i;
						if (ccr.x <= lMargin && ccr.y < pt.y) contactR = (int)i;
					}
					else if (pt.x >= rMargin) {
						if (ccr.x < rMargin || ccr.y > pt.y) contactR = (int)i;
						if (ccl.x >= rMargin && ccl.y < pt.y) contactL = (int)i;
					}
					else {
						if (ccl.x > pt.x) contactL = (int)i;
						if (ccr.x < pt.x) contactR = (int)i;
					}
				}
			}
			else {
				if (contactL == -1 || pt.y > contour[contactL].y) {
					contactL = contactR = (int)i;
				}
			}
		}

		if (contactL >= 0 && contactR >= 0) {
			this->contactL_ij = contour[contactL];
			this->contactR_ij = contour[contactR];
			// 求手臂中点的IJ坐标
			this->armmid = (contactL_ij + contactR_ij) / 2;

			// step 2: 从 lci 和 rci移动的方向检测
			// direction: 1 = lci +, rci -; -1 = lci -, rci +
			if (((contactR > contactL) && (contactR - contactL < (int)contour.size() / 2)) ||
				((contactR <= contactL) && (contactL - contactR >= (int)contour.size() / 2))) {
				direction = -1;
			}
			// 3.朝向移动，直到接近中心
			size_t i = contactL;
			do {
				Point2i pt = contour[i];
				Vec3f xyz = util::averageAroundPoint(xyzMap,
					pt - topLeftPt, params->xyzAverageSize);

				float dist = util::euclideanDistance(xyz, this->palmCenterXYZ);

				if (dist <= params->wristCenterDistThresh) {
					wristL = (int)i;
					break;
				}

				i = (i + direction + contour.size()) % contour.size();
			} while (i != contactR);

			i = contactR;
			do {
				Point2i pt = contour[i];
				Vec3f xyz = util::averageAroundPoint(xyzMap,
					pt - topLeftPt, params->xyzAverageSize);

				float dist = util::euclideanDistance(xyz, this->palmCenterXYZ);

				if (dist <= params->wristCenterDistThresh) {
					wristR = (int)i;
					break;
				}

				i = (i - direction + contour.size()) % contour.size();
			} while (i != contactL);

			if (contour[wristL].x > contour[wristR].x) {
				std::swap(wristL, wristR);
			}

		}
		else if (wristL < 0 || wristR < 0)
		{
			BOOST_LOG_TRIVIAL(debug) << "未找到手腕";
			return false;
		}

		wristL_ij = contour[wristL];
		wristR_ij = contour[wristR];
		wristL_xyz = util::averageAroundPoint(xyzMap,
			wristL_ij - topLeftPt, params->xyzAverageSize);
		wristR_xyz = util::averageAroundPoint(xyzMap,
			wristR_ij - topLeftPt, params->xyzAverageSize);

		float wristWidth = util::euclideanDistance(wristL_xyz, wristR_xyz);

		// 保存手腕点坐标
		this->wristXYZ.push_back(wristL_xyz);
		this->wristXYZ.push_back(wristR_xyz);
		this->wristIJ.push_back(wristL_ij);
		this->wristIJ.push_back(wristR_ij);

		// 消除手腕长度不符合要求的手的长度
		if (wristWidth < params->wristWidthMin || wristWidth > params->wristWidthMax) {
			BOOST_LOG_TRIVIAL(debug) << "手腕宽度不符（" << wristWidth << "m）";
			return false;
		}

		// 求手腕中点的IJ和XYZ坐标
		this->wristmidXYZ[0] = (wristXYZ[0][0] + wristXYZ[1][0]) / 2;
		this->wristmidXYZ[1] = (wristXYZ[0][1] + wristXYZ[1][1]) / 2;
		this->wristmidXYZ[2] = (wristXYZ[0][2] + wristXYZ[1][2]) / 2;
		this->wristmidIJ = (wristIJ[0] + wristIJ[1]) / 2;


		/** 移除手腕点以下的部分 */

		std::vector<Point2i> aboveWristPointsIJ;
		std::vector<Vec3f> aboveWristPointsXYZ;

		if (wristR_ij.x != wristL_ij.x) {
			double slope = ((double)wristR_ij.y - (double)wristL_ij.y) / ((double)wristR_ij.x - (double)wristL_ij.x);

			for (int i = 0; i < num_points; ++i) {
				const Point2i& pt = (*points)[i];
				double y_hat = wristL_ij.y + ((double)pt.x - wristL_ij.x) * slope;

				Vec3f& vec = xyzMap.at<Vec3f>(pt - topLeftPt);

				if (pt.y > y_hat) {
					vec = 0;
				}
				else {
					aboveWristPointsIJ.push_back(pt);
					aboveWristPointsXYZ.push_back(vec);
				}
			}
		}

		num_points = (int)aboveWristPointsIJ.size();
		points->swap(aboveWristPointsIJ);
		points_xyz->swap(aboveWristPointsXYZ);

		// 重新计算轮廓
		computeContour(xyzMap, points.get(), points_xyz.get(), topLeftPt, num_points);

		/** 寻找主导方向 */
		double contourFar = -1.0; uint contourFarIdx = 0;
		Vec3f contourFarXYZ;
		for (uint i = 0; i < contour.size(); ++i) {
			double norm = util::norm(util::averageAroundPoint(this->xyzMap, this->contour[i] - this->topLeftPt, this->params->xyzAverageSize) -
				this->palmCenterXYZ);
			if (norm > contourFar) {
				contourFar = norm;
				contourFarIdx = i;
			}
		}
		contourFarXYZ = util::averageAroundPoint(this->xyzMap, this->contour[contourFarIdx] - this->topLeftPt, this->params->xyzAverageSize);
		this->dominantDir = util::normalize(contour[contourFarIdx] - this->palmCenterIJ);
		this->dominantDirXYZ = util::normalize(contourFarXYZ - this->palmCenterXYZ);

		/** 使用SVM验证物体是否为手 */

		if (params->handUseSVM && handValidator.isTrained()) {
			this->svmConfidence = handValidator.classify(*this, xyzMap,
				topLeftPt, fullMapSize.width);
			if (this->svmConfidence < params->handSVMConfidenceThresh) {
				// SVM置信值低于阈值，反向决策并销毁手实例
				BOOST_LOG_TRIVIAL(debug) << "SVM判断定不为手";
				return false;
			}
		}

		// 如果物体的面积太大或者太小，则返回错误
		surfaceArea = util::surfaceArea(fullMapSize, *points, *points_xyz, num_points);
		if (surfaceArea < params->handMinArea || surfaceArea > params->handMaxArea) {
			BOOST_LOG_TRIVIAL(debug) << "手部面积不符（" << surfaceArea << "m^2）";
			return false;
		}

		/** 检测手指（指尖） */

		// 使用新轮廓计算凸包
		this->convexHull.clear();
		this->getConvexHull();

		// 计算缺陷
		std::vector<cv::Vec4i> defects;

		if (indexHull.size() > 3)
		{
			// std::vector<int> tmpHull;

			cv::convexityDefects(contour, indexHull, defects);
		}

		// 对所有缺陷按照角度大小排序
		DefectComparer comparer(contour, defects, this->palmCenterIJ);
		std::sort(defects.begin(), defects.end(), comparer);

		// 存储指尖和缺陷的候选（小标索引）
		std::vector<int> fingerTipCands, fingerDefectCands, goodDefects;

		/* 存储上一个缺陷的终点（用于考虑当前起始点作为单独的手指是否足够远） */
		Vec3f lastEnd;

		/* 如果当前缺陷是满足缺陷角度和距离标准的第一个缺陷，则为true */
		bool first = true;

		// 处理缺陷点
		for (int i = 0; i < defects.size(); ++i)
		{
			// 包含有关一个缺陷的所有信息
			cv::Vec4i defect = defects[i];

			// 凸包上缺陷开始的点。作为指尖候选
			Point2i start = contour[defect[0]] - topLeftPt;

			// 凸包上缺陷结束的点。作为指尖候选
			Point2i end = contour[defect[1]] - topLeftPt;

			// 凸包中离缺陷的最远的点
			Point2i farPt = contour[defect[2]] - topLeftPt;

			// 获取XYZ坐标
			start = util::nearestPointOnCluster(xyzMap, start);
			end = util::nearestPointOnCluster(xyzMap, end);
			farPt = util::nearestPointOnCluster(xyzMap, farPt);

			// 如果任何一个点以某种方式超出图像，则跳过
			if (!util::pointInImage(xyzMap, farPt) ||
				!util::pointInImage(xyzMap, start) ||
				!util::pointInImage(xyzMap, end)) continue;

			// 获得点的xyz坐标
			Vec3f far_xyz = util::averageAroundPoint(xyzMap, farPt, params->xyzAverageSize);
			Vec3f start_xyz = util::averageAroundPoint(xyzMap, start, params->xyzAverageSize);
			Vec3f end_xyz = util::averageAroundPoint(xyzMap, end, params->xyzAverageSize);

			// 使用探索法计算一些点之间的距离
			double farCenterDist = util::euclideanDistance(far_xyz, this->palmCenterXYZ);
			double startEndDist = util::euclideanDistance(start_xyz, end_xyz);

			if (farCenterDist > params->defectFarCenterMinDist &&
				farCenterDist < params->defectFarCenterMaxDist &&
				startEndDist > params->defectStartEndMinDist)
			{
				goodDefects.push_back(i);

				// 缺陷起始点与结束点之间的角度
				double angle = util::angleBetweenPoints(start, end, farPt);
				// 如果角度太大则跳过
				if (angle >= params->defectMaxAngle) continue;

				if (!util::pointOnEdge(fullMapSize, start + topLeftPt, params->bottomEdgeThresh, params->sideEdgeThresh) &&
					(first || util::euclideanDistance(lastEnd, start_xyz) > params->defectMinDist)) {
					// 满足角度要求，添加缺陷开始点作为候选
					fingerTipCands.push_back(defect[0]);
					fingerDefectCands.push_back(defect[2]);
					first = false;
				}

				if (!util::pointOnEdge(fullMapSize, end + topLeftPt, params->bottomEdgeThresh, params->sideEdgeThresh)) {
					// 满足角度要求，添加缺陷结束点作为候选
					fingerTipCands.push_back(defect[1]);
					fingerDefectCands.push_back(defect[2]);
				}

				// 保存上一个结束的点
				lastEnd = end_xyz;
			}
		}

		// 从上一轮筛选出来候选点中选择指尖点
		std::vector<Point2i> fingerTipsIj, fingerDefectsIj;
		std::vector<Vec3f> fingerTipsXyz;
		std::vector<int> fingerTipsIdx, fingerDefectsIdx;

		for (unsigned i = 0; i < fingerTipCands.size(); ++i)
		{
			Point2i finger_ij = contour[fingerTipCands[i]] - topLeftPt;
			Point2i defect_ij = contour[fingerDefectCands[i]] - topLeftPt;

			if (defect_ij.y < center.y + params->defectMaxYFromCenter &&
				defect_ij.y + topLeftPt.y < fullMapSize.height - params->bottomEdgeThresh) {

				Vec3f finger_xyz = util::averageAroundPoint(xyzMap, finger_ij, params->xyzAverageSize);
				Vec3f defect_xyz = util::averageAroundPoint(xyzMap, defect_ij, params->xyzAverageSize);

				// 计算用于消除手指候选的一些特征
				float finger_length = util::euclideanDistance(finger_xyz, defect_xyz);
				float centroid_defect_dist = util::euclideanDistance(this->palmCenterXYZ, defect_xyz);
				float finger_defect_slope = (float)(defect_ij.y - finger_ij.y) / abs(defect_ij.x - finger_ij.x);
				float finger_center_slope = (float)(center.y - finger_ij.y) / abs(center.x - finger_ij.x);
				double centroid_defect_finger_angle = util::angleBetweenPoints(finger_ij, center, defect_ij);

				float finger_length_ij = util::euclideanDistance(finger_ij, defect_ij);
				float curve_near = util::contourCurvature(contour, fingerTipCands[i], finger_length_ij * 0.15f);
				float curve_far = util::contourCurvature(contour, fingerTipCands[i], finger_length_ij * 0.45f);

				if (finger_length < params->fingerLenMax && finger_length > params->fingerLenMin &&
					finger_defect_slope > params->fingerDefectSlopeMin &&
					finger_center_slope > params->fingerCenterSlopeMin &&
					centroid_defect_finger_angle > params->centroidDefectFingerAngleMin &&
					finger_xyz[2] != 0 && curve_near >= params->fingerCurveNearMin &&
					curve_far >= params->fingerCurveFarMin)
				{

					fingerTipsXyz.push_back(finger_xyz);
					fingerTipsIj.push_back(finger_ij + topLeftPt);
					fingerDefectsIj.push_back(defect_ij + topLeftPt);
					fingerTipsIdx.push_back(fingerTipCands[i]);
					fingerDefectsIdx.push_back(fingerDefectCands[i]);
				}
			}
		}

		std::vector<int> fingerTipsIdxFiltered, defects_idx_filtered;

		// 将手指和手指靠近边缘阈值化
		for (int i = 0; i < fingerTipsXyz.size(); ++i) {
			double mindist = DBL_MAX;

			for (int j = 0; j < fingerTipsXyz.size(); ++j) {
				if (fingerTipsXyz[i][1] > fingerTipsXyz[j][1] ||
					(fingerTipsXyz[i][1] == fingerTipsXyz[j][1] && i >= j)) continue;

				double dist = util::euclideanDistance(fingerTipsXyz[i], fingerTipsXyz[j]);
				if (dist < mindist) {
					mindist = dist;
					if (mindist < params->fingerDistMin) break;
				}
			}

			// 删除此手指
			if (mindist < params->fingerDistMin) continue;

			// 添加到结果
			this->fingersIJ.push_back(fingerTipsIj[i]);
			this->fingersXYZ.push_back(fingerTipsXyz[i]);
			fingerTipsIdxFiltered.push_back(fingerTipsIdx[i]);

			this->defectsIJ.push_back(fingerDefectsIj[i]);
			Vec3f defXyz = util::averageAroundPoint(xyzMap, fingerDefectsIj[i] - topLeftPt,
				params->xyzAverageSize);
			this->defectsXYZ.push_back(defXyz);
			defects_idx_filtered.push_back(fingerDefectsIdx[i]);
		}

		// 一个或多个可见手指的特殊情况
		if (this->fingersXYZ.size() <= 1)
		{
			this->fingersXYZ.clear();
			this->fingersIJ.clear();
			fingerTipsIdxFiltered.clear();

			// 找到食指的候选
			Point2i indexFinger_ij, indexFinger_right, indexFinger_left;
			int indexFinger_idx;
			double farthest = 0;

			// 在凸包上找到最远的点并记录其左右两侧的点
			if (convexHull.size() > 1)
			{
				for (size_t i = 0; i < convexHull.size(); ++i)
				{
					Point2i convexPt = convexHull[i];

					if (util::pointOnEdge(fullMapSize, convexPt, params->bottomEdgeThresh,
						params->sideEdgeThresh)) continue;

					Vec3f convexPt_xyz = util::averageAroundPoint(xyzMap, convexPt - topLeftPt, 10);

					double dist = util::euclideanDistance(convexPt_xyz, this->palmCenterXYZ);
					double slope = ((double)this->palmCenterIJ.y - (double)convexPt.y) / abs(convexPt.x - this->palmCenterIJ.x);

					if (slope > -0.1 &&
						convexPt.y < fullMapSize.height - 10 && // 消除最低点
						dist > farthest)
					{
						farthest = dist;
						indexFinger_ij = convexPt;
						indexFinger_idx = indexHull[i];
						indexFinger_right = convexHull[(i + 1) % convexHull.size()];
						indexFinger_left = convexHull[(i - 1 + convexHull.size()) % convexHull.size()];
					}
				}
			}

			indexFinger_ij = util::nearestPointOnCluster(xyzMap, indexFinger_ij - topLeftPt, 10000) + topLeftPt;

			Vec3f indexFinger_xyz =
				util::averageAroundPoint(xyzMap, indexFinger_ij - topLeftPt, 10);

			double angle = util::angleBetweenPoints(indexFinger_left, indexFinger_right, indexFinger_ij);

			this->defectsIJ.clear(); this->defectsXYZ.clear(); defects_idx_filtered.clear();

			if (angle <= params->singleFingerAngleThresh ||
				util::pointOnEdge(fullMapSize, indexFinger_ij, params->bottomEdgeThresh,
					params->sideEdgeThresh) || goodDefects.size() == 0) {
				// 角度太小或边缘有点
				BOOST_LOG_TRIVIAL(debug) << "缺陷角度太小或点位于边界";
			}
			else {
				this->fingersXYZ.push_back(indexFinger_xyz);
				this->fingersIJ.push_back(indexFinger_ij);
				fingerTipsIdxFiltered.push_back(indexFinger_idx);

				double best = DBL_MAX;
				Point2i bestDef;
				Vec3f bestXyz;
				int bestIdx = -1;

				for (int j = 0; j < goodDefects.size(); ++j) {
					cv::Vec4i defect = defects[goodDefects[j]];
					Point2i farPt = contour[defect[2]] - topLeftPt;
					Vec3f far_xyz =
						util::averageAroundPoint(xyzMap, farPt, params->xyzAverageSize);

					farPt = util::nearestPointOnCluster(xyzMap, farPt);

					double dist = util::euclideanDistance(far_xyz, indexFinger_xyz);

					if (dist > params->singleFingerLenMin && dist < best) {
						best = dist;
						bestDef = farPt;
						bestXyz = far_xyz;
						bestIdx = defect[2];
					}
				}

				if (best == DBL_MAX) {
					this->defectsIJ.push_back(this->palmCenterIJ);
					this->defectsXYZ.push_back(this->palmCenterXYZ);
					defects_idx_filtered.push_back(-1);
				}
				else {
					this->defectsIJ.push_back(bestDef + topLeftPt);
					this->defectsXYZ.push_back(bestXyz);
					defects_idx_filtered.push_back(bestIdx);
				}

				// 按曲率过滤
				float finger_length_ij =
					util::euclideanDistance(indexFinger_ij, bestDef + topLeftPt);
				float curve_near = util::contourCurvature(contour, indexFinger_idx, finger_length_ij * 0.15f);
				float curve_far = util::contourCurvature(contour, indexFinger_idx, finger_length_ij * 0.45f);

				if (curve_near < params->fingerCurveNearMin ||
					curve_far < params->fingerCurveFarMin) {
					this->fingersIJ.clear(); this->fingersXYZ.clear();
					this->defectsIJ.clear(); this->defectsXYZ.clear();
				}
				else {
					double fingerLen = util::euclideanDistance(indexFinger_xyz, this->defectsXYZ[0]);
					// 手指太长或太短
					if (fingerLen > params->singleFingerLenMax || fingerLen < params->singleFingerLenMin) {
						this->fingersXYZ.clear(); this->fingersIJ.clear();
						this->defectsIJ.clear(); this->defectsXYZ.clear();
						fingerTipsIdxFiltered.clear(); defects_idx_filtered.clear();
					}
				}
			}
		}

		int nFin = (int)this->fingersIJ.size();

		if (nFin == 1 && util::distanOfTwoPointIJ(contour[contourFarIdx], this->fingersIJ.at(0)) > 100)
		{
			Point2i wristCenter((this->wristIJ[0] + this->wristIJ[1]) / 2);
			this->dominantDir = util::normalize(this->palmCenterIJ - wristCenter);
			this->fingersIJ.clear();
			this->fingersXYZ.clear();

			contourFarXYZ = (this->wristXYZ[0] + this->wristXYZ[1]) / 2;
			this->dominantDirXYZ = util::normalize(this->palmCenterXYZ - contourFarXYZ);
		}

		// 根据指尖到掌心的连线于手腕线的夹角从小到大排序 
		if (nFin <= 5)
		{
			//计算各个指尖与掌心连线与手腕线的夹角
			for (int i = 0; i < nFin; i++)
			{
				float angle;
				angle = static_cast<float>(util::getAngelOfTwoVector(wristIJ[0], wristIJ[1], palmCenterIJ, fingersIJ[i]));
				included_angle.push_back(angle);
			}
			//将夹角从小到大排序
			for (size_t i = 0; i < included_angle.size(); i++)
			{
				for (size_t j = 0; j < included_angle.size() - i - 1; j++)
				{
					if (included_angle[j] > included_angle[j + 1])
					{
						std::swap(included_angle[j], included_angle[j + 1]);
					}
				}
			}

			//给指尖及缺陷点坐标排序，指尖与掌心的连线与手腕线的夹角从小到大对应大拇指到小拇指或小拇指到到大拇指（受左右手和手心手背的影响）
			for (size_t i = 0; i < included_angle.size(); i++)
			{
				for (size_t j = 0; j < fingersIJ.size(); j++)
				{
					if (included_angle[i] == util::getAngelOfTwoVector(wristIJ[0], wristIJ[1], palmCenterIJ, fingersIJ[j]))
					{
						fingersIJSort.push_back(fingersIJ[j]);
						fingersXYZSort.push_back(fingersXYZ[j]);
						defectsIJSort.push_back(defectsIJ[j]);
						defectsXYZSort.push_back(defectsXYZ[j]);
						break;
					}
				}
			}
		}

		// 如果手指太少/太多，返回错误。
		if (nFin > 6) {
			BOOST_LOG_TRIVIAL(debug) << "手指数过多 (手指数：" << nFin << ")";
			return false;
		}
		else if (nFin < 1) {
			//BOOST_LOG_TRIVIAL(debug) << "手指数为0";
			Point2i wristCenter((this->wristIJ[0] + this->wristIJ[1]) / 2);
			this->dominantDir = util::normalize(this->palmCenterIJ - wristCenter);

			contourFarXYZ = (this->wristXYZ[0] + this->wristXYZ[1]) / 2;
			this->dominantDirXYZ = util::normalize(this->palmCenterXYZ - contourFarXYZ);
			return true;
		}

		return true;
	}

	void Hand::checkEdgeConnected()
	{
		int cols = fullMapSize.width, rows = fullMapSize.height;

		// 扫描底部
		int row = rows - params->bottomEdgeThresh - topLeftPt.y, col;

		if (row >= 0 && row < xyzMap.rows) {
			for (col = 0; col < std::min(cols / 2 - topLeftPt.x, xyzMap.cols); ++col)
			{
				if (xyzMap.at<Vec3f>(row, col)[2] != 0)
				{
					leftEdgeConnected = true;
					break;
				}
			}
		}

		if (!leftEdgeConnected) {
			// 扫描左侧
			col = params->sideEdgeThresh - topLeftPt.x;
			if (col >= 0 && col < xyzMap.cols) {
				for (row = std::min(rows - 1 - topLeftPt.y, xyzMap.rows - 1);
					row >= std::max(rows * params->handEdgeConnectMaxY - topLeftPt.y, 0.0); --row)
				{
					if (xyzMap.at<Vec3f>(row, col)[2] != 0)
					{
						leftEdgeConnected = true;
						break;
					}
				}
			}
		}

		// 扫描底部
		row = rows - params->bottomEdgeThresh - topLeftPt.y;
		if (row >= 0 && row < xyzMap.rows) {
			for (col = cols / 2 - topLeftPt.x; col < cols - topLeftPt.x; ++col)
			{
				if (col < 0 || col >= xyzMap.cols) continue;
				if (xyzMap.at<Vec3f>(row, col)[2] != 0)
				{
					rightEdgeConnected = true;
					break;
				}
			}
		}

		if (!rightEdgeConnected) {
			// 扫描右侧
			col = cols - params->sideEdgeThresh - topLeftPt.x;
			if (col >= 0 && col < xyzMap.cols) {
				for (row = std::min(rows - 1 - topLeftPt.y, xyzMap.rows - 1);
					row >= std::max(rows * params->handEdgeConnectMaxY - topLeftPt.y, 0.0); --row)
				{
					if (row < 0 || row >= xyzMap.rows) continue;
					if (xyzMap.at<Vec3f>(row, col)[2] != 0)
					{
						rightEdgeConnected = true;
						break;
					}
				}
			}
		}

		/*if (rightEdgeConnected) {
			BOOST_LOG_TRIVIAL(debug) << "接触到右边框";
		}
		else if (leftEdgeConnected)
		{
			BOOST_LOG_TRIVIAL(debug) << "接触到左边框";
		}*/

	}

	Vec3f Hand::getPalmCenter() const
	{
		return this->palmCenterXYZ;
	}

	Point2i Hand::getPalmCenterIJ() const
	{
		return this->palmCenterIJ;
	}

	std::vector<Vec3f> Hand::getFingers() const
	{
		return this->fingersXYZ;
	}

	std::vector<Point2i> Hand::getFingersIJ() const
	{
		return this->fingersIJ;
	}

	std::vector<Vec3f> Hand::getDefects() const
	{
		return this->defectsXYZ;
	}

	std::vector<Point2i> Hand::getDefectsIJ() const
	{
		return this->defectsIJ;
	}

	std::vector<Vec3f> Hand::getWrist() const
	{
		return this->wristXYZ;
	}

	std::vector<Point2i> Hand::getWristIJ() const
	{
		return this->wristIJ;
	}

	Vec3f Hand::getDominantDirectionXYZ() const {
		return this->dominantDirXYZ;
	}

	double Hand::getCircleRadius() const
	{
		return this->circleRadius;
	}

	Point2f Hand::getDominantDirection() const
	{
		return this->dominantDir;
	}

	double Hand::getSVMConfidence() const
	{
		return this->svmConfidence;
	}

	bool Hand::isValidHand() const
	{
		return this->isHand;
	}

	bool Hand::touchingEdge() const
	{
		return this->leftEdgeConnected || rightEdgeConnected;
	}

	bool Hand::touchingLeftEdge() const
	{
		return this->leftEdgeConnected;
	}

	bool Hand::touchingRightEdge() const
	{
		return this->rightEdgeConnected;
	}

	/** 打标签用接口 **/

	std::vector<Vec3f> Hand::getFingersSort() const
	{
		return this->fingersXYZSort;
	}

	std::vector<Point2i> Hand::getFingersIJSort() const
	{
		return this->fingersIJSort;
	}

	std::vector<Vec3f> Hand::getDefectsSort() const
	{
		return this->defectsXYZSort;
	}

	std::vector<float> Hand::getinclude() const
	{
		return included_angle;
	}

	std::vector<Point2i> Hand::getDefectsIJSort() const
	{
		return this->defectsIJSort;
	}

	Vec3f Hand::getwristmid() const
	{
		return this->wristmidXYZ;
	}

	Point2i Hand::getarmmidIJ() const
	{
		return this->armmid;
	}

	Point2i Hand::getwristmidIJ() const
	{
		return this->wristmidIJ;
	}

	Point2i Hand::getcontactL_ij() const
	{
		return this->contactL_ij;
	}

	Point2i Hand::getcontactR_ij() const
	{
		return this->contactR_ij;
	}
}


