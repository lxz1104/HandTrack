#include "Label.h"
#include "label.h"
#include "Util.h"

namespace ht {
	/**
	 * 针对缺陷点的个数对特定手势的手指尖，左右手识别，手心手背进行标签
	 */
	void Label::labelFingers(const cv::Mat& background, cv::Mat& output, Hand* hand)
	{
		Point2i center = hand->getPalmCenterIJ();
		Vec3f centerXYZ = hand->getPalmCenter();
		const std::vector<Vec3f> fingersXYZSort = hand->getFingersSort();
		const std::vector<Point2i> fingersIJSort = hand->getFingersIJSort();
		const std::vector<Vec3f> defectsXYZSort = hand->getDefectsSort();
		const std::vector<Point2i> defectsIJSort = hand->getDefectsIJSort();
		const Vec3f wristmidXYZ = hand->getwristmid();
		const Point2i wristmidIJ = hand->getwristmidIJ();
		const Point2i armmidpoint = hand->getarmmidIJ();
		std::string fingersname[5] = { "TF", "IF", "MF", "RF", "LF" };

		//通过手臂中点与手腕中点连线与像素坐标系X轴的夹角的余弦值判断左右手
		Point2i Armorientation = wristmidIJ - armmidpoint;
		Point2i xaxis = Point2i(1, 0);
		float unitWid = std::min((float)background.cols / 640, 1.75f);
		Point2i box = Point2i(static_cast<int>(unitWid * 6), static_cast<int>(unitWid * 6));
		double cosangle = Armorientation.dot(xaxis) / (sqrt(Armorientation.x * Armorientation.x + Armorientation.y * Armorientation.y) * sqrt(xaxis.x * xaxis.x + xaxis.y * xaxis.y));

		cv::rectangle(output, armmidpoint - box, armmidpoint + box,
			cv::Scalar(255, 255, 0), static_cast<int>(unitWid * 1.5));
		cv::line(output, armmidpoint, wristmidIJ, cv::Scalar(255, 0, 200), static_cast<int>(roundf(unitWid * 2)));

		//五个手指全部伸开，，可区分手心手背
		if (hand->getFingersIJSort().size() == 5 && hand->getDefectsIJSort().size() == 5)
		{
			double dist0 = util::euclideanDistance(centerXYZ, defectsXYZSort[0]);
			double dist1 = util::euclideanDistance(centerXYZ, defectsXYZSort[defectsXYZSort.size() - 1]);
			//std::cout<< "角度的余弦值：" << incldedangle << std::endl;
			//左手
			if (cosangle > 0)
			{
				//手背
				if (dist0 < dist1)
				{
					cv::putText(output, "L_HB", center, cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 255));
					for (size_t i = 0; i < fingersIJSort.size(); i++)
					{
						cv::putText(output, fingersname[i], fingersIJSort[i], cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 255));
					}
				}
				//手心
				if (dist0 > dist1)
				{
					cv::putText(output, "L_HP", center, cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 255));
					for (size_t i = 0; i < fingersIJSort.size(); i++)
					{
						cv::putText(output, fingersname[i], fingersIJSort[fingersIJSort.size() - 1 - i], cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 255));
					}
				}
			}
			//右手
			if (cosangle < 0)
			{
				//手心
				if (dist0 < dist1)
				{
					cv::putText(output, "R_HP", center, cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 255));
					for (size_t i = 0; i < fingersIJSort.size(); i++)
					{
						cv::putText(output, fingersname[i], fingersIJSort[i], cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 255));
					}
				}
				//手背
				if (dist0 > dist1)
				{
					cv::putText(output, "R_HB", center, cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 255));
					for (size_t i = 0; i < fingersIJSort.size(); i++)
					{
						cv::putText(output, fingersname[i], fingersIJSort[fingersIJSort.size() - 1 - i], cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 255));
					}
				}
			}
		}

		//伸出四个手指（针对特定手势：从食指到小拇指，不包括大拇指），可区分手心手背
		if (fingersXYZSort.size() == 4 && defectsXYZSort.size() == 4)
		{
			double dist0 = util::euclideanDistance(centerXYZ, fingersXYZSort[0]);
			double dist1 = util::euclideanDistance(centerXYZ, fingersXYZSort[3]);

			//左手
			if (cosangle > 0)
			{
				//手背
				if (dist0 > dist1)
				{
					cv::putText(output, "L_HB", center, cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 255));
					for (size_t i = 0; i < fingersIJSort.size(); i++)
					{
						cv::putText(output, fingersname[i + 1], fingersIJSort[i], cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 255));
					}
				}
				//手心
				if (dist0 < dist1)
				{
					cv::putText(output, "L_HP", center, cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 255));
					for (size_t i = 0; i < fingersIJSort.size(); i++)
					{
						cv::putText(output, fingersname[i + 1], fingersIJSort[fingersIJSort.size() - 1 - i], cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 255));
					}
				}
			}
			//右手
			if (cosangle < 0)
			{
				//手心
				if (dist0 > dist1)
				{
					cv::putText(output, "R_HP", center, cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 255));
					for (size_t i = 0; i < fingersIJSort.size(); i++)
					{
						cv::putText(output, fingersname[i + 1], fingersIJSort[i], cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 255));
					}
				}
				//手背
				if (dist0 < dist1)
				{
					cv::putText(output, "R_HB", center, cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 255));
					for (size_t i = 0; i < fingersIJSort.size(); i++)
					{
						cv::putText(output, fingersname[i + 1], fingersIJSort[fingersIJSort.size() - 1 - i], cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 255));
					}
				}
			}
		}

		//伸出三个手指（针对特定手势：中指到小拇指，不包括食指和大拇指），可区分手心手背
		if (fingersXYZSort.size() == 3 && defectsXYZSort.size() == 3)
		{
			double dist0 = util::euclideanDistance(wristmidXYZ, fingersXYZSort[0]);
			double dist1 = util::euclideanDistance(wristmidXYZ, fingersXYZSort[2]);
			//左手
			if (cosangle > 0)
			{
				//手背
				if (dist0 > dist1)
				{
					cv::putText(output, "L_HB", center, cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 255));
					for (size_t i = 0; i < fingersIJSort.size(); i++)
					{
						cv::putText(output, fingersname[i + 2], fingersIJSort[i], cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 255));
					}
				}
				//手心
				if (dist0 < dist1)
				{
					cv::putText(output, "L_HP", center, cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 255));
					for (size_t i = 0; i < fingersIJSort.size(); i++)
					{
						cv::putText(output, fingersname[i + 2], fingersIJSort[fingersIJSort.size() - 1 - i], cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 255));
					}
				}
			}
			//右手
			if (cosangle < 0)
			{
				//手背
				if (dist0 < dist1)
				{
					cv::putText(output, "R_HB", center, cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 255));
					for (size_t i = 0; i < fingersIJSort.size(); i++)
					{
						cv::putText(output, fingersname[i + 2], fingersIJSort[fingersIJSort.size() - 1 - i], cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 255));
					}
				}
				//手心
				if (dist0 > dist1)
				{
					cv::putText(output, "R_HP", center, cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 255));
					for (size_t i = 0; i < fingersIJSort.size(); i++)
					{
						cv::putText(output, fingersname[i + 2], fingersIJSort[i], cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 255));
					}
				}
			}
		}

		//伸出两个手指（针对特定手势：捏合动作（大拇指和食指），YES动作（食指和中指）），YES动作可区分手心手背，捏合动作值区分左右手
		if (fingersXYZSort.size() == 2 && defectsXYZSort.size() == 2)
		{
			double dist0 = util::euclideanDistance(defectsXYZSort[1], fingersXYZSort[0]);
			double dist1 = util::euclideanDistance(defectsXYZSort[1], fingersXYZSort[1]);
			double dist2 = util::euclideanDistance(centerXYZ, fingersXYZSort[0]);
			double dist3 = util::euclideanDistance(centerXYZ, fingersXYZSort[1]);
			//左手
			if (cosangle > 0)
			{
				//捏合动作
				if (dist2 < 0.105)
				{
					cv::putText(output, "LH", center, cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 255));
					for (size_t i = 0; i < fingersIJSort.size(); i++)
					{
						cv::putText(output, fingersname[i], fingersIJSort[i], cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 255));
					}
				}
				else
				{
					//Yes动作
					//手背
					if (dist0 < dist1)
					{
						cv::putText(output, "L_HB", center, cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 255));
						for (size_t i = 0; i < fingersIJSort.size(); i++)
						{
							cv::putText(output, fingersname[i + 1], fingersIJSort[i], cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 255));
						}
					}
					//手心
					if (dist0 > dist1)
					{
						cv::putText(output, "L_HP", center, cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 255));
						for (size_t i = 0; i < fingersIJSort.size(); i++)
						{
							cv::putText(output, fingersname[i + 1], fingersIJSort[fingersIJSort.size() - 1 - i], cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 255));
						}
					}
				}
			}
			//右手
			if (cosangle < 0)
			{
				//捏合动作
				if (dist3 < 0.105)
				{
					cv::putText(output, "RH", center, cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 255));
					for (size_t i = 0; i < fingersIJSort.size(); i++)
					{
						cv::putText(output, fingersname[i], fingersIJSort[fingersIJSort.size() - 1 - i], cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 255));
					}
				}
				else
				{
					//Yes动作
					//手背
					if (dist0 > dist1)
					{
						cv::putText(output, "R_HB", center, cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 255));
						for (size_t i = 0; i < fingersIJSort.size(); i++)
						{
							cv::putText(output, fingersname[i + 1], fingersIJSort[fingersIJSort.size() - 1 - i], cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 255));
						}
					}
					//手心
					if (dist0 < dist1)
					{
						cv::putText(output, "R_HP", center, cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 255));
						for (size_t i = 0; i < fingersIJSort.size(); i++)
						{
							cv::putText(output, fingersname[i + 1], fingersIJSort[i], cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 255));
						}
					}
				}
			}
		}

		//一个手指（针对特定手势：食指点击手势），只区分左右手
		if (fingersXYZSort.size() == 1 && defectsXYZSort.size() == 1)
		{
			//左手
			if (cosangle > 0)
			{
				cv::putText(output, "LH", center, cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 255));
				cv::putText(output, "IF", fingersIJSort[0], cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 255));
			}
			//右手
			if (cosangle < 0)
			{
				cv::putText(output, "RH", center, cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 255));
				cv::putText(output, "IF", fingersIJSort[0], cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 255));
			}
		}



	}




}