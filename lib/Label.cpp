#include "Label.h"
#include "Util.h"

namespace ht {
	/**
	 * ���ȱ�ݵ�ĸ������ض����Ƶ���ָ�⣬������ʶ�������ֱ����б�ǩ
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

		//ͨ���ֱ��е��������е���������������ϵX��ļнǵ�����ֵ�ж�������
		Point2i Armorientation = wristmidIJ - armmidpoint;
		Point2i xaxis = Point2i(1, 0);
		float unitWid = std::min((float)background.cols / 640, 1.75f);
		Point2i box = Point2i(static_cast<int>(unitWid * 6), static_cast<int>(unitWid * 6));
		double cosangle = Armorientation.dot(xaxis) / (sqrt(Armorientation.x * Armorientation.x + Armorientation.y * Armorientation.y) * sqrt(xaxis.x * xaxis.x + xaxis.y * xaxis.y));

		cv::rectangle(output, armmidpoint - box, armmidpoint + box,
			cv::Scalar(255, 255, 0), static_cast<int>(unitWid * 1.5));
		cv::line(output, armmidpoint, wristmidIJ, cv::Scalar(255, 0, 200), static_cast<int>(roundf(unitWid * 2)));

		//�����ָȫ���쿪���������������ֱ�
		if (hand->getFingersIJSort().size() == 5 && hand->getDefectsIJSort().size() == 5)
		{
			double dist0 = util::euclideanDistance(centerXYZ, defectsXYZSort[0]);
			double dist1 = util::euclideanDistance(centerXYZ, defectsXYZSort[defectsXYZSort.size() - 1]);
			//std::cout<< "�Ƕȵ�����ֵ��" << incldedangle << std::endl;
			//����
			if (cosangle > 0)
			{
				//�ֱ�
				if (dist0 < dist1)
				{
					cv::putText(output, "L_HB", center, cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 255));
					for (size_t i = 0; i < fingersIJSort.size(); i++)
					{
						cv::putText(output, fingersname[i], fingersIJSort[i], cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 255));
					}
				}
				//����
				if (dist0 > dist1)
				{
					cv::putText(output, "L_HP", center, cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 255));
					for (size_t i = 0; i < fingersIJSort.size(); i++)
					{
						cv::putText(output, fingersname[i], fingersIJSort[fingersIJSort.size() - 1 - i], cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 255));
					}
				}
			}
			//����
			if (cosangle < 0)
			{
				//����
				if (dist0 < dist1)
				{
					cv::putText(output, "R_HP", center, cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 255));
					for (size_t i = 0; i < fingersIJSort.size(); i++)
					{
						cv::putText(output, fingersname[i], fingersIJSort[i], cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 255));
					}
				}
				//�ֱ�
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

		//����ĸ���ָ������ض����ƣ���ʳָ��СĴָ����������Ĵָ���������������ֱ�
		if (fingersXYZSort.size() == 4 && defectsXYZSort.size() == 4)
		{
			double dist0 = util::euclideanDistance(centerXYZ, fingersXYZSort[0]);
			double dist1 = util::euclideanDistance(centerXYZ, fingersXYZSort[3]);

			//����
			if (cosangle > 0)
			{
				//�ֱ�
				if (dist0 > dist1)
				{
					cv::putText(output, "L_HB", center, cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 255));
					for (size_t i = 0; i < fingersIJSort.size(); i++)
					{
						cv::putText(output, fingersname[i + 1], fingersIJSort[i], cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 255));
					}
				}
				//����
				if (dist0 < dist1)
				{
					cv::putText(output, "L_HP", center, cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 255));
					for (size_t i = 0; i < fingersIJSort.size(); i++)
					{
						cv::putText(output, fingersname[i + 1], fingersIJSort[fingersIJSort.size() - 1 - i], cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 255));
					}
				}
			}
			//����
			if (cosangle < 0)
			{
				//����
				if (dist0 > dist1)
				{
					cv::putText(output, "R_HP", center, cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 255));
					for (size_t i = 0; i < fingersIJSort.size(); i++)
					{
						cv::putText(output, fingersname[i + 1], fingersIJSort[i], cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 255));
					}
				}
				//�ֱ�
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

		//���������ָ������ض����ƣ���ָ��СĴָ��������ʳָ�ʹ�Ĵָ���������������ֱ�
		if (fingersXYZSort.size() == 3 && defectsXYZSort.size() == 3)
		{
			double dist0 = util::euclideanDistance(wristmidXYZ, fingersXYZSort[0]);
			double dist1 = util::euclideanDistance(wristmidXYZ, fingersXYZSort[2]);
			//����
			if (cosangle > 0)
			{
				//�ֱ�
				if (dist0 > dist1)
				{
					cv::putText(output, "L_HB", center, cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 255));
					for (size_t i = 0; i < fingersIJSort.size(); i++)
					{
						cv::putText(output, fingersname[i + 2], fingersIJSort[i], cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 255));
					}
				}
				//����
				if (dist0 < dist1)
				{
					cv::putText(output, "L_HP", center, cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 255));
					for (size_t i = 0; i < fingersIJSort.size(); i++)
					{
						cv::putText(output, fingersname[i + 2], fingersIJSort[fingersIJSort.size() - 1 - i], cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 255));
					}
				}
			}
			//����
			if (cosangle < 0)
			{
				//�ֱ�
				if (dist0 < dist1)
				{
					cv::putText(output, "R_HB", center, cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 255));
					for (size_t i = 0; i < fingersIJSort.size(); i++)
					{
						cv::putText(output, fingersname[i + 2], fingersIJSort[fingersIJSort.size() - 1 - i], cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 255));
					}
				}
				//����
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

		//���������ָ������ض����ƣ���϶�������Ĵָ��ʳָ����YES������ʳָ����ָ������YES���������������ֱ�����϶���ֵ����������
		if (fingersXYZSort.size() == 2 && defectsXYZSort.size() == 2)
		{
			double dist0 = util::euclideanDistance(defectsXYZSort[1], fingersXYZSort[0]);
			double dist1 = util::euclideanDistance(defectsXYZSort[1], fingersXYZSort[1]);
			double dist2 = util::euclideanDistance(centerXYZ, fingersXYZSort[0]);
			double dist3 = util::euclideanDistance(centerXYZ, fingersXYZSort[1]);
			//����
			if (cosangle > 0)
			{
				//��϶���
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
					//Yes����
					//�ֱ�
					if (dist0 < dist1)
					{
						cv::putText(output, "L_HB", center, cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 255));
						for (size_t i = 0; i < fingersIJSort.size(); i++)
						{
							cv::putText(output, fingersname[i + 1], fingersIJSort[i], cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 255));
						}
					}
					//����
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
			//����
			if (cosangle < 0)
			{
				//��϶���
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
					//Yes����
					//�ֱ�
					if (dist0 > dist1)
					{
						cv::putText(output, "R_HB", center, cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 255));
						for (size_t i = 0; i < fingersIJSort.size(); i++)
						{
							cv::putText(output, fingersname[i + 1], fingersIJSort[fingersIJSort.size() - 1 - i], cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 255));
						}
					}
					//����
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

		//һ����ָ������ض����ƣ�ʳָ������ƣ���ֻ����������
		if (fingersXYZSort.size() == 1 && defectsXYZSort.size() == 1)
		{
			//����
			if (cosangle > 0)
			{
				cv::putText(output, "LH", center, cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 255));
				cv::putText(output, "IF", fingersIJSort[0], cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 255));
			}
			//����
			if (cosangle < 0)
			{
				cv::putText(output, "RH", center, cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 255));
				cv::putText(output, "IF", fingersIJSort[0], cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 255));
			}
		}



	}




}