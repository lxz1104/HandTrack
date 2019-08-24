/***************************************************************************
 *@File:label.h
 *@Author:YQ
 *@Date:2019-08-24
 *@Description����ͷ�ļ�ʵ�������ȱ�ݵ�ĸ������ض����Ƶ���ָ�⣬������ʶ�������ֱ����б�ǩ
 *@History:
 ***************************************************************************/

#pragma once
#ifndef HAND_LABEL_LABEL_H
#define HAND_LABEL_LABEL_H

#include "stdafx.h"
#include "Util.h"
#include "Hand.h"

namespace ht {
	/**
	 * ���ȱ�ݵ�ĸ��������ض����ƽ��б�ǩ��
	 */
	class Label
	{
	public:

		/**
		 * ����ָָ����ǩ
		 */
		static void labelFingers(const cv::Mat& background, cv::Mat& output, Hand* hand);


	};
}

#endif //HAND_LABEL_LABEL_H
