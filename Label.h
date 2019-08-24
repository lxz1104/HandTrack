/***************************************************************************
 *@File:label.h
 *@Author:YQ
 *@Date:2019-08-24
 *@Description：该头文件实现了针对缺陷点的个数对特定手势的手指尖，左右手识别，手心手背进行标签
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
	 * 针对缺陷点的个数，对特定手势进行标签化
	 */
	class Label
	{
	public:

		/**
		 * 对手指指尖打标签
		 */
		static void labelFingers(const cv::Mat& background, cv::Mat& output, Hand* hand);


	};
}

#endif //HAND_LABEL_LABEL_H
