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
#include "Hand.h"

namespace ht {

    // 手指标签枚举
    enum FingerLabel {
        L_THUMB,        //拇指（左）
        L_FOREFINGER,    //食指（左）
        L_MIDFINGER,    //中指（左）
        L_RINGFINGER,    //无名指（左）
        L_LITTERFINGER,    //小指（左）

        R_THUMB,        //拇指（右）
        R_FOREFINGER,    //食指（右）
        R_MIDFINGER,    //中指（右）
        R_RINGFINGER,    //无名指（右）
        R_LITTERFINGER    //小指（右）
    };

    // 手部标签枚举
    enum HandLabel {
        L_HAND,        //左手
        R_HAND        //右手
    };

    // 掌心标签枚举
    enum CenterLabel
    {
        L_CENTER,    //左手手心
        R_CENTER    //右手手心
    };


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