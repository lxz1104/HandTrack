/***************************************************************************
 *@File:HandClassifier.h
 *@Author:YQ
 *@Date:2019-07-26
 *@Description：该头文件主要实现了对分类器Classifier，SVM手分类器SVMHandClassifier
 *              基于SVM验证器SVMHandValidator三个类定义，对相关函数进行了声明
 *@History:
 ***************************************************************************/

#pragma once
#ifndef HAND_CLASSIFIER_H
#define HAND_CLASSIFIER_H

#include "Hand.h"
#include <opencv2/core.hpp>
#include <opencv2/ml.hpp>
#include <string>
#include <vector>
#include <exception>


namespace ht {
    namespace classifier {

        class ClassifierNotTrainedException : public std::exception {};
        class InvalidFeatureVectorException : public std::exception {};

		/**
		 * 包含数据标签的文件名（路径）
		 */
        const std::string DATA_LABELS_FILE_NAME = "labels.txt";

		/**
		 * 包含特征点信息的文件名（路径）
		 */
        const std::string DATA_FEATURES_FILE_NAME = "handfeatures.csv";

		/**
		 * 手和手指分类器的抽象基类
		 * @type:Abstract
		 */
        class Classifier {
        public:
			/**
			 * 从文件导入分类器模型
			 * @param  path 分类器模型的路径
			 * @return bool 成功时为真，错误时为假。
			 * 包含数据标签的文件名（data dir的路径）
			 */
            virtual bool loadFile(const std::string & path) = 0;

			/**
			 * 将分类器模型导出到文件。
			 * @param  path 分类器导出到的路径
			 * @return bool 成功时为真，错误时为假。
			 */
            virtual bool exportFile(const std::string & path) const = 0;

			/**
			 * 开始使用指定路径中的数据训练此分类器。
			 * @param  dataPath    培训数据目录的数据路径
			 * @param  hyperparams 超参数数组
			 * @return bool        成功时为真，错误时为假。如果已经训练过，则返回真。
			 */
            virtual bool train(const std::string & dataPath, const double hyperparams[]) = 0;

			/**
			 * @return bool 如果此分类器已完成训练，则返回真
			 */
            virtual bool isTrained() const;

			/**
			 * 使用此分类器对表示对象的特征向量进行分类。
			 * 如果分类器还没有被训练，throws ClassifierNotTrainedException.
			 * @param [in] features 特征向量 (CV_32F; 1xN)
			 * @return float        指示对象所属类别的双精度值
			 */
            virtual double classify(const cv::Mat & features) const = 0;

        protected:
			/**
			 * 如果分类器经过培训，则为真。
			 */
            bool trained = false;

			/** 计算均值和方差的辅助函数 */
            static void computeMeanAndVariance(const std::vector<ht::Vec3f> & points, cv::Vec3f center,
                double & avg_dist, double & var_dist, double & avg_depth, double & var_depth);
        };

		/**
		 * (OpenCV) 基于支持向量机的手分类器
		 */
        class SVMHandClassifier : public Classifier {
        public:
			/**
			 * 使用的SVM数量
			 * 每个可见手指数使用1个SVM，即1个用于带1个可见手指的手，
			 * 一只手，有两个可见的手指等。
			 */
            static const int NUM_SVMS = 4;

			/**
			* 默认支持向量机的超参数
			*/
            static const double DEFAULT_HYPERPARAMS[5 * NUM_SVMS];

			/**
			 * 创建新的未经训练的SVM手分类器
			 */
            SVMHandClassifier();

			/**
			 *  通过从磁盘加载分类器模型，构造一个SVM手分类器
			 *  @param 包含模型文件的目录路径
			 */
            explicit SVMHandClassifier(const std::string & path);

			/**
			 * 销毁此SVM手动分类器
			 */
            ~SVMHandClassifier();

			/**
			 * 从磁盘加载SVM模型
			 * @param  path 从中加载模型的路径目录。
			 * @return bool 成功时为真，错误时为假。
			 */
            bool loadFile(const std::string & path) override;

			/**
			 * 将SVM模型写入磁盘。
			 * @param  path 将模型导出到的路径目录.
			 * @return bool 成功时为真，错误时为假。
			 */
            bool exportFile(const std::string & path) const override;

			/**
			 * 开始使用指定路径中的数据训练此分类器。
			 * @param dataPath    训练数据目录的路径
			 * @param hyperparams 超参数数组
			 * @return bool       成功时为真，错误时为假。如果已经训练过，则返回真。
			 */
            virtual bool train(const std::string & dataPath,
                const double hyperparams[5 * NUM_SVMS] = DEFAULT_HYPERPARAMS) override;

			/**
			 *  使用此分类器对表示手对象的特征向量进行分类。
			 *  返回介于0和1之间的双精度值。
			 *  “1”表示我们完全相信物体是一只手，反之亦然。
			 *  如果分类器还没有被训练, throws ClassifierNotTrainedException.
			 *  @param [in] features 特征向量 (索引0应为手指数) (CV_32F; 1 x N)
			 *  @return float        确信对象是一只手（0到1之间的双精度值）
			 */
            double classify(const cv::Mat & features) const override;

			/**
			 *  使用此分类器对手实例进行分类。自动从这只手上提取特征。
			 *  “1”表示我们完全相信物体是一只手，反之亦然。
			 *  如果分类器还没有被训练, throws ClassifierNotTrainedException.
			 *  @param [in] hand      手的实例
			 *  @param [in] depth_map 深度图(注: 必须是 CV_32FC3)
			 *  @param [in] top_left  左上角（可选），深度图中表示的左上角点（要转换的X、Y坐标）
			 *  @param [in] full_wid  全宽可选，全深度地图的大小。默认情况下，使用深度宽度图
			 *  @return     float     确信对象是一只手（0到1之间的双精度值）
			 */
			double classify(ht::Hand & hand,
                const cv::Mat & depth_map, cv::Point top_left = cv::Point(0, 0), int full_wid = -1) const;

			/**
			* 得到该分类器将用于某个特征向量的支持向量机索引。
			* 用于检查哪些SVM工作正常，哪些不正常
			* 注: 每个可见手指数使用1个SVM，即1个用于带1个可见手指的手，1个用于带2个可见手指的手等。
			* @param [in]  features 特征向量 (索引0应为手指数)
			* @return  int 使用的SVM索引
			*/
            static int getSVMIdx(const cv::Mat & features);

			/**
			* 获取此分类器将用于给定手指数的SVM索引
			* 用于检查哪些SVM工作正常，哪些不正常
			* Note:   每个可见手指数使用1个SVM，即1个用于带1个可见手指的手，1个用于带2个可见手指的手等。
			* @param [in] num_fingers 手指数 (索引0应为手指数)
			* @return int 使用的SVM索引
			*/
            static int getSVMIdx(int num_fingers);

			/** 从给定的手对象和深度图中提取手的特定特征
			 *  @param [in] hand      手的实例
			 *  @param [in] depth_map 深度图 (注: 必须是 CV_32FC3)
			 *  @param [in] top_left  左上角（可选），深度图中表示的左上角点（要转换的X、Y坐标）
			 *  @param [in] full_wid  全宽可选，全深度地图的大小。默认情况下，使用深度宽度图
			 *  @return  Mat  特征向量 (类型： CV_32F, 1xN)
			 */
            static cv::Mat extractFeatures(ht::Hand & hand, const cv::Mat & depth_map,
                cv::Point top_left = cv::Point(0, 0), int full_wid = -1);

        private:
			/**
			 * 功能的最大数量。任何附加功能都将被切断。
			 */
            static const int MAX_FEATURES = 57;

			// 存储SVM 
            cv::Ptr<cv::ml::SVM> svm[NUM_SVMS];

			/**
			* 初始化分类器的辅助函数
			*/
            void initSVMs(const double hyperparams[5 * NUM_SVMS] = DEFAULT_HYPERPARAMS);
        };

		/**
		 * 基于支持向量机的验证器
		 */
        class SVMHandValidator : public Classifier {
        public:
			/**
			* 默认支持向量机的超参数
			*/
            static const double DEFAULT_HYPERPARAMS[5];

			/**
			 * 创建新的未经训练的SVM手分类器
			 */
            SVMHandValidator();

			/** 通过从磁盘加载分类器模型，构造一个SVM手部分类器。
             *  @param 模型文件目录的路径
             */
            explicit SVMHandValidator(const std::string & path);

			/**
			 * 销毁这个SVM手分类器
			 */
            ~SVMHandValidator();

			/**
			 * 从磁盘加载SVM模型
			 * @param  [in]  加载模型的目录。
			 * @return bool 成功时为真，错误时为假。
			 */
            bool loadFile(const std::string & path) override;

			/**
			 * 将SVM模型写入磁盘。
			 * @param  [in] path 要将模型导出到的目录。
			 * @return bool 成功时为真，错误时为假。
			 */
            bool exportFile(const std::string & path) const override;

			/**
			 * 开始使用指定路径中的数据训练此分类器。
			 * @param [in] dataPath    训练数据目录的路径
			 * @param [in] hyperparams 超参数数组
			 * @return bool 成功时为真，错误时为假，如果已经被训练则返回真。
			 */
            virtual bool train(const std::string & dataPath,
                const double hyperparams[5] = DEFAULT_HYPERPARAMS) override;

			/**
			 *  使用此分类器验证表示手对象的特征向量。
			 *  返回介于0和1之间的双精度值。
			 *  “1”表示我们完全相信物体是一只手，反之亦然。
			 *  如果分类器还没有经过训练, throws ClassifierNotTrainedException.
			 *  @param [in] features 特征向量 (CV_32F; 1 x N)
			 *  @return float 物体是一只手的置信度 (介于0和1之间的双精度值)
			 */
            double classify(const cv::Mat & features) const override;

			/**
			 *  使用此分类器验证手实例。自动从这只手上提取特征。
			 *  “1”表示我们完全相信物体是一只手，反之亦然。
			 *  如果分类器还没有经过训练, throws ClassifierNotTrainedException.
			 *  @param [in] hand      手的实例
			 *  @param [in] depth_map 深度图像 (注: 必须是 CV_32FC3)
			 *  @param [in] top_left  左上角（可选），深度图中表示的左上角点（要转换的X、Y坐标）
			 *  @param [in] full_wid  全宽可选，全深度地图的大小。默认情况下，使用深度宽度图
			 *  @param [in] num_features 要提取的圆特征数
			 *  @param [in] avg_size  从ij坐标中获取xyz坐标时，xyz地图上每个点周围的正方形的大小的平均值
			 *  @return float 物体是一只手的置信度 (介于0和1之间的双精度值)
			 */
            double classify(ht::Hand & hand,
                const cv::Mat & depth_map, cv::Point top_left = cv::Point(0, 0), int full_wid = -1, 
                int num_features = 48, int avg_size = 5) const;

			/** 从给定的手实例和深度图中提取特征，以便与svmhandvalidator一起使用
			 *  @param [in] hand      手的实例
			 *  @param [in] depth_map 深度图像(注: 必须是CV_32FC3)
			 *  @param [in] top_left  左上角（可选），深度图中表示的左上角点（要转换的X、Y坐标）
			 *  @param [in] full_wid  全宽可选，全深度地图的大小。默认情况下，使用深度宽度图
			 *  @param [in] num_features 要提取的特征数
			 *  @param [in] avg_size  从ij坐标中获取xyz坐标时，xyz地图上每个点周围的正方形的大小的平均值
			 *  @return Mat 特征向量 (type CV_32F, 1xN)
			 */
            static cv::Mat extractFeatures(ht::Hand & hand,
                const cv::Mat & depth_map, cv::Point top_left = cv::Point(0, 0), int full_wid = -1,
                int num_features = 48, int avg_size = 5);

        private:

			//  SVM容器
            cv::Ptr<cv::ml::SVM> svm;

			/**
			* 初始化分类器的帮助函数
			*/
            void initSVMs(const double hyperparams[5] = DEFAULT_HYPERPARAMS);
        };
    }
}

#endif //HAND_CLASSIFIER_H