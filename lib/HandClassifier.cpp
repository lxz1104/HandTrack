#include "stdafx.h"
#include "Hand.h"
#include "Util.h"
#include "HandClassifier.h"

namespace ht {
    namespace classifier {
        // 分类器实现部分
        bool Classifier::isTrained() const {
            return trained;
        }

        void Classifier::computeMeanAndVariance(const std::vector<ht::Vec3f> & points,
            cv::Vec3f center, double& avg_dist, double& var_dist, double& avg_depth, double& var_depth) {

            avg_dist = avg_depth = 0;
            int totalpts = 0;

            for (const ht::Vec3f pt : points) {
                avg_dist +=
                    sqrtf((pt[0] - center[0]) * (pt[0] - center[0]) +
                    (pt[1] - center[1]) * (pt[1] - center[1]));
                avg_depth += pt[2];
                ++totalpts;
            }

            if (totalpts == 0) {
                avg_dist = avg_depth = 1.0;
                var_dist = var_depth = 0.0;
                return;
            }

            avg_dist /= totalpts;
            avg_depth /= totalpts;

            var_dist = var_depth = 0;

            for (const ht::Vec3f pt : points) {
                double dist =
                    sqrtf((pt[0] - center[0]) * (pt[0] - center[0]) +
                    (pt[1] - center[1]) * (pt[1] - center[1]));
                var_dist += (dist - avg_dist) * (dist - avg_dist);
                var_depth += (pt[2] - avg_depth) * (pt[2] - avg_depth);
            }

            var_dist /= totalpts;
            var_depth /= totalpts;
        }

        // SVM手部分类器实现

        const double SVMHandClassifier::DEFAULT_HYPERPARAMS[5 * NUM_SVMS] = {
            // gamma       coef0       C       eps     p
            0.8219,     0.5000,     0.5000, 9e-16,  0.0548,
            0.3425,     0.5000,     0.4041, 1e-16,  0.0548,
            0.3425,     0.5000,     0.5493, 1e-16,  0.0548,
            0.2740,     0.5000,     0.4100, 1e-16,  0.0548,
        };

        void SVMHandClassifier::initSVMs(const double hyperparams[5 * NUM_SVMS]) {
            for (int i = 0; i < NUM_SVMS; ++i) {
                svm[i] = cv::ml::SVM::create();
                svm[i]->setType(cv::ml::SVM::EPS_SVR);
                svm[i]->setKernel(cv::ml::SVM::RBF);
                svm[i]->setGamma(hyperparams[i * 5 + 0]);
                svm[i]->setCoef0(hyperparams[i * 5 + 1]);
                svm[i]->setC(hyperparams[i * 5 + 2]);
                svm[i]->setP(hyperparams[i * 5 + 4]);
            }
        }

        SVMHandClassifier::SVMHandClassifier() { }

        SVMHandClassifier::SVMHandClassifier(const std::string & path) {
            initSVMs();
            loadFile(util::resolveRootPath(path));
        }

        SVMHandClassifier::~SVMHandClassifier() {
            // 。。。

        }

        bool SVMHandClassifier::loadFile(const std::string & ipath) {
            using namespace boost::filesystem;

            const char * env = std::getenv("OPENARK_DIR");
            path filePath(ipath);

            if (env) filePath = path(env) / filePath;

            trained = true;

            for (int i = 0; i < NUM_SVMS; ++i) {
                path loadPath = filePath / ("svm_" + std::to_string(i) + ".xml");

                if (!boost::filesystem::exists(loadPath)){
                    trained = false;
                    break;
                }

                svm[i] = cv::ml::SVM::load(loadPath.string());
                if (!svm[i]->isTrained()) {
                    trained = false;
                    break;
                }
            }

            return trained;
        }

        bool SVMHandClassifier::exportFile(const std::string & opath) const {
            boost::filesystem::path filePath(opath);
            for (int i = 0; i < NUM_SVMS; ++i) {
                boost::filesystem::path savePath = filePath / ("svm_" + std::to_string(i) + ".xml");
                svm[i]->save(savePath.string());
            }
            return true;
        }

        bool SVMHandClassifier::train(const std::string & dataPath_, const double hyperparams[5 * NUM_SVMS]) {
            initSVMs(hyperparams);

            std::string dataPath = dataPath_;
            if (dataPath[dataPath.size() - 1] != '/' && dataPath[dataPath.size() - 1] != '\\') {
                dataPath += boost::filesystem::path::preferred_separator;
            }

            std::string labelsPath = dataPath + DATA_LABELS_FILE_NAME,
                featuresPath = dataPath + DATA_FEATURES_FILE_NAME;

            std::ifstream ifsLabels(labelsPath), ifsFeats(featuresPath);

            // 所有选项
            int N; ifsLabels >> N;

            // 忽略要素文件的第一行（特征名称）
            std::string _; getline(ifsFeats, _);

            // 记录SVM的特征数量 #
            int numFeats[NUM_SVMS];

            // 记录SVM的手指数量 #
            int numFing[NUM_SVMS];

            // 记录SVM样本数 #
            int numSamples[NUM_SVMS];

            memset(numFeats, 0x3f, sizeof numFeats); // 重置为最大值，63
            memset(numFing, 0x3f, sizeof numFing); // 重置为最大值，63
            memset(numSamples, 0, sizeof numSamples);

            // 用于确定数据矩阵尺寸的预扫描特征文件
            for (int i = 0; i < N; ++i) {
                std::string name;
                int numFeatures, numFingers;
                ifsFeats >> name >> numFeatures >> numFingers;

                int svmID = getSVMIdx(numFingers);

                if (svmID >= 0) {
                    ++numSamples[svmID];
                    numFeats[svmID] = std::min(numFeatures, numFeats[svmID]);
                    numFing[svmID] = std::min(numFingers, numFing[svmID]);
                }

                // 忽略该行的其余部分
                std::getline(ifsFeats, _);
            }

            // 从文件头开始
            ifsFeats.seekg(0, std::ios::beg); getline(ifsFeats, _);

            cv::Mat data[NUM_SVMS], labels[NUM_SVMS];

            for (int i = 0; i < NUM_SVMS; ++i) {
                data[i].create(numSamples[i], numFeats[i] - 1, CV_32F);
                labels[i].create(1, numSamples[i], CV_32S);
                std::cerr << numFeats[i] << " ";
            }

            // 用于记录读取的样本
            memset(numSamples, 0, sizeof numSamples);

            size_t i;
            for (i = 0; i < N; ++i) {
                std::string lbName = "", ftName = "";
                int label, numFeatures, numFingers;

                // 同步
                if (!(ifsLabels >> lbName >> label) || !(ifsFeats >> ftName >> numFeatures >> numFingers)) {
                    break;
                }
                while (lbName != ftName && (ifsLabels >> lbName >> label)) {}

                int currSVMId = getSVMIdx(numFingers);
                if (currSVMId < 0) {
                    std::getline(ifsFeats, _);
                    continue;
                }

                // 添加标签
                labels[currSVMId].at<int>(0, numSamples[currSVMId]) = label;
                float * ptr = data[currSVMId].ptr<float>(numSamples[currSVMId]++);

                // read features
                for (int j = 0; j < numFeatures - 1; ++j) {
                    if (j >= numFeats[currSVMId] - 1) {
                        // 忽略
                        std::getline(ifsFeats, _);
                        break;
                    }
                    ifsFeats >> ptr[j];
                }
            }

            // 清除旧指针并分配内存
            for (int i = 0; i < NUM_SVMS; ++i) {
                std::cout << "Loaded " << numSamples[i] << " training samples for SVM #" << i <<
                    " (" << numSamples[i] << " features)" << "\n";
            }

            for (int i = 0; i < NUM_SVMS; ++i) {
                std::cout << "Training SVM " << i << "...\n";
                auto trainData = cv::ml::TrainData::create(data[i], cv::ml::ROW_SAMPLE, labels[i]);
                svm[i]->train(trainData);
                trainData.release();
            }

            trained = true;

            std::cout << "\nTesting...\n";

            int good = 0, goodSVM[NUM_SVMS];
            memset(goodSVM, 0, sizeof goodSVM);

            ifsLabels.close(); ifsFeats.close();

            for (i = 0; i < NUM_SVMS; ++i) {
                for (int j = 0; j < data[i].rows; ++j) {
                    cv::Mat feats(1, numFeats[i], CV_32F);
                    feats.at<float>(0, 0) = (float)numFing[i];

                    int label = labels[i].at<int>(0, j);

                    float * ptr = data[i].ptr<float>(j);

                    for (int k = 0; k < data[i].cols; ++k) {
                        feats.at<float>(0, k + 1) = ptr[k];
                    }

                    double res = classify(feats);
                    if (res < 0.5 && label == 0 || res > 0.5 && label == 1) {
                        ++good;
                        ++goodSVM[i];
                    }
                }
            }

            std::cout << "Training Results:\n";
            for (int i = 0; i < NUM_SVMS; ++i) {
                std::cout << "\tSVM " << i << ":" <<
                    (double)goodSVM[i] / numSamples[i] * 100 << "% Correct\n";
            }

            std::cout << "Overall:" << (double)good / N * 100 << "% Correct\n\n";

            return trained;
        }

        double SVMHandClassifier::classify(const cv::Mat & features) const {
            if (!trained) throw ClassifierNotTrainedException();

            // 如果没有手指，停止预测手
            if (!features.data || features.cols == 0) return 0.0;

            int nFeat = features.cols;
            if (nFeat > MAX_FEATURES) nFeat = MAX_FEATURES;

            cv::Mat samples = features(cv::Rect(1, 0, nFeat - 1, 1));
            int svmIdx = getSVMIdx(features);
            double result = svm[svmIdx]->predict(samples);

            // 控制范围在 [0, 1]
            return std::max(std::min(1.0, result), 0.0);
        }

        double SVMHandClassifier::classify(ht::Hand & hand, const cv::Mat & depth_map, cv::Point top_left, int full_wid) const
        {
            cv::Mat features = extractFeatures(hand, depth_map, top_left, full_wid);
            return classify(features);
        }

        int SVMHandClassifier::getSVMIdx(const cv::Mat & features) {
            int numFingers = features.at<int>(0, 0);
            return getSVMIdx(numFingers);
        }

        int SVMHandClassifier::getSVMIdx(int num_fingers) {
            return std::min(num_fingers - 1, NUM_SVMS - 1);
        }

        cv::Mat SVMHandClassifier::extractFeatures(ht::Hand & hand,
            const cv::Mat & depth_map, cv::Point top_left, int full_wid) {

            if (full_wid < 0) full_wid = depth_map.cols;

            std::vector<float> result;

            int nFingers = hand.getNumFingers();
            result.push_back((float)nFingers);
            if (nFingers == 0) return cv::Mat::zeros(1, 1, CV_32F);

            cv::Vec3f center = hand.getPalmCenter();
            cv::Point centerij = hand.getPalmCenterIJ();

            double avgdist, vardist, avgdepth, vardepth;
            computeMeanAndVariance(hand.getPoints(), center, avgdist, vardist, avgdepth, vardepth);

            if (nFingers > 1) result.reserve(nFingers * 13 + 10);
            else result.reserve(20);

            double area = hand.getSurfArea();

            // 手掌中心的平均距离（所有点中）
            result.push_back(avgdist * 20.0);

            // 与手掌中心的差异（所有点）
            result.push_back(sqrt(vardist) * 25.0f);

            // 表面积
            result.push_back(area * 10.00f);

            // 深度变化（未使用深度平均值）
            result.push_back(sqrt(vardepth) * 25.0f);

            std::vector<ht::Point2i> cont = hand.getContour(),
                hull = hand.getConvexHull();
            std::vector<ht::Vec3f> wrist = hand.getWrist();

            double contArea = cv::contourArea(cont);

            // 轮廓面积作为凸包面积的一部分
            result.push_back(float(contArea / cv::contourArea(hull)));

            cv::Rect bounds = hand.getBoundingBox();

            // 轮廓区域作为边界框区域的一部分
            result.push_back(contArea / double(bounds.width * bounds.height));

            // 轮廓的弧长作为凸包弧长的一部分
            result.push_back(cv::arcLength(cont, true) /
                cv::arcLength(hull, true) * 0.5f);

            int pa, pb;
            double diam = util::diameter(cont, pa, pb);

            // 内切圆半径直径的大小
            result.push_back(hand.getCircleRadius() / diam * 2.0f);

			ht::Vec3f paXYZ = ht::util::averageAroundPoint(depth_map, cont[pa]);
			ht::Vec3f pbXYZ = ht::util::averageAroundPoint(depth_map, cont[pb]);

            // 集群的直径，投影到3d
            result.push_back(ht::util::euclideanDistance(paXYZ, pbXYZ));

            // 手腕宽度
            result.push_back(ht::util::euclideanDistance(wrist[0], wrist[1]));

            typedef std::pair<boost::polygon::detail::fpt64, int> pfi;

            // 按长度排序手指
            std::vector<pfi> fingerOrder;

			ht::Vec3f midWrist = wrist[0] + (wrist[1] - wrist[0]) / 2;

            double avgLen = 0.0;
            double avgMidWristDist = 0.0;
            for (int i = 0; i < nFingers; ++i) {
                cv::Vec3f finger = hand.getFingers()[i], defect = hand.getDefects()[i];

                double finger_len = ht::util::euclideanDistance(finger, defect);
                avgLen += finger_len;
                avgMidWristDist += ht::util::euclideanDistance(finger, midWrist);

                fingerOrder.push_back(pfi(finger_len, i));
            }
            std::sort(fingerOrder.begin(), fingerOrder.end(), std::greater<pfi>());

            // 手指平均长度
            result.push_back(avgLen / nFingers * 5.0f);

            // 手指到手腕中部的平均距离
            result.push_back(avgMidWristDist / nFingers * 2.0f);

            auto fingers = hand.getFingers(), defects = hand.getDefects();
            auto fingersIJ = hand.getFingersIJ(), defectsIJ = hand.getDefectsIJ();

            for (int k = 0; k < nFingers; ++k) {
                int j = fingerOrder[k].second;

                const cv::Vec3f & finger = fingers[j], &defect = defects[j];
                const cv::Point & fingerij = fingersIJ[j], &defectij = defectsIJ[j];

                result.push_back(ht::util::euclideanDistance(finger, defect) * 5.0f);
                result.push_back(ht::util::euclideanDistance(defect, center) * 5.0f);
                result.push_back(ht::util::euclideanDistance(finger, center) * 5.0f);

                result.push_back(ht::util::angleBetween3DVec(finger, defect, center) / PI);
                result.push_back(ht::util::angleBetweenPoints(fingerij, centerij, defectij) / PI);

                result.push_back(ht::util::pointToAngle(fingerij - centerij));
                result.push_back(ht::util::pointToAngle(defectij - centerij));

                if (nFingers > 1) {
                    double minDistDefect = depth_map.cols, minDistFinger = depth_map.cols;
                    double maxDistDefect = 0, maxDistFinger = 0;

                    for (int jj = 0; jj < nFingers; ++jj) {
                        if (j == jj) continue;

                        double distDefect = ht::util::euclideanDistance(defect, defects[jj]),
                            distFinger = ht::util::euclideanDistance(finger, fingers[jj]);

                        if (distDefect < minDistDefect) minDistDefect = distDefect;
                        if (distDefect > maxDistDefect) maxDistDefect = distDefect;
                        if (distFinger < minDistFinger) minDistFinger = distFinger;
                        if (distFinger > maxDistFinger) maxDistFinger = distFinger;
                    }

                    result.push_back(minDistFinger * 5.0);
                    result.push_back(maxDistFinger * 5.0);
                    result.push_back(minDistDefect * 5.0);
                    result.push_back(maxDistDefect * 5.0);
                }
            }

            for (unsigned i = 0; i < result.size(); ++i) {
                if (std::isnan(result[i])) {
                    result[i] = 1.0;
                }
                else if (result[i] >= FLT_MAX) {
                    result[i] = 100.0;
                }
            }

            cv::Mat mat(1, (int)result.size(), CV_32F);
            for (uint i = 0; i < result.size(); ++i) {
                mat.at<float>(0, i) = result[i];
            }
            return mat;
        }

        // SVM超平面参数
        const double SVMHandValidator::DEFAULT_HYPERPARAMS[5] = {
            // gamma       coef0       C       eps     p
            1.0959,     0.5000,     0.5048, 1.5e-16,  0.0548,
        };

        void SVMHandValidator::initSVMs(const double hyperparams[5]) {
            svm = cv::ml::SVM::create();
            svm->setType(cv::ml::SVM::EPS_SVR);
            svm->setKernel(cv::ml::SVM::RBF);
            svm->setGamma(hyperparams[0]);
            svm->setCoef0(hyperparams[1]);
            svm->setC(hyperparams[2]);
            svm->setTermCriteria(cv::TermCriteria(cv::TermCriteria::EPS, 10000, hyperparams[4]));
            svm->setP(hyperparams[4]);
        }

        SVMHandValidator::SVMHandValidator() { }

        SVMHandValidator::SVMHandValidator(const std::string & path) {
            initSVMs();
            loadFile(util::resolveRootPath(path));
        }

        SVMHandValidator::~SVMHandValidator() {

        }

        bool SVMHandValidator::loadFile(const std::string & ipath) {
            using namespace boost::filesystem;

            const char * FILE_NAME = "svm.xml";

            std::string loadPath = ipath + "/" + FILE_NAME;

            std::ifstream testIfs(loadPath);
            if (!testIfs) {
                return trained = false;
            }
			
            svm = cv::ml::SVM::load(loadPath);
			BOOST_LOG_TRIVIAL(info) << "Load SVM:" << loadPath << std::endl;
            trained = true;
            return trained;
        }

        bool SVMHandValidator::exportFile(const std::string & opath) const {
            boost::filesystem::path filePath(opath);
            boost::filesystem::path savePath = filePath / "svm.xml";
            svm->save(savePath.string());
            return true;
        }

        bool SVMHandValidator::train(const std::string & dataPath_, const double hyperparams[5]) {
            initSVMs(hyperparams);

            std::string dataPath = dataPath_;
            if (dataPath[dataPath.size() - 1] != '/' && dataPath[dataPath.size() - 1] != '\\') {
                dataPath += boost::filesystem::path::preferred_separator;
            }

            std::string labelsPath = dataPath + DATA_LABELS_FILE_NAME,
                featuresPath = dataPath + DATA_FEATURES_FILE_NAME;

            std::ifstream ifsLabels(labelsPath), ifsFeats(featuresPath);

            // 所有选项
            int N; ifsLabels >> N;

            // 忽略要素文件的第一行（特征名称）
            std::string _; getline(ifsFeats, _);

            int numFeats;
            ifsFeats >> _ >> numFeats;

            // 从文件头开始读取
            ifsFeats.seekg(0, std::ios::beg); getline(ifsFeats, _);

            cv::Mat data, labels;
            data.create(N, numFeats, CV_32F);
            labels.create(1, N, CV_32S);

            int i;
            for (i = 0; i < N; ++i) {
                std::string lbName = "", ftName = "";
                int label, numFeatures;

                // 同步
                if (!(ifsLabels >> lbName >> label) || !(ifsFeats >> ftName >> numFeatures)) {
                    break;
                }
                while (lbName != ftName && (ifsLabels >> lbName >> label)) {}

                // 添加标签
                labels.at<int>(0, i) = label;
                float * ptr = data.ptr<float>(i);

                // 读取特征信息
                for (int j = 0; j < numFeatures; ++j) {
                    if (j >= numFeats) {
                        // 忽略
                        std::getline(ifsFeats, _);
                        break;
                    }
                    ifsFeats >> ptr[j];
                }
            }

            // 清除旧指针并分配内存
            BOOST_LOG_TRIVIAL(info) << "Loaded " << i <<
                " training samples (" << numFeats << " features)";

			BOOST_LOG_TRIVIAL(info) << "Training SVM...";
            cv::Ptr<cv::ml::TrainData> trainData =
                cv::ml::TrainData::create(data, cv::ml::ROW_SAMPLE, labels);
            svm->train(trainData);
            trainData.release();

            trained = true;

			BOOST_LOG_TRIVIAL(info) << "\nTesting...";

            int good = 0;

            ifsLabels.close(); ifsFeats.close();

            for (int j = 0; j < data.rows; ++j) {
                cv::Mat feats = data.row(j);
                int label = labels.at<int>(0, j);

                double res = classify(feats);
                if (res < 0.5 && label == 0 || res > 0.5 && label == 1) {
                    ++good;
                }
            }

			BOOST_LOG_TRIVIAL(info) << "Training Results:";
			BOOST_LOG_TRIVIAL(info) << (double)good / N * 100.0 << "% Correct\n";

            return trained;
        }

        double SVMHandValidator::classify(const cv::Mat & features) const {
            if (!trained) throw ClassifierNotTrainedException();

            // 如果没有手指，停止预测手。
            if (features.data == nullptr || features.cols == 0) return 0.0f;
            float result = svm->predict(features);

            // 控制范围为 [0, 1]
            return std::max(std::min(1.0f, result), 0.0f);
        }

        double SVMHandValidator::classify(
			Hand & hand, 
            const cv::Mat & depth_map, cv::Point top_left, int full_wid, 
            int num_features, int avg_size) const
        {
            cv::Mat features = extractFeatures(hand, depth_map, top_left, full_wid, num_features, avg_size);
            return classify(features);
        }

        cv::Mat SVMHandValidator::extractFeatures(ht::Hand & hand,
            const cv::Mat & depth_map, cv::Point top_left, int full_wid,
            int num_features, int avg_size)
        {
            Point2f center = hand.getPalmCenterIJ() - top_left;
            Vec3f centerXYZ = hand.getPalmCenter();

            double wristDir = util::pointToAngle(-hand.getDominantDirection());

            cv::Mat result(1, num_features + 4, CV_32F);
            float * ptr = result.ptr<float>(0);

            std::vector<cv::Point2i> cont = hand.getContour();
            double contArea = cv::contourArea(cont);
            double boxArea = hand.getBoundingBox().area();

            ptr[num_features] = contArea / boxArea;

            int da, db;
            util::diameter(cont, da, db);
            Vec3f vda = util::averageAroundPoint(depth_map, cont[da] - top_left);
            Vec3f vdb = util::averageAroundPoint(depth_map, cont[db] - top_left);
            ptr[num_features + 1] = util::euclideanDistance(vda, vdb) * 10.0f;

            double avgDist, varDist, avgDepth, varDepth;
            computeMeanAndVariance(hand.getPoints(), hand.getPalmCenter(), avgDist, varDist, avgDepth, varDepth);

            ptr[num_features + 2] = sqrtf(varDist) * 25.0f;
            ptr[num_features + 3] = sqrtf(varDepth) * 25.0f;
            
            double step = PI / num_features * 2.0;

			double angle = 0.0;
            for (int i = 0; i < num_features; ++i) {
                 float rad = util::radiusInDirection(depth_map, center, angle, wristDir);
                 if (rad < 0.0f) rad = 0.0f;

                 Point2f dir = util::angleToPoint(static_cast<float>(angle + wristDir));
                 Point2i farPoint = center + rad * dir;

                 Vec3f pos = util::averageAroundPoint(depth_map, farPoint, avg_size);
                 ptr[i] = util::euclideanDistance(pos, centerXYZ) * 10;


                 angle += step;
            }

            return result;
        }


    }
}
