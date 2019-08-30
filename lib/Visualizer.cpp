﻿#include "stdafx.h"
#include "Visualizer.h"
#include "Util.h"
#include <pcl/surface/gp3.h>

namespace ht {
    pcl::visualization::PCLVisualizer::Ptr Visualizer::viewer = nullptr;

	/***
	将矩阵值映射到[0，255]以供查看
	***/
    void Visualizer::visualizeMatrix(const cv::Mat & input, cv::Mat & output)
    {
        if (output.rows == 0) output.create(input.size(), CV_8UC3);
        cv::normalize(input, output, 0, 255, cv::NORM_MINMAX, CV_8UC3);
    }

	/***
	RGB-D图像可视化
	***/
    void Visualizer::visualizeXYZMap(const cv::Mat & xyzMap, cv::Mat & output, float max_depth)
    {
        cv::Mat depth;
        cv::extractChannel(xyzMap, depth, 2);
        output = depth * 255 / max_depth;
        output.convertTo(output, CV_8UC1);
        cv::applyColorMap(output, output, cv::COLORMAP_HOT);
    }

    void Visualizer::visualizeNormalMap(const cv::Mat & normal_map, cv::Mat & output, 
                                        int resolution)
    {
        if (output.rows == 0) output.create(normal_map.size() / resolution, CV_8UC3);

        for (int i = 0; i < output.rows; ++i) {
            const auto * inPtr = normal_map.ptr<Vec3f>(i * resolution);
            auto * outPtr = output.ptr<Vec3b>(i);

            for (int j = 0; j < output.cols; ++j) {
                int jj = j * resolution;

                if (inPtr[jj] == Vec3f(0, 0, 0)) {
                    outPtr[j] = Vec3b(0, 0, 0);
                }
                else {
                    outPtr[j][2] = (uchar)((inPtr[jj][0] + 1.0) * 127.5);
                    outPtr[j][1] = (uchar)((inPtr[jj][1] + 1.0) * 127.5);
                    outPtr[j][0] = (uchar)(-inPtr[jj][2] * 127.5 + 127.5);
                }
            }
        }

        cv::resize(output, output, normal_map.size(), (double)resolution,
            (double)resolution, cv::INTER_LINEAR);
    }

	/***
	手部对象可视化，使用像素坐标；用户可参考
	***/
    void Visualizer::visualizeHand(const cv::Mat & background, cv::Mat & output,
                Hand * hand, double display,
                const std::vector<std::shared_ptr<FramePlane> > * touch_planes)
    {
        if (background.type() == CV_32FC3)
        {
            Visualizer::visualizeXYZMap(background, output);
        }
        else if (output.rows == 0)
        {
            output = background.clone();
        }

		// 等比缩放,获得一个单位大小
        float unitWid = std::min((float)background.cols / 640, 1.75f);

		// 绘制手掌轮廓（手掌边界点的连线（像素坐标））
        if (hand->getContour().size() > 2) {
            cv::polylines(output, hand->getContour(), true, cv::Scalar(0, 255, 0), 1);
        }

        Point2i center = hand->getPalmCenterIJ();
        Vec3f centerXYZ = hand->getPalmCenter();
		// 绘制手掌中心（以掌心像素坐标为圆心画圆）
        cv::circle(output, center, static_cast<int>(std::round(unitWid * 10)), cv::Scalar(255, 0, 0));

		// 绘制手腕(通过手腕两边的顶点(wrists[0] 代表左边, wrists[1]代表右边)的(i,j)坐标位置，绘制矩形来表示手腕的两点)
		Point2i box = Point2i(int(unitWid * 6), int(unitWid * 6));
		const std::vector<Point2i>& wrists = hand->getWristIJ();
		// 左边手腕点
		cv::rectangle(output, wrists[0] - box, //左上顶点
			wrists[0] + box, //右下顶点
			cv::Scalar(255, 255, 0), int(unitWid * 1.5));
		// 右边手腕点
		cv::rectangle(output, wrists[1] - box, //左上顶点
			wrists[1] + box, //右下顶点
			cv::Scalar(255, 255, 0), int(unitWid * 1.5));

		// 绘制手掌的最大内切圆
        cv::circle(output, center, static_cast<int>(hand->getCircleRadius()), cv::Scalar(100, 100, 100),
                   static_cast<int>(std::round(unitWid)));

        const std::vector<Point2i> & fingers = hand->getFingersIJ();	// 被检测到的手指的指尖的像素坐标
        const std::vector<Point2i> & defects = hand->getDefectsIJ();	// 缺陷点像素坐标
        const std::vector<Vec3f> & fingersXYZ = hand->getFingers();		// 被检测到的手指的指尖的世界坐标
        const std::vector<Vec3f> & defectsXYZ = hand->getDefects();		// 缺陷点像素世界坐标

		// 绘制每根手指（绘制的是被检测到的手指，并不是绘制所有的手指）
        for (int i = 0; i < hand->getNumFingers(); ++i)
        {
			// 画手指
            cv::line(output, defects[i], fingers[i], cv::Scalar(0, 150, 255), roundf(unitWid * 2));
            cv::circle(output, fingers[i], static_cast<int>(roundf(unitWid * 7)), cv::Scalar(0, 0, 255), -1);

            std::stringstream sstr;
			// 缺陷点到指尖距离（利用xyz坐标计算得到的）
            sstr << std::setprecision(1) << std::fixed <<
                util::euclideanDistance(defectsXYZ[i], fingersXYZ[i]) * 100;

			// 显示指尖到缺陷点的距离
            cv::putText(output,
                sstr.str(), (defects[i] + fingers[i]) / 2 - Point2i(15, 0), 0,
                0.7 * unitWid, cv::Scalar(0, 255, 255), 1);

			// 可视化缺陷点
            cv::circle(output, defects[i], static_cast<int>(roundf(unitWid * 5)), cv::Scalar(255, 0, 200), -1);
			// 绘制缺陷点到掌心的连线
            cv::line(output, defects[i], center, cv::Scalar(255, 0, 200), roundf(unitWid * 2));

			// 显示缺陷到掌心的距离（利用xyz坐标计算得到的）
            sstr.str("");
            sstr << util::euclideanDistance(defectsXYZ[i], centerXYZ) * 100;
            cv::putText(output,
                sstr.str(),
                (defects[i] + center) / 2 - Point2i(15, 0), 0,
                0.4 * unitWid, cv::Scalar(0, 255, 255), 1);
        }
        
		// 绘制主方向箭头
        Point2f dir = hand->getDominantDirection();
        cv::arrowedLine(output, Point2f(center), Point2f(center) + dir * 50,
                        cv::Scalar(200, 200, 120), static_cast<int>(std::round(unitWid * 3)), 8, 0, 0.3);
            
        if (display < FLT_MAX) {
			// 显示文本信息: 需要在手部中心显示的自定义数值（后期用于区分左右手）
            std::stringstream sstr;
            sstr << std::setprecision(3) << std::fixed << display;
            Point2i dispPt = center - Point2i((int)sstr.str().size() * 8, 0);
            cv::putText(output, sstr.str(), dispPt, 0, 0.8, cv::Scalar(255, 255, 255), 1);
        }
    }

    void Visualizer::visualizePlaneRegression(const cv::Mat & input_mat, cv::Mat & output, std::vector<double> &equation, const double threshold, bool clicked)
    {
        if (input_mat.type() == CV_32FC3)
        {
            Visualizer::visualizeXYZMap(input_mat, output);
        }

        else
        {
            output = input_mat;
        }

        if (equation.size() < 3) return;

        cv::Scalar color = cv::Scalar(255 * (double)clicked, 255, 0);
        int pointsDetected = 0;

        for (int r = 0; r < input_mat.rows; r++)
        {
            const auto * ptr = input_mat.ptr<Vec3f>(r);

            for (int c = 0; c < input_mat.cols; c++)
            {
                const Vec3f & v = ptr[c];

                if (v[2] == 0) continue;

                double r_squared =
                    util::pointPlaneDistance((Vec3d)v, equation[0], equation[1], equation[2]);

                if (r_squared < threshold)
                {
                    cv::circle(output, Point2i(c, r), 1, color, -1);
                    ++ pointsDetected;
                }

            }

        }
    }


    void Visualizer::visualizePlanePoints(cv::Mat &input_mat, const std::vector<Point2i>& indicies)
    {
        for (auto & indicie : indicies) {
            input_mat.at<uchar>(indicie.y, indicie.x) = static_cast<uchar>(255);
        }
    }

    void Visualizer::visulizePolygonMesh(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
    {
        if (cloud.get()->width == 0)
        {
            return;
        }
        initPCLViewer();

        // Normal estimation*
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud);
        n.setInputCloud(cloud);
        n.setSearchMethod(tree);
        n.setKSearch(20);
        n.compute(*normals);
        //* normals should not contain the point normals + surface curvatures

        // Concatenate the XYZ and normal fields*
        pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
        pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
        //* cloud_with_normals = cloud + normals

        // Create search tree*
        pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
        tree2->setInputCloud(cloud_with_normals);

        // Initialize objects
        pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
        pcl::PolygonMesh triangles;

        // Set the maximum distance between connected points (maximum edge length)
        gp3.setSearchRadius(0.025);

        // Set typical values for the parameters
        gp3.setMu(2.5);
        gp3.setMaximumNearestNeighbors(100);
        gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
        gp3.setMinimumAngle(M_PI / 18); // 10 degrees
        gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees
        gp3.setNormalConsistency(false);

        // Get result
        gp3.setInputCloud(cloud_with_normals);
        gp3.setSearchMethod(tree2);
        gp3.reconstruct(triangles);

        Visualizer::viewer->setBackgroundColor(0, 0, 0);

        if (!Visualizer::viewer->updatePolygonMesh(triangles))
            Visualizer::viewer->addPolygonMesh(triangles);

        Visualizer::viewer->spinOnce();
    }

    bool Visualizer::initPCLViewer() {
        if (Visualizer::viewer != nullptr) return false;
        Visualizer::viewer = boost::make_shared<pcl::visualization::PCLVisualizer>("3D Viewport");
        Visualizer::viewer->registerKeyboardCallback([](const pcl::visualization::KeyboardEvent & evt) {
            // add handler to allow quit
            unsigned char k = evt.getKeyCode();
            if (k == 'Q' || k == 'q' || k == 27) {
                std::exit(0);
            }
        });
        return Visualizer::viewer != nullptr;
    }

    int Visualizer::createPCLViewport(double xmin, double ymin, double xmax, double ymax)
    {
        initPCLViewer();
        int id;
        Visualizer::viewer->createViewPort(xmin, ymin, xmax, ymax, id);
        return id;
    }

    pcl::visualization::PCLVisualizer::Ptr Visualizer::getPCLVisualizer()
    {
        initPCLViewer();
        return Visualizer::viewer;
    }
}
