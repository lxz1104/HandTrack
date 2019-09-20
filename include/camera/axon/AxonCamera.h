/***********************************************************
@File:PlaneDetector.h
@Author:lxz
@Date:2019-9-18
@Description: ����axon���������д�������
@History: NULL
************************************************************/

#pragma once
#ifndef AXON_CAMERA_H
#define AXON_CAMERA_H

// STL Libraries
#include <memory>

// OpenNI Libraries
#include <OpenNI.h>

// AXon camera Libraries
#include <AXonLink.h>

// base class
#include "DepthCamera.h"



namespace ht {

	/**
	 * @AXonCamera: AXon���������
	 * ��������AXon����������࣬��д��һЩͼ���ȡ�ķ�����
	 *
	 * @Type: Normal
	 */
	class AXonCamera : public DepthCamera
	{
	public:
		/**
		 * ���캯��
		 * @Descrip:������ʼ������������Ȳ���
		 * @param [in] params: ƽ����������ʵ��
		 */
		explicit AXonCamera();

		/** ����PMD���ʵ�� */
		~AXonCamera() override;

		/**
		 * ��ȡ����ͺ�
		 * @Descrip: ��дDepthCamrea���еĺ���
		 *
		 * @retun [std::string]: ��������ƻ��ͺ�
		 */
		const std::string getModelName() const override;

		/**
		 * ��ȡ���ͼ������ؿ���
		 * @Descrip: �˿�����������Դ���������
		 *
		 * @return [int] ͼ�����ؿ���
		 */
		int getWidth() const override;

		/**
		 * ��ȡ���ͼ������ظ߶�
		 * @Descrip: �˿�����������Դ���������
		 *
		 * @return [int] ͼ�����ظ߶�
		 */
		int getHeight() const override;

		/**
		 * ȷ��������������flagMap����Ч�Ե�ֵ
		 * @Descrip:
		 *
		 * return [float]
		 */
		float flagMapConfidenceThreshold() const override;

		/**
		 * ȷ�����AmpMap�ĵ���Ч��ֵ
		 * @Descrip:
		 *
		 * return [int]
		 */
		int ampMapInvalidFlagValue() const override;

		/**
		 * �Ƿ���Ҫ�����ܹ�������л�ȡAmpMap
		 * @Descrip: ֻ��Ҫ���ú����ķ���ֵ����;
		 * ������������ܻ�ȡAmpMapͼ�������������Ϊfalse��
		 * �������Ϊtrue�󣬱��뵽update()��ʵ�ֻ�ȡ��ͼ��ķ���
		 *
		 * @return [bool]
		 */
		bool hasAmpMap() const override;

		/**
		 * �Ƿ���Ҫ�����ܹ�������л�ȡflagMap
		 * @Descrip: ֻ��Ҫ���ú����ķ���ֵ����;
		 * ������������ܻ�ȡflagMapͼ�������������Ϊfalse��
		 * �������Ϊtrue�󣬱��뵽update()��ʵ�ֻ�ȡ��ͼ��ķ���
		 *
		 * @return [bool]
		 */
		bool hasFlagMap() const override;

		/** ָ��PMD���ʵ�� */
		typedef std::shared_ptr<AXonCamera> Ptr;

	protected:
		/**
		 * ������л����һ֡ͼ����Ϣ
		 * @Descrip: ���������л�ȡ��һ֡��XYZ,RGB,IR��ͼ��
		 * ͨ�����ô˺�����ˢ�¶�Ӧ��ͼ����Ϣ��������ڴ˺����д�����л�ȡͼ��ת��Ϊ��Ӧ��map��ʽ��
		 * ע�⣺��ȡ��***_map�Ŀ��߱���������getHeight(),getWidth()������ȡ����һ�£�������ܵ��³��������
		 * ע�⣺������ں���ĺ�����has***Map()����Ϊflase�����Ӧ��***_map����Բ��û�ȡ��
		 * ���磬����������ܹ���ȡ��RGBͼ��������hasRGBMap()����true,��ô�����Ҫ������rgb_map�������ݡ�
		 * ע�⣺�˷�������Ҫ����������ʵ�֣�����
		 *
		 * @param [out] xyz_map		���ͼ�������е��XYZ����. CV_32FC3
		 * @param [out] rgb_map		RGBͼ��. CV_8UC3
		 * @param [out] ir_map		����ͼ��. CV_8UC1
		 * @param [out] amp_map		���Ų���. CV_32FC1
		 * @param [out] flag_map	����ͼ��. CV_8UC1
		 *
		 * @return [void]
		 */
		void update(cv::Mat& xyz_map, cv::Mat& rgb_map, cv::Mat& ir_map,
			cv::Mat& amp_map, cv::Mat& flag_map) override;
	private:

		/**
		 * ��ʼ�����
		 */
		void initCamera();

		/** �豸���������� */
		openni::Device device;
		/** ��������� */
		openni::VideoStream depth;
		openni::VideoFrameRef frame;

		//���ͼ�����
		static const int depth_width = 640; // ���ͼ�����
		static const int depth_height = 480; // ���ͼ��߶�

	};

}

#endif //AXON_CAMERA_H