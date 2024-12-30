#pragma once
/**
 ********************************************************************************************************
 *                                               示例代码
 *                                             EXAMPLE  CODE
 *
 *                      (c) Copyright 2024; SaiShu.Lcc.; Leo; https://bjsstech.com
 *                                   版权所属[SASU-北京赛曙科技有限公司]
 *
 *            The code is for internal use only, not for commercial transactions(开源学习,请勿商用).
 *            The code ADAPTS the corresponding hardware circuit board(代码适配百度Edgeboard-智能汽车赛事版),
 *            The specific details consult the professional(欢迎联系我们,代码持续更正，敬请关注相关开源渠道).
 *********************************************************************************************************
 * @file preprocess.cpp
 * @author Leo
 * @brief 图像预处理：RGB转灰度图，图像二值化
 * @version 0.1
 * @date 2023-12-26
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <fstream>
#include <iostream>
#include <cmath>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "../include/common.hpp"

using namespace cv;
using namespace std;

/**
**[1] 读取视频
**[2] 图像二值化
*/
class Preprocess
{
public:
	/**
	 * @brief 图像矫正参数初始化
	 *
	 */
	Preprocess()
	{
		// 读取xml中的相机标定参数
		cameraMatrix = Mat(3, 3, CV_32FC1, Scalar::all(0)); // 摄像机内参矩阵
		distCoeffs = Mat(1, 5, CV_32FC1, Scalar::all(0));	// 相机的畸变矩阵
		FileStorage file;
		if (file.open("../res/calibration/valid/calibration.xml", FileStorage::READ)) // 读取本地保存的标定文件
		{
			file["cameraMatrix"] >> cameraMatrix;
			file["distCoeffs"] >> distCoeffs;
			cout << "相机矫正参数初始化成功!" << endl;
			enable = true;
		}
		else
		{
			cout << "打开相机矫正参数失败!!!" << endl;
			enable = false;
		}
	};

	/**
	 * @brief 图像二值化
	 *
	 * @param frame	输入原始帧
	 * @return Mat	二值化图像
	 */
	Mat binaryzation(Mat &frame)
	{
		Mat imageGray, imageBinary;

		cvtColor(frame, imageGray, COLOR_BGR2GRAY); // RGB转灰度图

		threshold(imageGray, imageBinary, 0, 255, THRESH_OTSU); // OTSU二值化方法

		return imageBinary;
	}

	/**
	 * @brief 矫正图像
	 *
	 * @param imagesPath 图像路径
	 */
	Mat correction(Mat &image)
	{
		if (enable)
		{
			Size sizeImage; // 图像的尺寸
			sizeImage.width = image.cols;
			sizeImage.height = image.rows;

			Mat mapx = Mat(sizeImage, CV_32FC1);	// 经过矫正后的X坐标重映射参数
			Mat mapy = Mat(sizeImage, CV_32FC1);	// 经过矫正后的Y坐标重映射参数
			Mat rotMatrix = Mat::eye(3, 3, CV_32F); // 内参矩阵与畸变矩阵之间的旋转矩阵

			// 采用initUndistortRectifyMap+remap进行图像矫正
			initUndistortRectifyMap(cameraMatrix, distCoeffs, rotMatrix, cameraMatrix, sizeImage, CV_32FC1, mapx, mapy);
			Mat imageCorrect = image.clone();
			remap(image, imageCorrect, mapx, mapy, INTER_LINEAR);

			// 采用undistort进行图像矫正
			//  undistort(image, imageCorrect, cameraMatrix, distCoeffs);

			return imageCorrect;
		}
		else
		{
			return image;
		}
	}

private:
	bool enable = false; // 图像矫正使能：初始化完成
	Mat cameraMatrix;	 // 摄像机内参矩阵
	Mat distCoeffs;		 // 相机的畸变矩阵
};
