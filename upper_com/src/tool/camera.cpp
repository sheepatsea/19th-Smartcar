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
* @file camera.cpp
* @author HC
* @brief 图像感知
* @version 0.1
* @date 2023/12/14 10:11:42
* @copyright  :Copyright (c) 2023
* @note 具体功能模块:
*                    [01] 绘制田字格作为基准线
*                    [02] 通过基准线来调整摄像头位置
*/

#include <fstream>            // 文件操作类
#include <iostream>           // 输入输出类
#include <opencv2/opencv.hpp> // OpenCV终端部署
#include "../include/common.hpp"

using namespace std;
using namespace cv;

#define COLS_IMG 640 // 摄像头：图像的列数
#define ROWS_IMG 480 // 摄像头：图像的行数

int main(int argc, char const *argv[])
{
  // 打开摄像头
  VideoCapture capture("/dev/video0");
  
  if (!capture.isOpened())
  {
    cout << "can not open video device " << endl;
    return 1;
  }

   capture.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    capture.set(cv::CAP_PROP_FPS, 120);
    capture.set(cv::CAP_PROP_FRAME_WIDTH, COLSIMAGE);
    capture.set(cv::CAP_PROP_FRAME_HEIGHT, ROWSIMAGE);

  double rate = capture.get(CAP_PROP_FPS);            // 读取图像的帧率
  double width = capture.get(CAP_PROP_FRAME_WIDTH);   // 读取图像的宽度
  double height = capture.get(CAP_PROP_FRAME_HEIGHT); // 读取图像的高度
  cout << "Camera Param: frame rate = " << rate << " width = " << width
       << " height = " << height << endl;

  while (1)
  {
    Mat frame;
    if (!capture.read(frame))
    {
      cout << "no video frame" << endl;
      continue;
    }

    // 绘制田字格：基准线
    uint16_t rows = ROWS_IMG / 30; // 8
    uint16_t cols = COLS_IMG / 32; // 10

    for (size_t i = 1; i < rows; i++) // 使用for循环绘制行线
    {
      line(frame, Point(0, 30 * i), Point(frame.cols - 1, 30 * i), Scalar(211, 211, 211), 1);
    }
    for (size_t i = 1; i < cols; i++) // 使用for循环绘制列线
    {
      if (i == (int)(cols / 2))
        line(frame, Point(32 * i, 0), Point(32 * i, frame.rows - 1), Scalar(0, 0, 255), 2);
      else
        line(frame, Point(32 * i, 0), Point(32 * i, frame.rows - 1), Scalar(211, 211, 211), 1);
    }

    imshow("img", frame);
    waitKey(10);
  }
  capture.release();
}