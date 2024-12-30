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
 * @file img2video.cpp
 * @author Leo
 * @brief 图像合成视频（mp4）
 * @version 0.1
 * @date 2023-02-21
 *
 * @copyright Copyright (c) 2023
 *
 */
#include <fstream>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "../include/common.hpp"

using namespace std;
using namespace cv;

int main()
{
    VideoWriter writer;
    int frame_fps = 20;
    int frame_width = 320;
    int frame_height = 240;

    writer = VideoWriter("../res/samples/sample.mp4", CV_FOURCC('P', 'I', 'M', '1'),
                         frame_fps, Size(frame_width, frame_height), true);
    cout << "frame_width is " << frame_width << endl;
    cout << "frame_height is " << frame_height << endl;
    cout << "frame_fps is " << frame_fps << endl;

    Mat img;
    for (int i = 1; i < 5000; i++) // 需要合成的图像编号
    {

        string image_name = "../res/samples/train/" + to_string(i) + ".jpg";

        img = imread(image_name);
        if (!img.empty())
        {
            writer << img;
        }
    }

    waitKey(1);
    return 0;
}
