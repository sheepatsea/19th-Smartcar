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
 * @file bridge.cpp
 * @author Leo ()
 * @brief 坡道（桥）路径处理
 * @version 0.1
 * @date 2022-06-26
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <fstream>
#include <iostream>
#include <cmath>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "../../include/common.hpp"
#include "../../include/detection.hpp"
#include "../recognition/tracking.cpp"

using namespace cv;
using namespace std;

class Bridge
{
public:
    enum Step{
        None = 0,
        Enable
    };
    
    Step step = Step::None;
    float dis = 0;

    bool process(Tracking &track, uint64_t frametime, float speed, vector<int> RoadWidth, int left_num, int right_num)
    {
        if(step==Step::None)
        {
            if(track.counter_white>=160 && track.stdevLeft<=10 &&track.stdevLeft>0 && track.stdevRight<=10 && track.stdevRight>0 && left_num>=160 && right_num>=160)
            {
                int cnt=0;
                for(int i=track.pointsEdgeLeft.size()-1;i>track.pointsEdgeLeft.size()-10;i--)
                {
                    int fact_width = track.pointsEdgeRight[i].y - track.pointsEdgeLeft[i].y;
                    int width1 = track.pointsEdgeRight[i].y - track.pointsEdgeLeft[i].y;
                    if(fact_width>0 && fact_width>RoadWidth[i]+20) cnt ++;
                }
                if(cnt>=5) step = Step::Enable;
            }
        }
        else if(step==Step::Enable)
        {
            if(track.counter_white<160) {dis=0;step = Step::None; return false;}
            dis += frametime*speed/1000;
            if(dis>=2.0) {dis=0;step = Step::None;}
            // int cnt=0;
            // for(int i=40;i<50;i++)
            // {
            //     int fact_width = track.pointsEdgeRight[i].y - track.pointsEdgeLeft[i].y;
            //     if(fact_width>0 && fact_width<RoadWidth[i]-20) cnt ++;
            // }

        }
        if(step==Step::Enable) return true;
        else return false; 
        // if (bridgeEnable) // 进入坡道
        // {
        //     if (track.pointsEdgeLeft.size() > ROWSIMAGE / 2 && track.pointsEdgeRight.size() > ROWSIMAGE / 2) // 切行，防止错误前瞻引发转向
        //     {
        //         track.pointsEdgeLeft.resize(track.pointsEdgeLeft.size() / 2);
        //         track.pointsEdgeRight.resize(track.pointsEdgeRight.size() / 2);
        //     }
        //     counterSession++;
        //     if (counterSession > 40) // 上桥40场图像后失效
        //     {
        //         counterRec = 0;
        //         counterSession = 0;
        //         bridgeEnable = false;
        //     }

        //     return true;
        // }
        // else // 检测坡道
        // {
        //     for (int i = 0; i < predict.size(); i++)
        //     {
        //         if (predict[i].type == LABEL_BRIDGE && predict[i].score > 0.6 && (predict[i].y + predict[i].height) > ROWSIMAGE * 0.32)
        //         {
        //             counterRec++;
        //             break;
        //         }
        //     }

        //     if (counterRec)
        //     {
        //         counterSession++;
        //         if (counterRec >= 4 && counterSession < 8)
        //         {
        //             counterRec = 0;
        //             counterSession = 0;
        //             bridgeEnable = true; // 检测到桥标志
        //             return true;
        //         }
        //         else if (counterSession >= 8)
        //         {
        //             counterRec = 0;
        //             counterSession = 0;
        //         }
        //     }

        //     return false;
        // }
    }

    /**
     * @brief 识别结果图像绘制
     *
     */
    void drawImage(Tracking track, Mat &image)
    {
        // 赛道边缘
        for (int i = 0; i < track.pointsEdgeLeft.size(); i++)
        {
            circle(image, Point(track.pointsEdgeLeft[i].y, track.pointsEdgeLeft[i].x), 1,
                   Scalar(0, 255, 0), -1); // 绿色点
        }
        for (int i = 0; i < track.pointsEdgeRight.size(); i++)
        {
            circle(image, Point(track.pointsEdgeRight[i].y, track.pointsEdgeRight[i].x), 1,
                   Scalar(0, 255, 255), -1); // 黄色点
        }

        if (bridgeEnable)
            putText(image, "[1] BRIDGE - ENABLE", Point(COLSIMAGE / 2 - 30, 10), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA);
    }

private:
    uint16_t counterSession = 0; // 图像场次计数器
    uint16_t counterRec = 0;     // 加油站标志检测计数器
    bool bridgeEnable = false;   // 桥区域使能标志
};