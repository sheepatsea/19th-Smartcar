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
 * @file parking.cpp
 * @author
 * @brief 停车区AI识别与路径规划
 * @version 0.1
 * @date 2024-01-02
 *
 * @copyright Copyright (c) 2024
 *
 */
#include <fstream>
#include <iostream>
#include <cmath>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "../../include/common.hpp"
#include "../../include/detection.hpp" // Ai模型预测

using namespace std;
using namespace cv;

/**
 * @brief 停车区AI识别与路径规划类
 *
 */
class Parking
{
private:
    /**
     * @brief 场景状态
     *
     */
    enum Step
    {
        init = 0, // 初始化屏蔽
        det,      // AI标识检测
        enable,   // 场景使能
        stop      // 准备停车
    };
    Step step = Step::init; // 场景状态
    uint16_t countRec = 0;  // AI场景识别计数器
    uint16_t countSes = 0;  // 场次计数器

public:
    float distance=0;
    uint16_t countExit = 0; // 程序退出计数器
    bool park = false;      // 停车标志

    /**
     * @brief 停车区AI识别与路径规划处理
     *
     * @param predict AI检测结果
     * @return true
     * @return false
     */
    bool process(Tracking track, uint64_t frameTime, float speed/*vector<PredictResult> predict*/)
    {
        if(step == Step::init)
        {
            if(track.garageEnable.x==1 && track.garageEnable.y>=ROWSIMAGE*0.3)
            {
                step = Step::stop;
                return true;
            }    
            else return false;
        }
        else
        {
            distance += speed*frameTime/1000;
            if(distance >= 0.05) 
            {
                park = true;
            }
            return true;
        }
        
  /*      switch (step)
        {
        case Step::init: // 初始化：起点斑马线屏蔽
            countSes++;
            for (int i = 0; i < predict.size(); i++)
            {
                if (predict[i].type == LABEL_CROSSWALK) // AI识别标志
                {
                    if ((predict[i].y + predict[i].height) > ROWSIMAGE * 0.2) // 标志距离计算
                    {
                        countSes = 0;
                        break;
                    }
                }
            }
            if (countSes > 50)
            {
                countSes = 0;
                step = Step::det;
            }
            break;

        case Step::det: // AI未识别
            for (int i = 0; i < predict.size(); i++)
            {
                if (predict[i].type == LABEL_CROSSWALK) // AI识别标志
                {
                    if ((predict[i].y + predict[i].height) > ROWSIMAGE * 0.4) // 标志距离计算
                    {
                        countRec++;
                        break;
                    }
                }
            }
            if (countRec) // 识别AI标志后开始场次计数
                countSes++;

            if (countSes >= 8)
            {
                if (countRec >= 4)
                {
                    step = Step::enable;
                }

                countRec = 0;
                countSes = 0;
            }
            break;

        case Step::enable: // 场景使能: 检测斑马线标识丢失
            countSes++;
            for (int i = 0; i < predict.size(); i++)
            {
                if (predict[i].type == LABEL_CROSSWALK) // AI识别标志
                {
                    if (predict[i].y > ROWSIMAGE * 0.3) // 标志距离计算
                    {
                        countSes = 0;
                        break;
                    }
                }
            }
            if (countSes > 10)
            {
                countExit = 0;
                step = Step::stop;
            }
            break;

        case Step::stop: // 准备停车
            park = true;
            countExit++; // 停车倒计时
            break;
        }

        // 输出场景状态结果
        if (step == Step::init || step == Step::det)
            return false;
        else
            return true;
     */       
    }

    /**
     * @brief 图像绘制禁行区识别结果
     *
     * @param img 需要叠加显示的图像
     */
    void drawImage(Mat &img)
    {
        if (step == Step::enable)
            putText(img, "[5] PARK - ENABLE", Point(COLSIMAGE / 2 - 30, 10), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA);
    }
};
