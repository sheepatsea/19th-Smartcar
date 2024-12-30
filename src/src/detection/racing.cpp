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
 * @file racing.cpp
 * @author Leo
 * @brief 追逐区检测
 * @version 0.1
 * @date 2024-01-11
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

class Racing
{
public:
    bool carStoping = false; // 停车标志
    bool sideLeft = true;          // AI标识左右侧使能
    int count=0;
    /**
     * @brief 场景类型
     *
     */
    enum TypeRace
    {
        None = 0, // AI检测
        Safe,     // 普通车辆
        Spy,      // 嫌疑车辆
        Danger    // 危险车辆
    };
    TypeRace typeRace = TypeRace::None; // 场景类型

    /**
     * @brief 嫌疑车辆逼停阶段
     *
     */
    enum StepSpy
    {
        Bypass=0,  // 车辆绕行
        Inside,  // 变道
        Resist   // 阻挡
    };

    StepSpy stepSpy = StepSpy::Bypass; // 嫌疑车辆逼停阶段

    enum StepDanger
    {
        Chasing=0,  //追逐
        Hitting,  //撞击
    };

    StepDanger stepDanger = StepDanger::Chasing;
    /**
     * @brief 检测与路径规划
     *
     * @param track 赛道识别结果
     * @param detection AI检测结果
     */
    bool process(bool AIFlag, Tracking &track, vector<PredictResult> predicts, vector<int> RoadWidth)
    {
        carStoping = false; // 停车标志
        switch (typeRace)
        {
            case TypeRace::None:          // AI检测
            {
                if(!AIFlag) return false;
                searchTypeRace(track, predicts); // 检索AI场景类型
                break;
            }
            case TypeRace::Safe: // 普通车辆
            {
                POINT targetCar = searchCar(track, sideLeft, 0);  //找车头
                if(targetCar.x>ROWSIMAGE*0.5) 
                {
                    typeRace = TypeRace::None;
                    sideLeft = true;
                }
                curtailTracking(track, !sideLeft, RoadWidth); // 远离小车
                break;
            }
            case TypeRace::Spy: // 嫌疑车辆
            {
                /**
                *嫌疑车辆逼停策略：绕行至车辆前方阻挡其运行
                */
                POINT targetCar = searchCar(track, sideLeft, 0);  //车头
                if (stepSpy == StepSpy::Bypass) // 车辆绕行阶段
                {
                    curtailTracking(track, !sideLeft, RoadWidth); // 远离小车
                    if(targetCar.x>ROWSIMAGE*0.5) stepSpy = StepSpy::Inside;                
                }
                else if (stepSpy == StepSpy::Inside) // 车辆变道
                {
                    // curtailTracking(track, sideLeft, RoadWidth); //靠近小车
                    // if(sideLeft && track.pointsEdgeRight[track.pointsEdgeRight.size()-40].y>=COLSIMAGE-3) stepSpy = StepSpy::Resist;
                    // else if(!sideLeft && track.pointsEdgeLeft[track.pointsEdgeLeft.size()-40].y<=3) stepSpy = StepSpy::Resist;
                    // count++;
                    // if (count > 50) // 变道完毕
                    // {
                    //     count = 0;
                    //     stepSpy = StepSpy::Resist;
                    // }

                    if(sideLeft)
                    {
                        if(track.pointsEdgeRight[100].y>=COLSIMAGE-3) stepSpy = StepSpy::Resist;
                        if(stepSpy==StepSpy::Inside)
                        {
                            POINT start = POINT(ROWSIMAGE - 31, COLSIMAGE - 31);
                            POINT end = POINT(20, 30);
                            POINT middle =
                                POINT((start.x + end.x) * 0.4, (start.y + end.y) * 0.6);
                            vector<POINT> input = {start, middle, end};
                            track.pointsEdgeRight = Bezier(0.02, input);
                            track.pointsEdgeLeft.clear();
                            for (int i = 0; i < track.pointsEdgeRight.size(); i ++)
                            {
                                track.pointsEdgeLeft.push_back(POINT(track.pointsEdgeRight[i].x, 0));
                            }
                        }
                
                    }
                    else{
                        if(track.pointsEdgeLeft[100].y<=3) stepSpy = StepSpy::Resist;
                        if(stepSpy==StepSpy::Inside)
                        {
                        POINT start = POINT(ROWSIMAGE - 31, 30);
                            POINT end = POINT(20, COLSIMAGE-30);
                            POINT middle = POINT((start.x + end.x) * 0.4, (start.y + end.y) * 0.6);
                            vector<POINT> input = {start, middle, end};
                            track.pointsEdgeLeft = Bezier(0.02, input);
                            
                            track.pointsEdgeRight.clear();
                            for (int i = 0; i < track.pointsEdgeLeft.size(); i ++)
                            {
                                track.pointsEdgeRight.push_back(POINT(track.pointsEdgeLeft[i].x, COLSIMAGE - 1));
                            }
                        }
                    }
                    
                }
                else if (stepSpy == StepSpy::Resist) // 停车阻挡逼停
                {
                    carStoping = true;
                    count++;
                    if (count > 50) // 停车逼停时间: 2.3s
                    {
                        sideLeft = true;
                        carStoping = false;
                        count = 0;
                        typeRace = TypeRace::None; // 完成，退出场景
                    }
                }
                break;
            }

            case TypeRace::Danger: // 危险车辆
            {
                /**
                *恐怖车辆逼停策略：沿赛道左/右侧通行，强行撞击车辆逼停
                */
                
                if(stepDanger==StepDanger::Chasing)
                {
                    POINT targetCar = searchCar(track, sideLeft, 1);  //找车尾
                    if(targetCar.x>ROWSIMAGE*0.1) stepDanger = StepDanger::Hitting;
                }
                else if(stepDanger==StepDanger::Hitting)
                {
                    curtailTracking(track, sideLeft, RoadWidth); //向车靠近
                    POINT targetCar = searchCar(track, sideLeft, 0);  //找车头
                    if(targetCar.x>ROWSIMAGE*0.5 || targetCar.x==0) count++;
                    if(count>=5)
                    {
                        sideLeft = true;
                        count=0;
                        typeRace = TypeRace::None;
                    }
                }
                break;
            }
        }

        if (typeRace == TypeRace::None)
            return false;
        else
            return true;
    }

    /**
     * @brief 图像绘制禁行区识别结果
     *
     * @param img 需要叠加显示的图像
     */
    void drawImage(Mat &img)
    {
        if (typeRace == TypeRace::Spy)
        {
            switch (stepSpy)
            {
            case StepSpy::Bypass:
                putText(img, "[4] RACE - SPY - Bypass", Point(COLSIMAGE / 2 - 50, 10), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA);
                break;
            case StepSpy::Inside:
                putText(img, "[4] RACE - SPY - Inside", Point(COLSIMAGE / 2 - 50, 10), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA);
                break;
            case StepSpy::Resist:
                putText(img, "[4] RACE - SPY - Resist", Point(COLSIMAGE / 2 - 50, 10), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA);
                break;
            default:
                break;
            }
        }
        else if (typeRace == TypeRace::Danger)
            putText(img, "[4] RACE - DANGER", Point(COLSIMAGE / 2 - 30, 10), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA);
        else if (typeRace == TypeRace::Safe)
            putText(img, "[4] RACE - Safe", Point(COLSIMAGE / 2 - 30, 10), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA);
    }

private:
    /**
     * @brief 检索AI场景类型
     *
     */
    void searchTypeRace(Tracking track, vector<PredictResult> predicts)
    {
        // 标识AI检测
        PredictResult result;
        for (int i = 0; i < predicts.size(); i++)
        {
            if (predicts[i].type == LABEL_SAFETY) // 普通车辆
            {
                printf("Safety\n");
                typeRace = TypeRace::Safe; // 场景类型
                result = predicts[i];
                break;
            }
            else if (predicts[i].type == LABEL_SPY) // 嫌疑车辆
            {
                printf("Spy\n");
                typeRace = TypeRace::Spy; // 场景类型
                result = predicts[i];
                break;
            }
            else if (predicts[i].type == LABEL_DANGER) // 危险车辆
            {
                printf("Danger\n");
                typeRace = TypeRace::Danger; // 场景类型
                result = predicts[i];
                break;
            }
        }
        if(typeRace != TypeRace::None)
        {
            int row = track.pointsEdgeLeft.size() - (result.y + result.height - track.rowCutUp);
            int disLeft = abs(result.x + result.width - track.pointsEdgeLeft[row].y);  //障碍右侧到左边线的水平距离
            int disRight = abs(track.pointsEdgeRight[row].y - result.x);  //右边线到障碍左侧的水平距离
            printf("%d %d\n", disLeft, disRight);
            if (disLeft >= disRight) {sideLeft=false;printf("Right\n");}
            else if(disLeft < disRight) {sideLeft=true;printf("Left\n");}   
        }
    }

    /**
     * @brief 检索目标图像坐标
     *
     * @param predicts AI识别结果
     * @param index 检索序号
     * @return PredictResult
     */
    POINT searchCar(Tracking track, bool sideLeft, int mode)
    {
        POINT result;
        result.x = result.y = 0;
        if(mode==0)
        {
            if(sideLeft)
            {
                for(int i=5;i<track.pointsEdgeLeft.size();i+=2)
                {
                    if(track.pointsEdgeLeft[i].y-track.pointsEdgeLeft[i-5].y<-10) 
                    {
                 //       printf("%d\n",track.pointsEdgeLeft[i].y-track.pointsEdgeLeft[i-5].y);
                        result.x = track.pointsEdgeLeft[i].x;
                        result.y = track.pointsEdgeLeft[i].y;
                        break;
                    }
                }            
            }
            else
            {
                for(int i=5;i<track.pointsEdgeRight.size();i+=2)
                {
                    if(track.pointsEdgeRight[i].y-track.pointsEdgeRight[i-5].y>10) 
                    {
                        result.x = track.pointsEdgeRight[i].x;
                        result.y = track.pointsEdgeRight[i].y;
                        break;
                    }
                }   
            }
        }
        else if(mode==1)
        {
            if(sideLeft)
            {
                for(int i=5;i<track.pointsEdgeLeft.size();i+=2)
                {
                    if(track.pointsEdgeLeft[i].y-track.pointsEdgeLeft[i-5].y>10) 
                    {
                        result.x = track.pointsEdgeLeft[i].x;
                        result.y = track.pointsEdgeLeft[i].y;
                        break;
                    }
                }            
            }
            else
            {
                for(int i=5;i<track.pointsEdgeRight.size();i+=2)
                {
                    if(track.pointsEdgeRight[i].y-track.pointsEdgeRight[i-5].y<-10) 
                    {
                        result.x = track.pointsEdgeRight[i].x;
                        result.y = track.pointsEdgeRight[i].y;
                        break;
                    }
                }   
            }            
        }   
        return result;     
    }

    /**
     * @brief 由小车位置缩减优化车道线（双车道→单车道）
     *
     * @param track
     * @param left
     */
    void curtailTracking(Tracking &track, bool left, vector<int> RoadWidth)
    {
        if (left) // 向左侧缩进
        {
            if (track.pointsEdgeRight.size() > track.pointsEdgeLeft.size())
                track.pointsEdgeRight.resize(track.pointsEdgeLeft.size());

            for (int i = 0; i < track.pointsEdgeRight.size(); i++)
            {
                track.pointsEdgeRight[i].y = track.pointsEdgeLeft[i].y + RoadWidth[i]/ 2;
               // if(typeRace==TypeRace::Safe) track.pointsEdgeRight[i].y = track.pointsEdgeLeft[i].y + RoadWidth[i]/ 2;
                //else track.pointsEdgeRight[i].y = (track.pointsEdgeRight[i].y + track.pointsEdgeLeft[i].y) / 2;
            }
        }
        else // 向右侧缩进
        {
            
            if (track.pointsEdgeRight.size() < track.pointsEdgeLeft.size())
                track.pointsEdgeLeft.resize(track.pointsEdgeRight.size());

            for (int i = 0; i < track.pointsEdgeLeft.size(); i++)
            {
                track.pointsEdgeLeft[i].y = track.pointsEdgeRight[i].y - RoadWidth[i]/ 2;
                //if(typeRace==TypeRace::Safe) track.pointsEdgeLeft[i].y = track.pointsEdgeRight[i].y - RoadWidth[i]/ 2;
                //else track.pointsEdgeLeft[i].y = (track.pointsEdgeRight[i].y + track.pointsEdgeLeft[i].y) / 2;
            }
        }
    }
};
