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
 * @file danger.cpp
 * @author Leo
 * @brief 危险区AI识别与路径规划
 * @version 0.1
 * @date 2024-01-09
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
 * @brief 危险区AI识别与路径规划类
 *
 */
class Danger
{
public:
    enum Step{
        None = 0,
        Start,   //识别到爆炸物
        Inside, 
        End
    };

    Step step = Step::None;
    PredictResult resultObs; // 避障目标锥桶
    bool findFlag = false;
    bool enable = false;     // 场景检测使能标志
    bool left = true;
    int block_stage=1;
    int first = 0;
    bool last_left = true;
    int passblock = 0;
    int counterExit=0;
    /**
     * @brief 危险区AI识别与路径规划处理
     *
     * @param track 赛道识别结果
     * @param predict AI检测结果
     * @return true
     * @return false
     */
    bool process(Mat imageBinary, Mat imageCorrect, Tracking &track, uint64_t frameTime, float speed, int DisLeaving, vector<int> RoadWidth, float UpScale, float block_scale, float distance_block, bool DangerFlag, int DangerMode)
    {

        if(step==Step::None)
        {
            if(DangerFlag)
            {
                first = 0;
                step = Step::Start;
                counterExit = 0;
            }
        }

        if(step==Step::Start)
        {
            findFlag = false;
            //锥桶检测
            Mat hsv;
            cvtColor(imageCorrect, hsv, COLOR_BGR2HSV);
            // 定义黄色的HSV范围
            Scalar lowerYellow(15, 100, 60);
            Scalar upperYellow(35, 255, 255);
            
            Mat Mask;
            inRange(hsv, lowerYellow, upperYellow, Mask);

            vector<vector<Point>> contours;
            findContours(Mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
            
            
            int x_Near = 0; // 行坐标
            if (!contours.empty())
            {
               // 选取距离最近的锥桶   
                Rect Cone;
                for(const auto& contour : contours)
                {
                    Rect Tempcone = boundingRect(contour);
                    if ( Tempcone.y>ROWSIMAGE*UpScale && Tempcone.y+Tempcone.height < ROWSIMAGE-DisLeaving && Tempcone.y > x_Near)
                    {
                        int row = track.pointsEdgeLeft.size() - (Tempcone.y - track.rowCutUp);
                        if (row < 0) // 太远不管
                        {
                           continue;
                        }
                        int disLeft = Tempcone.x + Tempcone.width - track.pointsEdgeLeft[row].y;  //障碍右侧到左边线的水平距离
                        int disRight = track.pointsEdgeRight[row].y - Tempcone.x;  //右边线到障碍左侧的水平距离
                        if(disLeft>=0 && disRight>=0)
                        {
                            findFlag = true;
                            Cone = Tempcone;
                            x_Near = Tempcone.y;
                        }
                    }
                }
                if(findFlag)
                {
                    resultObs.type = LABEL_CONE;
                    resultObs.x = Cone.x;
                    resultObs.y = Cone.y;
                    resultObs.height = Cone.height;
                    resultObs.width = Cone.width;     
                }
                else 
                {
                    counterExit ++;
                    if(counterExit>50)
                    {
                        step = Step::None;
                        return false;
                    }
                }
            }   
            if(DangerMode==0)
            {
                if(findFlag)
                {
                    // 障碍物方向判定（左/右）
                    int row = track.pointsEdgeLeft.size() - (resultObs.y - track.rowCutUp);
                    int disLeft = resultObs.x + resultObs.width - track.pointsEdgeLeft[row].y;  //障碍右侧到左边线的水平距离
                    int disRight = track.pointsEdgeRight[row].y - resultObs.x;  //右边线到障碍左侧的水平距离
                    if (disLeft <= disRight) //[1] 障碍物在赛道内， 且靠左
                    {
                        left = false;
                    }
                    else if (disLeft > disRight) //[2] 障碍物在赛道内，且靠右
                    {
                        left = true;
                    }
                    if(first==0)
                    {
                        last_left = left;
                        first = 1;
                    } 
                    else if(first==1 && last_left!=left) {step=Step::Inside;first=0;}
                }       
                if(first==1) curtailTracking(track, left, RoadWidth); // 缩减优化车道线（双车道→单车道）
                // if(left)
                // {
                //     for (int i = 0; i < track.pointsEdgeRight.size(); i++)
                //     {
                //         track.pointsEdgeRight[i].y = track.pointsEdgeLeft[i].y + RoadWidth[i]/ 2 + 20;
                //     }
                // }
                // else
                // {
                //     for (int i = 0; i < track.pointsEdgeLeft.size(); i++)
                //     {
                //         track.pointsEdgeLeft[i].y = track.pointsEdgeRight[i].y - RoadWidth[i]/ 2 - 20;
                //     }
                // }
            }
            else if(DangerMode==1)
            {
                if(findFlag) step=Step::Inside;
            }

        }

        if(step==Step::Inside)
        {
            findFlag = false;
            //锥桶检测
            Mat hsv;
            cvtColor(imageCorrect, hsv, COLOR_BGR2HSV);
            // 定义黄色的HSV范围
            Scalar lowerYellow(15, 100, 60);
            Scalar upperYellow(35, 255, 255);
            
            Mat Mask;
            inRange(hsv, lowerYellow, upperYellow, Mask);

            vector<vector<Point>> contours;
            findContours(Mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

            int x_Near = 0; // 框面积
            if (!contours.empty())
            {
               // 选取距离最近的锥桶   
                Rect Cone;
                for(const auto& contour : contours)
                {
                    Rect Tempcone = boundingRect(contour);
                    if (Tempcone.y > ROWSIMAGE*UpScale &&  Tempcone.y+Tempcone.height < ROWSIMAGE-DisLeaving && Tempcone.y > x_Near)
                    {
                        int row = track.pointsEdgeLeft.size() - (Tempcone.y - track.rowCutUp);
                        if (row < 0) // 太远不管
                        {
                           continue;
                        }
                        int disLeft = Tempcone.x + Tempcone.width - track.pointsEdgeLeft[row].y;  //障碍右侧到左边线的水平距离
                        int disRight = track.pointsEdgeRight[row].y - Tempcone.x;  //右边线到障碍左侧的水平距离
                        if(disLeft>=0 && disRight>=0)
                        {
                            findFlag = true;
                            Cone = Tempcone;
                            x_Near = Tempcone.y;
                        }
                    }
                }
                if(findFlag)
                {
                    resultObs.type = LABEL_CONE;
                    resultObs.x = Cone.x;
                    resultObs.y = Cone.y;
                    resultObs.height = Cone.height;
                    resultObs.width = Cone.width;     
                }
            } 
            if(!findFlag)  //障碍之间未识别到
            {
                //情况二 障碍与赛道边线无间隔
                for(int i=40;i<track.pointsEdgeLeft.size();i+=2)
                {
                     if(track.pointsEdgeLeft[i].y<track.pointsEdgeLeft[i-5].y - 5 /**/&& track.pointsEdgeLeft[i].x>ROWSIMAGE*block_scale)
                    {
                        distance=0;
                        step = Step::End;
                        passblock=0;
                        left=false;
                        break;
                    }
                    else if(track.pointsEdgeRight[i].y>track.pointsEdgeRight[i-5].y + 5/**/ && track.pointsEdgeLeft[i].x>ROWSIMAGE*block_scale)
                    {
                        distance=0;
                        step = Step::End;
                        passblock=0;
                        left = true;
                        break;
                    }
                }
                //单车道补线            
                if(step!= Step::End) 
                {
                    curtailTracking(track, left, RoadWidth);
                }
             }
            else
            {
                // 障碍物方向判定（左/右）
                int row = track.pointsEdgeLeft.size() - (resultObs.y - track.rowCutUp);
                int disLeft = resultObs.x + resultObs.width - track.pointsEdgeLeft[row].y;  //障碍右侧到左边线的水平距离
                int disRight = track.pointsEdgeRight[row].y - resultObs.x;  //右边线到障碍左侧的水平距离
                if (disLeft>0 && disRight>0 && disLeft <= disRight) //[1] 障碍物在赛道内， 且靠左
                {
                    left = false;
                    vector<POINT> points(4); // 三阶贝塞尔曲线
                    points[0] = track.pointsEdgeLeft[row / 2];
                    points[1] = {resultObs.y + resultObs.height, resultObs.x + resultObs.width};
                    points[2] = {(resultObs.y + resultObs.height + resultObs.y) / 2, resultObs.x + resultObs.width};
                    if (resultObs.y > track.pointsEdgeLeft[track.pointsEdgeLeft.size() - 1].x)
                        points[3] = track.pointsEdgeLeft[track.pointsEdgeLeft.size() - 1];
                    else
                        points[3] = {resultObs.y, resultObs.x + resultObs.width};

                    track.pointsEdgeLeft.resize((size_t)row / 2); // 删除错误路线
                    vector<POINT> repair = Bezier(0.01, points);  // 重新规划车道线
                    for (int i = 0; i < repair.size(); i++)
                    {
                         repair[i].y +=40;
                        // if(repair[i].y>COLSIMAGE) repair[i].y=COLSIMAGE-1;
                        track.pointsEdgeLeft.push_back(repair[i]);
                    }
                }
                else if (disLeft>0 && disRight>0 && disLeft > disRight) //[2] 障碍物在赛道内，且靠右
                {
                    left = true;   
                    vector<POINT> points(4); // 三阶贝塞尔曲线
                    points[0] = track.pointsEdgeRight[row / 2];
                    points[1] = {resultObs.y + resultObs.height, resultObs.x - resultObs.width};
                    points[2] = {(resultObs.y + resultObs.height + resultObs.y) / 2, resultObs.x - resultObs.width};
                    if (resultObs.y > track.pointsEdgeRight[track.pointsEdgeRight.size() - 1].x)
                        points[3] = track.pointsEdgeRight[track.pointsEdgeRight.size() - 1];
                    else
                        points[3] = {resultObs.y, resultObs.x};

                     track.pointsEdgeRight.resize((size_t)row / 2); // 删除错误路线
                     vector<POINT> repair = Bezier(0.01, points);   // 重新规划车道线
                    for (int i = 0; i < repair.size(); i++)
                    {
                         repair[i].y-=40;
                        // if(repair[i].y<0) repair[i].y=0;
                        track.pointsEdgeRight.push_back(repair[i]);
                    } 
                }
           }
         }  

         if(step == Step::End)
         {
            distance += speed * frameTime / 1000;
            if(distance>distance_block)
            {
                first = 0;
                step = Step::None;
                return false;
            } 
            curtailTracking(track, left, RoadWidth);
         }

        if(step==Step::None) return false;
        else return true;
    }           
    


    /**
     * @brief 图像绘制禁行区识别结果
     *
     * @param img 需要叠加显示的图像
     */
    void drawImage(Mat &img)
    {
        if (findFlag)
        {
            putText(img, "[2] DANGER - ENABLE", Point(COLSIMAGE / 2 - 30, 10), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA);
            cv::Rect rect(resultObs.x, resultObs.y, resultObs.width, resultObs.height);
            cv::rectangle(img, rect, cv::Scalar(0, 0, 255), 1);
        }
    }

private:
    float distance=0;

    /**
     * @brief 缩减优化车道线（双车道→单车道）
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
            }
        }
        else // 向右侧缩进
        {
            if (track.pointsEdgeRight.size() < track.pointsEdgeLeft.size())
                track.pointsEdgeLeft.resize(track.pointsEdgeRight.size());

            for (int i = 0; i < track.pointsEdgeLeft.size(); i++)
            {
                track.pointsEdgeLeft[i].y = track.pointsEdgeRight[i].y - RoadWidth[i]/ 2;
            }
        }
    }

};
