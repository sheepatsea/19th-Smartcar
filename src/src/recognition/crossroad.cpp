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
 * @file crossroad.cpp
 * @author Leo
 * @brief 十字道路识别与图像处理
 * @version 0.1
 * @date 2022-03-14
 *
 * @copyright Copyright (c) 2022
 *
 * @note 十字道路处理步骤：
 *                      [01] 入十字类型识别：tracking.cpp
 *                      [02] 补线起止点搜索
 *                      [03] 边缘重计算
 *
 */

#include <fstream>
#include <iostream>
#include <cmath>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "../../include/common.hpp"
#include "tracking.cpp"
#include "../../include/mat_use.h"

using namespace cv;
using namespace std;

class Crossroad
{
public:
    bool crossstraight = true;
    bool last_left_down=false, last_right_down=false;
    Mat inv_Mat = auto_init_ipm_mat();
    /**
     * @brief 初始化
     *
     */
    void reset(void)
    {
        crossroadType = CrossroadType::None; // 十字道路类型
    }

    /**
     * @brief 十字道路识别与图像处理
     *
     * @param track 赛道识别结果
     * @param imagePath 输入图像
     */
    bool crossRecognition(Tracking &track)
    {
        int left_up = 0, left_stage=0;
        int left_down = 0, left_down_1=0;
        int right_up = 0, right_stage=0;
        int right_down = 0, right_down_1=0;
        
        // if (track.pointsEdgeRight.size() < ROWSIMAGE / 2 || track.pointsEdgeLeft.size() < ROWSIMAGE / 2) // 十字有效行限制
        //     return false;

        // vector<Point2f> pointsToTransform_left = convertPointsToCvPoints(track.pointsEdgeLeft),
        //         pointsToTransform_right = convertPointsToCvPoints(track.pointsEdgeRight), 
        //         transformedPoints;
        // vector<POINT> transformedPoints_Left, transformedPoints_Right;

        // //左边线逆透视
        // perspectiveTransform(pointsToTransform_left, transformedPoints, inv_Mat);
        // transformedPoints_Left = convertCvPointsToPoints(transformedPoints);
        // //右边线逆透视
        // perspectiveTransform(pointsToTransform_right, transformedPoints, inv_Mat);
        // transformedPoints_Right = convertCvPointsToPoints(transformedPoints);

        // transformedPoints_Left = blur_points(transformedPoints_Left, using_kernel_num);   //三角滤波
        // transformedPoints_Right = blur_points(transformedPoints_Right, using_kernel_num);
        // transformedPoints_Left = resample_points(transformedPoints_Left, using_resample_dist*pixel_per_meter);  //下采样
        // transformedPoints_Right = resample_points(transformedPoints_Right, using_resample_dist*pixel_per_meter);
        
        // for(int i=0;i<transformedPoints_Left.size()-3;i += 1)
        // {

        //     int n1_x = transformedPoints_Left[i].x - transformedPoints_Left[i+1].x; 
        //     int n1_y = transformedPoints_Left[i].y - transformedPoints_Left[i+1].y; 
        //     int n2_x = transformedPoints_Left[i+2].x - transformedPoints_Left[i+3].x; 
        //     int n2_y = transformedPoints_Left[i+2].y - transformedPoints_Left[i+3].y;
           
        //     double n1_len = sqrt(n1_x*n1_x + n1_x*n1_y);
        //     float n2_len = sqrt(n2_x*n2_x + n2_y*n2_y);
        //     float ang_cos = float(n1_x*n2_x + n1_y*n2_y) / (n1_len * n2_len);
        //     ang_cos = ang_cos>1 ? 1 : ang_cos;
        //     float ang = acos(ang_cos)*180/3.14159;
        //     if(ang>80 && n2_y>0) {left_down_1 = i;break;}
        // }

        for (int i = 10; i < track.pointsEdgeLeft.size() - 10; i += 2)
        {
            // if(!left_down && left_down_1!=0 && track.pointsEdgeLeft[i - 5].y - track.pointsEdgeLeft[i].y >= 2 
            //     && track.pointsEdgeLeft[i - 5].y - track.pointsEdgeLeft[i - 10].y >= 2) left_down = i-10;
           
            if (track.pointsEdgeLeft[i].y - track.pointsEdgeLeft[i - 5].y <= -20 &&
                track.pointsEdgeLeft[i - 5].y >= track.pointsEdgeLeft[i - 10].y)
            {
                if (left_down == 0)
                    left_down = i - 5;
                else if (track.pointsEdgeLeft[i - 5].y > track.pointsEdgeLeft[left_down].y && abs(track.pointsEdgeLeft[i - 5].x - track.pointsEdgeLeft[left_down].x) < 10)
                    left_down = i - 5;
            }
            if (2 * track.pointsEdgeLeft[i - 5].y - track.pointsEdgeLeft[i].y - track.pointsEdgeLeft[i - 10].y >= 10 &&
                track.pointsEdgeLeft[i - 5].y - track.pointsEdgeLeft[i].y >= 2 &&
                track.pointsEdgeLeft[i - 5].y - track.pointsEdgeLeft[i - 10].y >= 2)
            {
                if (left_down == 0)
                    left_down = i - 5;
                else if (track.pointsEdgeLeft[i - 5].y > track.pointsEdgeLeft[left_down].y && abs(track.pointsEdgeLeft[i - 5].x - track.pointsEdgeLeft[left_down].x) < 10)
                    left_down = i - 5;
            }

		        if (track.pointsEdgeLeft[i].y - track.pointsEdgeLeft[i - 5].y >= -5 &&
                    track.pointsEdgeLeft[i].y - track.pointsEdgeLeft[i - 5].y <= 4 &&
                    track.pointsEdgeLeft[i - 5].y - track.pointsEdgeLeft[i - 10].y >= 20)
            {
                if (left_up == 0)
                    left_up = i - 5;
                else if (track.pointsEdgeLeft[i].y > track.pointsEdgeLeft[left_up].y)
                    left_up = i - 5;
            }

        }

        // for(int i=2;i<transformedPoints_Right.size()-3;i += 1)
        // {
        //     int n1_x = transformedPoints_Right[i].x - transformedPoints_Right[i+1].x; 
        //     int n1_y = transformedPoints_Right[i].y - transformedPoints_Right[i+1].y; 
        //     int n2_x = transformedPoints_Right[i+2].x - transformedPoints_Right[i+3].x; 
        //     int n2_y = transformedPoints_Right[i+2].y - transformedPoints_Right[i+3].y; 
        //     float n1_len = sqrt(n1_x*n1_x + n1_y*n1_y);
        //     float n2_len = sqrt(n2_x*n2_x + n2_y*n2_y);
        //     float ang_cos = float(n1_x*n2_x + n1_y*n2_y) / (n1_len * n2_len);
        //     ang_cos = ang_cos>1 ? 1 : ang_cos;
        //     float ang = acos(ang_cos)*180/3.14159;
        //     ang = abs(ang)>90 ? (180-abs(ang)) : abs(ang);
        //     if(ang>80 && n2_y<0) {right_down_1 = i;break;}
        // }

        for (int i = 10; i < track.pointsEdgeRight.size() - 10; i += 2)
        {
            // if(!right_down && right_down_1!=0 &&track.pointsEdgeRight[i - 5].y - track.pointsEdgeRight[i].y <= -2 &&
            //     track.pointsEdgeRight[i - 5].y - track.pointsEdgeRight[i - 10].y <= -2) right_down = i-10;
            
            if (track.pointsEdgeRight[i].y - track.pointsEdgeRight[i - 5].y >= 20 &&
                track.pointsEdgeRight[i - 5].y <= track.pointsEdgeRight[i - 10].y)
            {
                if (right_down == 0)
                    right_down = i - 5;
                else if (track.pointsEdgeRight[i - 5].y < track.pointsEdgeRight[right_down].y && abs(track.pointsEdgeRight[i - 5].x - track.pointsEdgeRight[right_down].x) < 10)
                    right_down = i - 5;
            }
            if (2 * track.pointsEdgeRight[i - 5].y - track.pointsEdgeRight[i].y - track.pointsEdgeRight[i - 10].y <= -10 &&
                track.pointsEdgeRight[i - 5].y - track.pointsEdgeRight[i].y <= -2 &&
                track.pointsEdgeRight[i - 5].y - track.pointsEdgeRight[i - 10].y <= -2)
            {
                if (right_down == 0)
                    right_down = i - 5;
                else if (track.pointsEdgeRight[i - 5].y < track.pointsEdgeRight[right_down].y && abs(track.pointsEdgeRight[i - 5].x - track.pointsEdgeRight[right_down].x) < 10)
                    right_down = i - 5;
            }
            
		        if (track.pointsEdgeRight[i].y - track.pointsEdgeRight[i - 5].y <=5 &&
                    track.pointsEdgeRight[i].y - track.pointsEdgeRight[i - 5].y >= -4 &&
                track.pointsEdgeRight[i - 5].y - track.pointsEdgeRight[i - 10].y <= -20)
            {
                if (right_up == 0)
                    right_up = i - 5;
                else if (track.pointsEdgeRight[i].y < track.pointsEdgeRight[right_up].y)
                    right_up = i - 5;
            }
        }
        
        //if (track.pointsEdgeLeft[left_up].x >= track.pointsEdgeLeft[left_down].x) left_down = 0;
        //if (track.pointsEdgeRight[right_up].x >= track.pointsEdgeRight[right_down].x) right_down = 0;
        if (abs(track.pointsEdgeLeft[left_up].x - track.pointsEdgeLeft[left_down].x) < 10) left_up = 0;
        if (abs(track.pointsEdgeRight[right_up].x - track.pointsEdgeRight[right_down].x) < 10) right_up = 0;

        if (left_down != 0 && left_up != 0 && track.pointsEdgeLeft[left_up].x < track.pointsEdgeLeft[left_down].x)
        {
            if (track.pointsEdgeLeft[left_down].x != track.pointsEdgeLeft[left_up].x)
            {
                int temp_y;
                for (int i = left_down + 1; i < left_up; i ++)
                {
                    temp_y = track.pointsEdgeLeft[left_down].y +
                        (track.pointsEdgeLeft[left_down].x - track.pointsEdgeLeft[i].x) *
                        (track.pointsEdgeLeft[left_up].y - track.pointsEdgeLeft[left_down].y) /
                        (track.pointsEdgeLeft[left_down].x - track.pointsEdgeLeft[left_up].x);
                    if (temp_y < 0) track.pointsEdgeLeft[i].y = 0;
                    else if (temp_y >= COLSIMAGE) track.pointsEdgeLeft[i].y = COLSIMAGE - 1;
                    else track.pointsEdgeLeft[i].y = temp_y;
                }
            }
        }
        else
        {
            if (left_down != 0)
            {
                uint8_t left_begin = left_down > 20 ? left_down - 20 : 0;
                while (abs(track.pointsEdgeLeft[left_begin].y - track.pointsEdgeLeft[left_down].y) > 10 &&
                       left_begin < left_down - 10) left_begin ++;
                if (track.pointsEdgeLeft[left_begin].x != track.pointsEdgeLeft[left_down].x)
                {
                    // if(track.pointsEdgeLeft[left_down].y +
                    //         (track.pointsEdgeLeft[left_down].x - track.pointsEdgeLeft[track.pointsEdgeLeft.size()-1].x) *
                    //         (track.pointsEdgeLeft[left_down].y - track.pointsEdgeLeft[left_begin].y) /
                    //         (track.pointsEdgeLeft[left_begin].x - track.pointsEdgeLeft[left_down].x) >= track.pointsEdgeRight[track.pointsEdgeRight.size()-1].y)
                    //     {left_down=0;}
                    // else
                    // {
                        int temp_y;
                        for (int i = left_down + 1; i < track.pointsEdgeLeft.size(); i ++)
                        {
                            temp_y = track.pointsEdgeLeft[left_down].y +
                                (track.pointsEdgeLeft[left_down].x - track.pointsEdgeLeft[i].x) *
                                (track.pointsEdgeLeft[left_down].y - track.pointsEdgeLeft[left_begin].y) /
                                (track.pointsEdgeLeft[left_begin].x - track.pointsEdgeLeft[left_down].x);
                            if (temp_y < 0) track.pointsEdgeLeft[i].y = 0;
                            else if (temp_y >= COLSIMAGE) track.pointsEdgeLeft[i].y = COLSIMAGE - 1;
                            else track.pointsEdgeLeft[i].y = temp_y;
                        }
                    // }
                }
            }
            if (left_up != 0)
            {
                uint8_t left_end = left_up + 20 < track.pointsEdgeLeft.size() - 1 ? left_up + 20 : track.pointsEdgeLeft.size() - 1;
                while (abs(track.pointsEdgeLeft[left_end].y - track.pointsEdgeLeft[left_up].y) > 10 &&
                       left_end > left_up + 10) left_end --;
                if (track.pointsEdgeLeft[left_up].x != track.pointsEdgeLeft[left_end].x)
                {
                    // if(track.pointsEdgeLeft[left_up].y -
                    //         (track.pointsEdgeLeft[0].x - track.pointsEdgeLeft[left_up].x) *
                    //         (track.pointsEdgeLeft[left_end].y - track.pointsEdgeLeft[left_up].y) /
                    //         (track.pointsEdgeLeft[left_up].x - track.pointsEdgeLeft[left_end].x)>=track.pointsEdgeRight[0].y)
                    // {
                    //     left_up=0;   
                    // }
                    // else 
                    // {
                        int temp_y;
                        for (int i = left_up - 1; i >= 0; i --)
                        {
                            temp_y = track.pointsEdgeLeft[left_up].y -
                                (track.pointsEdgeLeft[i].x - track.pointsEdgeLeft[left_up].x) *
                                (track.pointsEdgeLeft[left_end].y - track.pointsEdgeLeft[left_up].y) /
                                (track.pointsEdgeLeft[left_up].x - track.pointsEdgeLeft[left_end].x);
                            if (temp_y < 0) track.pointsEdgeLeft[i].y = 0;
                            else if (temp_y >= COLSIMAGE) track.pointsEdgeLeft[i].y = COLSIMAGE - 1;
                            else track.pointsEdgeLeft[i].y = temp_y;
                        }
                    // }
                    
                }
            }
        }

        if (right_down != 0 && right_up != 0 && track.pointsEdgeRight[right_up].x < track.pointsEdgeRight[right_down].x)
        {
            if (track.pointsEdgeRight[right_down].x != track.pointsEdgeRight[right_up].x)
            {
                int temp_y;
                for (int i = right_down + 1; i < right_up; i ++)
                {
                    temp_y = track.pointsEdgeRight[right_down].y +
                        (track.pointsEdgeRight[right_down].x - track.pointsEdgeRight[i].x) *
                        (track.pointsEdgeRight[right_up].y - track.pointsEdgeRight[right_down].y) /
                        (track.pointsEdgeRight[right_down].x - track.pointsEdgeRight[right_up].x);
                    if (temp_y < 0) track.pointsEdgeRight[i].y = 0;
                    else if (temp_y >= COLSIMAGE) track.pointsEdgeRight[i].y = COLSIMAGE - 1;
                    else track.pointsEdgeRight[i].y = temp_y;
                }
            }
        }
        else
        {
            if (right_down != 0)
            {
                uint8_t right_begin = right_down > 20 ? right_down - 20 : 0;
                while (abs(track.pointsEdgeRight[right_begin].y - track.pointsEdgeRight[right_down].y) > 10 &&
                       right_begin < right_down - 10) right_begin ++;
                // if (track.pointsEdgeRight[right_begin].x != track.pointsEdgeRight[right_down].x)
                // {
                //     if(track.pointsEdgeRight[right_down].y +
                //             (track.pointsEdgeRight[right_down].x - track.pointsEdgeRight[track.pointsEdgeRight.size()-1].x) *
                //             (track.pointsEdgeRight[right_down].y - track.pointsEdgeRight[right_begin].y) /
                //             (track.pointsEdgeRight[right_begin].x - track.pointsEdgeRight[right_down].x)<=track.pointsEdgeLeft[track.pointsEdgeLeft.size()-1].y)
                //     {
                //         right_down=0;   
                //     }
                //     else
                //     {
                         int temp_y;
                        for (int i = right_down + 1; i < track.pointsEdgeRight.size(); i ++)
                        {
                            temp_y = track.pointsEdgeRight[right_down].y +
                                (track.pointsEdgeRight[right_down].x - track.pointsEdgeRight[i].x) *
                                (track.pointsEdgeRight[right_down].y - track.pointsEdgeRight[right_begin].y) /
                                (track.pointsEdgeRight[right_begin].x - track.pointsEdgeRight[right_down].x);
                            if (temp_y < 0) track.pointsEdgeRight[i].y = 0;
                            else if (temp_y >= COLSIMAGE) track.pointsEdgeRight[i].y = COLSIMAGE - 1;
                            else track.pointsEdgeRight[i].y = temp_y;
                        }
                    // }
    
            }
        
            if (right_up != 0)
            {
                uint8_t right_end = right_up + 20 < track.pointsEdgeRight.size() - 1 ? right_up + 20 : track.pointsEdgeRight.size() - 1;
                while (abs(track.pointsEdgeRight[right_end].y - track.pointsEdgeRight[right_up].y) > 10 &&
                       right_end > right_up + 10) right_end --;
                // if (track.pointsEdgeRight[right_up].x != track.pointsEdgeRight[right_end].x)
                // {
                //     if(track.pointsEdgeRight[right_up].y -
                //             (track.pointsEdgeRight[0].x - track.pointsEdgeRight[right_up].x) *
                //             (track.pointsEdgeRight[right_end].y - track.pointsEdgeRight[right_up].y) /
                //             (track.pointsEdgeRight[right_up].x - track.pointsEdgeRight[right_end].x)<=track.pointsEdgeLeft[0].y)
                //     {
                //         right_up=0;   
                //     }
                //     else
                //     {
                        int temp_y;
                        for (int i = right_up - 1; i >= 0; i --)
                        {
                            temp_y = track.pointsEdgeRight[right_up].y -
                                (track.pointsEdgeRight[i].x - track.pointsEdgeRight[right_up].x) *
                                (track.pointsEdgeRight[right_end].y - track.pointsEdgeRight[right_up].y) /
                                (track.pointsEdgeRight[right_up].x - track.pointsEdgeRight[right_end].x);
                            if (temp_y < 0) track.pointsEdgeRight[i].y = 0;
                            else if (temp_y >= COLSIMAGE) track.pointsEdgeRight[i].y = COLSIMAGE - 1;
                            else track.pointsEdgeRight[i].y = temp_y;
                        }
                    // }
                    
                
            }
        }
        
        if (left_up || left_down || right_up || right_down)
        {
            if (left_down && track.pointsEdgeLeft[left_down].x > 50 && track.pointsEdgeLeft[left_down].y > 160)
                crossroadType = CrossroadType::CrossroadRight;
            else if (right_down && track.pointsEdgeRight[right_down].x > 50 && track.pointsEdgeRight[right_down].y < 160)
                crossroadType = CrossroadType::CrossroadLeft;
            else
                crossroadType = CrossroadType::CrossroadStraight;
        }
        else
            crossroadType = CrossroadType::None;
        
        if (crossroadType == CrossroadType::CrossroadLeft || crossroadType == CrossroadType::CrossroadRight)
            crossstraight = false;
        else
            crossstraight = true;

        return left_up || left_down || right_up || right_down;
    }

    /**
     * @brief 绘制十字道路识别结果
     *
     * @param Image 需要叠加显示的图像/RGB
     */
    void drawImage(Tracking track, Mat &Image)
    {
        // // 绘制边缘点
        // for (int i = 0; i < track.pointsEdgeLeft.size(); i++)
        // {
        //     circle(Image, Point(track.pointsEdgeLeft[i].y, track.pointsEdgeLeft[i].x), 2,
        //            Scalar(0, 255, 0), -1); // 绿色点
        // }
        // for (int i = 0; i < track.pointsEdgeRight.size(); i++)
        // {
        //     circle(Image, Point(track.pointsEdgeRight[i].y, track.pointsEdgeRight[i].x), 2,
        //            Scalar(0, 255, 255), -1); // 黄色点
        // }

        // // 绘制岔路点
        // for (int i = 0; i < track.spurroad.size(); i++)
        // {
        //     circle(Image, Point(track.spurroad[i].y, track.spurroad[i].x), 6,
        //            Scalar(0, 0, 255), -1); // 红色点
        // }

        // // 斜入十字绘制补线起止点
        // if (crossroadType == CrossroadType::CrossroadRight) // 右入十字
        // {
        //     circle(Image, Point(pointBreakLU.y, pointBreakLU.x), 5, Scalar(226, 43, 138), -1); // 上补线点：紫色
        //     circle(Image, Point(pointBreakLD.y, pointBreakLD.x), 5, Scalar(255, 0, 255), -1);  // 下补线点：粉色
        //     if (pointBreakRU.x > 0)
        //         circle(Image, Point(pointBreakRU.y, pointBreakRU.x), 5, Scalar(226, 43, 138), -1); // 上补线点：紫色
        //     if (pointBreakRD.x > 0)
        //         circle(Image, Point(pointBreakRD.y, pointBreakRD.x), 5, Scalar(255, 0, 255), -1); // 下补线点：粉色

        //     putText(Image, "Right", Point(COLSIMAGE / 2 - 15, ROWSIMAGE - 20), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA);
        // }
        // else if (crossroadType == CrossroadType::CrossroadLeft) // 左入十字
        // {
        //     circle(Image, Point(pointBreakRU.y, pointBreakRU.x), 5, Scalar(226, 43, 138), -1); // 上补线点：紫色
        //     circle(Image, Point(pointBreakRD.y, pointBreakRD.x), 5, Scalar(255, 0, 255), -1);  // 下补线点：粉色
        //     if (pointBreakLU.x > 0)
        //         circle(Image, Point(pointBreakLU.y, pointBreakLU.x), 5, Scalar(226, 43, 138), -1); // 上补线点：紫色
        //     if (pointBreakLD.x > 0)
        //         circle(Image, Point(pointBreakLD.y, pointBreakLD.x), 5, Scalar(255, 0, 255), -1); // 下补线点：粉色

        //     putText(Image, "Left", Point(COLSIMAGE / 2 - 15, ROWSIMAGE - 20), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA);
        // }
        // else if (crossroadType == CrossroadType::CrossroadStraight) // 直入十字
        // {
        //     circle(Image, Point(pointBreakLU.y, pointBreakLU.x), 5, Scalar(226, 43, 138), -1); // 上补线点：紫色
        //     circle(Image, Point(pointBreakLD.y, pointBreakLD.x), 5, Scalar(255, 0, 255), -1);  // 下补线点：粉色
        //     circle(Image, Point(pointBreakRU.y, pointBreakRU.x), 5, Scalar(226, 43, 138), -1); // 上补线点：紫色
        //     circle(Image, Point(pointBreakRD.y, pointBreakRD.x), 5, Scalar(255, 0, 255), -1);  // 下补线点：粉色
        //     putText(Image, "Straight", Point(COLSIMAGE / 2 - 20, ROWSIMAGE - 20), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA);
        // }

        // putText(Image, "[6] CROSS - ENABLE", Point(COLSIMAGE / 2 - 30, 10), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA);
        // putText(Image, to_string(_index), Point(COLSIMAGE / 2 - 5, ROWSIMAGE - 40), cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 0, 155), 1, CV_AA);
    }

private:
    int _index = 0; // 测试

    POINT pointBreakLU;
    POINT pointBreakLD;
    POINT pointBreakRU;
    POINT pointBreakRD;
    uint16_t counterFild = 0;
    /**
     * @brief 十字道路类型
     *
     */
    enum CrossroadType
    {
        None = 0,
        CrossroadLeft,     // 左斜入十字
        CrossroadRight,    // 右斜入十字
        CrossroadStraight, // 直入十字
    };

    CrossroadType crossroadType = CrossroadType::None; // 十字道路类型

    /**
     * @brief 搜索十字赛道突变行（左上）
     *
     * @param pointsEdgeLeft
     * @return uint16_t
     */
    uint16_t searchBreakLeftUp(vector<POINT> pointsEdgeLeft)
    {
        uint16_t rowBreakLeftUp = pointsEdgeLeft.size() - 5;
        uint16_t counter = 0;
        uint16_t counterFilter = 0;
        for (int i = pointsEdgeLeft.size() - 5; i > 50; i--)
        {
            if (pointsEdgeLeft[i].y > 2 & abs(pointsEdgeLeft[i].y - pointsEdgeLeft[i + 1].y) < 3)
            {
                rowBreakLeftUp = i;
                counter = 0;
                counterFilter++;
            }
            else if (pointsEdgeLeft[i].y <= 2 && counterFilter > 10)
            {
                counter++;
                if (counter > 5)
                    return rowBreakLeftUp;
            }
        }

        return rowBreakLeftUp;
    }
    /**
     * @brief 搜索十字赛道突变行（左下）
     *
     * @param pointsEdgeLeft
     * @return uint16_t
     */
    uint16_t searchBreakLeftDown(vector<POINT> pointsEdgeLeft)
    {
        uint16_t rowBreakLeft = 0;
        uint16_t counter = 0;

        for (int i = 0; i < pointsEdgeLeft.size() / 2; i++) // 寻找左边跳变点
        {
            if (pointsEdgeLeft[i].y > pointsEdgeLeft[rowBreakLeft].y)
            {
                rowBreakLeft = i;
                counter = 0;
            }
            else if (pointsEdgeLeft[i].y <= pointsEdgeLeft[rowBreakLeft].y) // 突变点计数
            {
                counter++;
                if (counter > 5)
                    return rowBreakLeft;
            }
        }

        return rowBreakLeft;
    }
    /**
     * @brief 搜索十字赛道突变行（右上）
     *
     * @param pointsEdgeRight
     * @return uint16_t
     */
    uint16_t searchBreakRightUp(vector<POINT> pointsEdgeRight)
    {
        uint16_t rowBreakRightUp = pointsEdgeRight.size() - 5;
        uint16_t counter = 0;
        uint16_t counterFilter = 0;
        for (int i = pointsEdgeRight.size() - 5; i > 50; i--)
        {
            if (pointsEdgeRight[i].y < COLSIMAGE - 2 & abs(pointsEdgeRight[i].y - pointsEdgeRight[i + 1].y) < 3)
            {
                rowBreakRightUp = i;
                counter = 0;
                counterFilter++;
            }
            else if (pointsEdgeRight[i].y >= COLSIMAGE - 2 && counterFilter > 10)
            {
                counter++;
                if (counter > 5)
                    return rowBreakRightUp;
            }
        }

        return rowBreakRightUp;
    }
    /**
     * @brief 搜索十字赛道突变行（右下）
     *
     * @param pointsEdgeRight
     * @return uint16_t
     */
    uint16_t searchBreakRightDown(vector<POINT> pointsEdgeRight)
    {
        uint16_t rowBreakRightDown = 0;
        uint16_t counter = 0;
        bool start = false;

        for (int i = 0; i < pointsEdgeRight.size() - 10; i++) // 寻找左边跳变点
        {
            if (pointsEdgeRight[i].y < COLSIMAGE - 1)
                counter++;
            else
                counter = 0;

            if (counter > 2)
            {
                start = true;
                counter = 0;
            }

            if (start) // 屏蔽初始行
            {
                if (pointsEdgeRight[i].y > pointsEdgeRight[i - 2].y)
                    counter++;
                else
                    counter = 0;

                if (counter > 2)
                    return i - 3;
            }
        }

        return rowBreakRightDown;
    }

    /**
     * @brief 直入十字搜索
     *
     * @param pointsEdgeLeft 赛道左边缘点集
     * @param pointsEdgeRight 赛道右边缘点集
     * @return true
     * @return false
     */
    bool searchStraightCrossroad(vector<POINT> pointsEdgeLeft, vector<POINT> pointsEdgeRight)
    {
        if (pointsEdgeLeft.size() < ROWSIMAGE * 0.8 || pointsEdgeRight.size() < ROWSIMAGE * 0.8)
        {
            return false;
        }

        uint16_t counterLeft = 0;
        uint16_t counterRight = 0;
        for (int i = pointsEdgeLeft.size() - 10; i > 1; i--) // 搜索上半部分边缘点
        {
            if (pointsEdgeLeft[i].x > ROWSIMAGE / 2)
                break;
            else if (pointsEdgeLeft[i].y < 2)
                counterLeft++;
        }
        for (int i = pointsEdgeRight.size() - 10; i > 1; i--) // 搜索上半部分边缘点
        {
            if (pointsEdgeRight[i].x > ROWSIMAGE / 2)
                break;
            else if (pointsEdgeRight[i].y > COLSIMAGE - 2)
                counterRight++;
        }
        if (counterLeft > 30 && counterRight > 30)
            return true;
        else
            return false;
    }
};