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
 * @file controlcenter.cpp
 * @author Leo
 * @brief 智能车控制中心计算
 * @version 0.1
 * @date 2022-02-18
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <fstream>
#include <iostream>
#include <cmath>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "../include/common.hpp"
#include "recognition/tracking.cpp"
#include "mat_use.cpp"
using namespace cv;
using namespace std;

#define big_conner_coefficient 1.3

class ControlCenter
{
public:
    int aim_idx, speed_aim_idx;
    int left_num=0, right_num=0;
    int controlCenter;           // 智能车控制中心（0~320）
    vector<POINT> centerEdge, centerEdge_show,
                    centerEdge1, centerEdge_show1;    // 赛道中心点集
    vector<Point2f> last_pointsToTransform_left, last_pointsToTransform_right;
    uint16_t validRowsLeft = 0;  // 边缘有效行数（左）
    uint16_t validRowsRight = 0; // 边缘有效行数（右）
    double sigmaCenter = 0;      // 中心点集的方差
    Mat invMat_auto = auto_init_ipm_mat();
    Mat inverse = invMat_auto.inv();;
    
    vector<POINT> transformedPoints_Left, transformedPoints_Right,
                    last_transformedPoints_Left, last_transformedPoints_Right;  //逆透视后左右边线

    vector<POINT> speed_centeredge;
    /**
     * @brief 控制中心计算
     *
     * @param pointsEdgeLeft 赛道左边缘点集
     * @param pointsEdgeRight 赛道右边缘点集
     */

    void fitting(Tracking &track, uint8_t calstartline, vector<int> RoadWidth, Scene scene,float aim_distance, bool R100, int ringstep, int ringleft)
    {
        sigmaCenter = 0;
        controlCenter = COLSIMAGE / 2;
        centerEdge.clear();
        centerEdge_show.clear();
        vector<POINT> v_center(4); // 三阶贝塞尔曲线
        style = "STRIGHT";
        int premode = 0;   ////////

        if(scene==Scene::NormalScene || scene==Scene::CrossScene || scene==Scene::ParkingScene || scene==Scene::RingScene || scene==Scene::RescueScene || scene==Scene::BridgeScene)
        {
            Size imageSize(320, 240);
            Mat only_boundary = Mat::zeros(imageSize, CV_8UC1);

            vector<Point2f> pointsToTransform_left = convertPointsToCvPoints(track.pointsEdgeLeft),
                            pointsToTransform_right = convertPointsToCvPoints(track.pointsEdgeRight), 
                            transformedPoints;
                            
            //防止点集为空导致逆透视爆炸
            if(pointsToTransform_left.size()<5 || pointsToTransform_left.size()> 200)  pointsToTransform_left = last_pointsToTransform_left;
            else last_pointsToTransform_left = pointsToTransform_left;
            if(pointsToTransform_right.size()<5 || pointsToTransform_right.size()>200) pointsToTransform_right = last_pointsToTransform_right;
            else last_pointsToTransform_right = pointsToTransform_right;

            //左边线逆透视
            perspectiveTransform(pointsToTransform_left, transformedPoints, invMat_auto);
            transformedPoints_Left = convertCvPointsToPoints(transformedPoints);
            //右边线逆透视
            perspectiveTransform(pointsToTransform_right, transformedPoints, invMat_auto);
            transformedPoints_Right = convertCvPointsToPoints(transformedPoints);

            transformedPoints_Left = blur_points(transformedPoints_Left, using_kernel_num);   //三角滤波
            transformedPoints_Right = blur_points(transformedPoints_Right, using_kernel_num);
            transformedPoints_Left = resample_points(transformedPoints_Left, using_resample_dist*pixel_per_meter);  //下采样
            transformedPoints_Right = resample_points(transformedPoints_Right, using_resample_dist*pixel_per_meter);

            //画左边线
            transformedPoints = convertPointsToCvPoints(transformedPoints_Left);
            for (size_t i = 0; i < transformedPoints.size(); i++) 
            {
                circle(only_boundary, transformedPoints[i], 0, Scalar(255, 255, 255), 2);
            }
            // 画右边线
            transformedPoints = convertPointsToCvPoints(transformedPoints_Right);
            for (size_t i = 0; i < transformedPoints.size(); i++) 
            {
                circle(only_boundary, transformedPoints[i], 0, Scalar(255, 255, 255), 2);
            }

            //中线
            left_num=0, right_num=0;
            int bound = (track.pointsEdgeLeft.size()>100?(track.pointsEdgeLeft.size()-20):track.pointsEdgeLeft.size());
            if(track.pointsEdgeLeft.size()<20) ;
            else 
            {
                 for(int i=0;i<bound;i++)
                {
                    if(track.pointsEdgeLeft[i].y<=3) left_num = 0; else left_num ++;
                }
                for(int i=0;i<bound;i++)
                {
                    if(track.pointsEdgeRight[i].y>=COLSIMAGE-3) right_num = 0; else right_num ++;
                }
            }

            if(scene==Scene::RingScene && ringstep==4 && R100)
            {
                if(ringleft==1) centerEdge = track_leftline(transformedPoints_Left, 5, 0.225*pixel_per_meter);
                else centerEdge = track_rightline(transformedPoints_Right, 5, 0.225*pixel_per_meter); 
            }
            else
            {
                if(left_num >= right_num) centerEdge = track_leftline(transformedPoints_Left, 5, 0.225*pixel_per_meter);
                else centerEdge = track_rightline(transformedPoints_Right, 5, 0.225*pixel_per_meter);
            }
            speed_centeredge = centerEdge; //用作速度策略的中线

            // 找最近点(起始点中线归一化)
            POINT car(ROWSIMAGE-1, COLSIMAGE/2);
            float min_dist = 1e10;
            int begin_id = -1;
            for (int i = 0; i < centerEdge.size(); i++) {
                float dx = centerEdge[i].x - car.x;
                float dy = centerEdge[i].y - car.y;
                float dist = sqrt(dx * dx + dy * dy);
                if (dist < min_dist) {
                    min_dist = dist;
                    begin_id = i;
                }
            }
            centerEdge[begin_id].x = car.x;
            centerEdge[begin_id].y = car.y;
            centerEdge = resample_points(vector<POINT>(centerEdge.begin() + begin_id, centerEdge.end()), using_resample_dist*pixel_per_meter);  //中线下采样
            
            centerEdge1 = track_rightline(transformedPoints_Right, 5, 0.225*pixel_per_meter);
            centerEdge1[begin_id].x = car.x;
            centerEdge1[begin_id].y = car.y;
            centerEdge1 = resample_points(vector<POINT>(centerEdge1.begin() + begin_id, centerEdge1.end()), using_resample_dist*pixel_per_meter);  //中线下采样


            vector<Point2f> pointsToTransform_center = convertPointsToCvPoints(centerEdge);  //中线转为cvpoint
            perspectiveTransform(pointsToTransform_center, transformedPoints, inverse);  //透视
            centerEdge_show = convertCvPointsToPoints(transformedPoints);  //透视后中线转为POINT
            
            pointsToTransform_center = convertPointsToCvPoints(centerEdge1);  //中线转为cvpoint
            perspectiveTransform(pointsToTransform_center, transformedPoints, inverse);  //透视
            centerEdge_show1= convertCvPointsToPoints(transformedPoints);  //透视后中线转为POINT

            aim_idx = clip(round(aim_distance / using_resample_dist), 0, centerEdge.size() - 1);  //跟随点
            speed_aim_idx = clip(round(0.8 / using_resample_dist), 0, centerEdge.size() - 1);  //跟随点

            // 加权控制中心计算
            int controlNum = 1;

            int temp_point = calstartline;
            if(centerEdge[centerEdge.size() - 1].x > temp_point)
            {
                temp_point = centerEdge[centerEdge.size() - 1].x;
            }
            for (auto p : centerEdge)
            {
            if (p.x > temp_point && p.x < temp_point + 10)
            {
                controlNum += ROWSIMAGE / 2;
                controlCenter += p.y * ROWSIMAGE / 2;
            }
            }
            if (controlNum > 1)
            {
                controlCenter = controlCenter / controlNum;
            }
            
            if (speed_centeredge.size() > 10)
            {
                vector<POINT> centerV;
                int filt = speed_centeredge.size() / 5;
                for (int i = filt; i < speed_centeredge.size() - filt; i++) // 过滤中心点集前后1/5的诱导性
                {
                    centerV.push_back(speed_centeredge[i]);
                }
                sigmaCenter = sigma(centerV);
            }
            else
                sigmaCenter = 1000;


            // for (size_t i = 0; i < pointsToTransform_center.size(); i++)
            // {
            //     if(i==aim_idx) circle(only_boundary, pointsToTransform_center[i], 3, Scalar(255, 255, 255), 2);
            //     else circle(only_boundary, pointsToTransform_center[i], 0, Scalar(255, 255, 255), 2);
            // }
            // imshow("11", only_boundary);
            // waitKey(1);
        }
        else
        {
            POINT pp;
            for(int i=0;i<track.pointsEdgeLeft.size();i++)
            {
                pp.x = track.pointsEdgeLeft[i].x;
                pp.y = (track.pointsEdgeLeft[i].y + track.pointsEdgeRight[i].y)/2;
                centerEdge.push_back(pp);
                centerEdge_show.push_back(pp);
            }
            // 加权控制中心计算
            int controlNum = 1;

            int temp_point = calstartline;
            if(centerEdge[centerEdge.size() - 1].x > temp_point)
            {
                temp_point = centerEdge[centerEdge.size() - 1].x;
            }
            for (auto p : centerEdge)
            {
            if (p.x > temp_point && p.x < temp_point + 20)
            {
                controlNum += ROWSIMAGE / 2;
                controlCenter += p.y * ROWSIMAGE / 2;
            }
            }
            if (controlNum > 1)
            {
                controlCenter = controlCenter / controlNum;
            }

            if (controlCenter > COLSIMAGE)
                controlCenter = COLSIMAGE;
            else if (controlCenter < 0)
                controlCenter = 0;

            // 控制率计算
            if (centerEdge.size() > 20)
            {
                vector<POINT> centerV;
                int filt = centerEdge.size() / 5;
                for (int i = filt; i < centerEdge.size() - filt; i++) // 过滤中心点集前后1/5的诱导性
                {
                    centerV.push_back(centerEdge[i]);
                }
                sigmaCenter = sigma(centerV);
            }
            else
                sigmaCenter = 1000;            
       }
    }

    /**
     * @brief 车辆冲出赛道检测（保护车辆）
     *
     * @param track
     * @return true
     * @return false
     */
    bool derailmentCheck(Tracking track)
    {
        if (track.pointsEdgeLeft.size() < 30 && track.pointsEdgeRight.size() < 30) // 防止车辆冲出赛道
        {
            countOutlineA++;
            countOutlineB = 0;
            if (countOutlineA > 20)
                return true;
        }
        else
        {
            countOutlineB++;
            if (countOutlineB > 50)
            {
                countOutlineA = 0;
                countOutlineB = 50;
            }
        }
        return false;
    }

    /**
     * @brief 显示赛道线识别结果
     *
     * @param centerImage 需要叠加显示的图像
     */
    void drawImage(Tracking track, Mat &centerImage)
    {

         line(centerImage, cv::Point(160, 0), cv::Point(160, 239), cv::Scalar(0, 0, 0), 2);
        // // 赛道边缘绘制
        for (int i = 0; i < track.pointsEdgeLeft.size(); i++)
        {
            if(track.pointsEdgeLeft[i].x>=0 && track.pointsEdgeLeft[i].x<ROWSIMAGE && track.pointsEdgeLeft[i].y>=0 && track.pointsEdgeLeft[i].y<COLSIMAGE)
            {
            circle(centerImage, Point(track.pointsEdgeLeft[i].y, track.pointsEdgeLeft[i].x), 1, Scalar(0, 255, 0), -1); // 绿色点
            }
        }
        for (int i = 0; i < track.pointsEdgeRight.size(); i++)
        {
            if(track.pointsEdgeRight[i].x>=0 && track.pointsEdgeRight[i].x<ROWSIMAGE && track.pointsEdgeRight[i].y>=0 && track.pointsEdgeRight[i].y<COLSIMAGE)
            {
            circle(centerImage, Point(track.pointsEdgeRight[i].y, track.pointsEdgeRight[i].x), 1, Scalar(0, 255, 255), -1); // 黄色点
            }
        }

        // 绘制中心点集
        for (int i = 0; i < centerEdge.size(); i++)
        {
            if(centerEdge_show[i].x>=0 && centerEdge_show[i].x<ROWSIMAGE && centerEdge_show[i].y>=0 && centerEdge_show[i].y<COLSIMAGE)
            {
                if(i==aim_idx)  circle(centerImage, Point(centerEdge_show[i].y, centerEdge_show[i].x), 3, Scalar(0, 0, 255), -1);
                else circle(centerImage, Point(centerEdge_show[i].y, centerEdge_show[i].x), 1, Scalar(0, 0, 255), -1);
            }
            if(centerEdge_show1[i].x>=0 && centerEdge_show1[i].x<ROWSIMAGE && centerEdge_show1[i].y>=0 && centerEdge_show1[i].y<COLSIMAGE)
            {
                if(i==aim_idx)  circle(centerImage, Point(centerEdge_show1[i].y, centerEdge_show1[i].x), 3, Scalar(0, 0, 255), -1); 
                circle(centerImage, Point(centerEdge_show1[i].y, centerEdge_show1[i].x), 1, Scalar(0, 0, 255), -1);
            }
        }

        // 绘制加权控制中心：方向
        Rect rect(controlCenter, ROWSIMAGE - 20, 10, 20);
        rectangle(centerImage, rect, Scalar(0, 0, 255), CV_FILLED);

        // 详细控制参数显示
        int dis = 20;
        string str;
        putText(centerImage, style, Point(COLSIMAGE - 60, dis), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 1); // 赛道类型

        str = "Edge: " + formatDoble2String(track.stdevLeft, 1) + " | " + formatDoble2String(track.stdevRight, 1);
        putText(centerImage, str, Point(COLSIMAGE - 150, 2 * dis), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 1); // 斜率：左|右

        str = "Center: " + formatDoble2String(sigmaCenter, 2);
        putText(centerImage, str, Point(COLSIMAGE - 120, 3 * dis), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 1); // 中心点方差

        putText(centerImage, to_string(controlCenter), Point(COLSIMAGE / 2 - 10, ROWSIMAGE - 40), FONT_HERSHEY_PLAIN, 1.2, Scalar(0, 0, 255), 1); // 中心
    }

private:
    int countOutlineA = 0; // 车辆脱轨检测计数器
    int countOutlineB = 0; // 车辆脱轨检测计数器
    string style = "";     // 赛道类型
    /**
     * @brief 搜索十字赛道突变行（左下）
     *
     * @param pointsEdgeLeft
     * @return uint16_t
     */
    uint16_t searchBreakLeftDown(vector<POINT> pointsEdgeLeft)
    {
        uint16_t counter = 0;

        for (int i = 0; i < pointsEdgeLeft.size() - 10; i++)
        {
            if (pointsEdgeLeft[i].y >= 2)
            {
                counter++;
                if (counter > 3)
                {
                    return i - 2;
                }
            }
            else
                counter = 0;
        }

        return 0;
    }

    /**
     * @brief 搜索十字赛道突变行（右下）
     *
     * @param pointsEdgeRight
     * @return uint16_t
     */
    uint16_t searchBreakRightDown(vector<POINT> pointsEdgeRight)
    {
        uint16_t counter = 0;

        for (int i = 0; i < pointsEdgeRight.size() - 10; i++) // 寻找左边跳变点
        {
            if (pointsEdgeRight[i].y < COLSIMAGE - 2)
            {
                counter++;
                if (counter > 3)
                {
                    return i - 2;
                }
            }
            else
                counter = 0;
        }

        return 0;
    }

    /**
     * @brief 赛道中心点计算：单边控制
     *
     * @param pointsEdge 赛道边缘点集
     * @param side 单边类型：左边0/右边1
     * @return vector<POINT>
     */
    vector<POINT> centerCompute(vector<POINT> pointsEdge, int side)
    {
        int step = 4;                    // 间隔尺度
        int offsetWidth = COLSIMAGE / 2; // 首行偏移量
        int offsetHeight = 0;            // 纵向偏移量

        vector<POINT> center; // 控制中心集合

        if (side == 0) // 左边缘
        {
            uint16_t counter = 0, rowStart = 0;
            for (int i = 0; i < pointsEdge.size(); i++) // 删除底部无效行
            {
                if (pointsEdge[i].y > 1)
                {
                    counter++;
                    if (counter > 2)
                    {
                        rowStart = i - 2;
                        break;
                    }
                }
                else
                    counter = 0;
            }

            offsetHeight = pointsEdge[rowStart].x - pointsEdge[0].x;
            counter = 0;
            for (int i = rowStart; i < pointsEdge.size(); i += step)
            {
                int py = pointsEdge[i].y + offsetWidth;
                if (py > COLSIMAGE - 1)
                {
                    counter++;
                    if (counter > 2)
                        break;
                }
                else
                {
                    counter = 0;
                    center.emplace_back(pointsEdge[i].x - offsetHeight, py);
                }
            }
        }
        else if (side == 1) // 右边沿
        {
            uint16_t counter = 0, rowStart = 0;
            for (int i = 0; i < pointsEdge.size(); i++) // 删除底部无效行
            {
                if (pointsEdge[i].y < COLSIMAGE - 1)
                {
                    counter++;
                    if (counter > 2)
                    {
                        rowStart = i - 2;
                        break;
                    }
                }
                else
                    counter = 0;
            }

            offsetHeight = pointsEdge[rowStart].x - pointsEdge[0].x;
            counter = 0;
            for (int i = rowStart; i < pointsEdge.size(); i += step)
            {
                int py = pointsEdge[i].y - offsetWidth;
                if (py < 1)
                {
                    counter++;
                    if (counter > 2)
                        break;
                }
                else
                {
                    counter = 0;
                    center.emplace_back(pointsEdge[i].x - offsetHeight, py);
                }
            }
        }

        return center;
        // return Bezier(0.2,center);
    }
};