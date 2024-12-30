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
 * @file ring.cpp
 * @author Leo
 * @brief 环岛识别（基于track赛道识别后）
 * @version 0.1
 * @date 2022-02-28
 *
 * @copyright Copyright (c) 2022
 *
 * @note  环岛识别步骤（ringStep）：
 *          1：环岛识别（初始化）
 *          2：入环处理
 *          3：环中处理
 *          4：出环处理
 *          5：出环结束
 */

#include <fstream>
#include <iostream>
#include <cmath>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "../../include/common.hpp"
#include "tracking.cpp"

using namespace cv;
using namespace std;



class Ring
{
public:

 enum RingStep
    {
        None = 0, // 未知类型
        Verifing,
        Waiting,        
        Entering, // 入环
        Inside,   // 环中
        Exiting,  // 出环
        Finish    // 环任务结束
    };
    RingStep ringStep = RingStep::None;     // 环岛处理阶段
    /**
     * @brief 环岛类型
     *
     */
    enum RingType
    {
        RingNone = 0, // 未知类型
        RingLeft,     // 左入环岛
        RingRight     // 右入环岛
    };

    RingType ringType = RingType::RingLeft; // 环岛类型
    uint8_t ringNum = 0;
    uint16_t counterShield = 0; // 环岛检测屏蔽计数器：屏蔽车库误检测
    bool R100 = false;

    /**
     * @brief 环岛识别初始化|复位
     *
     */
    void reset(void)
    {
        RingType ringType = RingType::RingLeft; // 环岛类型
        RingStep ringStep = RingStep::None;     // 环岛处理阶段
        int rowRepairLine = 0;                  // 用于环补线的点（行号）
        int colRepairLine = 0;                  // 用于环补线的点（列号）
        counterSpurroad = 0;                    // 岔路计数器
        counterShield = 0;
    }
    /**
     * @brief 环岛识别与行径规划
     *
     * @param track 基础赛道识别结果
     * @param imagePath 赛道路径图像
     */
    bool ringRecognition(Tracking &track, Mat &imagePath, uint8_t ringSum, uint8_t *ringEnter, vector<int> RoadWidth, float speed, uint32_t frametime)
  {
    // cout << ringStep << endl;
    if (ringStep == RingStep::None)
    {
      if (track.stdevRight <15 && track.stdevLeft >20) // 判左环
      {
        uint8_t right_sum = 0;  //右边线数目
        uint8_t left_cnt = 0;  //判断增减趋势
        uint8_t left_state = 0;  //0:增, 1:减
        uint8_t left_edge = 0;  //左丢线数目
        int cnt[3]={0,0,0};
        for (int i = 2; i < track.widthBlock.size(); i += 2)
        {
          if (track.pointsEdgeRight[i].y <= track.pointsEdgeRight[i - 2].y && track.pointsEdgeRight[i-2].y<COLSIMAGE-3)
          {
            right_sum ++;
          }
          else right_sum=0;

          if(left_state==0)
          {
            if (track.pointsEdgeLeft[i].y <= 3)  //丢线
            {
              cnt[0] ++; //25
            }
            if(track.pointsEdgeLeft[i-2].y <= 3 && track.pointsEdgeLeft[i].y <= 20 && track.pointsEdgeLeft[i].y > 3) {left_state++;}
          }
          else if(left_state==1)
          {
            if (track.pointsEdgeLeft[i].y > track.pointsEdgeLeft[i - 2].y + 1)  //左边线向中间增
            {
              cnt[1] ++; //5
            }
            if(track.pointsEdgeLeft[i].y < track.pointsEdgeLeft[i-2].y - 1 ) {left_state++;}
          }
          else if(left_state==2)
          {
            if (track.pointsEdgeLeft[i].y < track.pointsEdgeLeft[i-2].y - 1)  //左边线向中间增
            {
              cnt[2] ++;
            }
          }
        } 
        // printf("%d %d %d %d\n", left_state,left_cnt,right_sum,left_edge );
        printf("%d %d %d %d\n", cnt[0],cnt[1],cnt[2],left_edge);
       // params[0]=left_state;params[1]=left_cnt;params[2]=right_sum;params[3]=left_edge;
        if (right_sum > 60 && ((cnt[0]>5&& cnt[0]<20 && cnt[1]>10 && cnt[1]<45 && cnt[2]>0&&track.stdevLeft<70)||(cnt[0]>25 && cnt[1]>10 && cnt[2]>3&&track.stdevLeft>80)))
        {
          if((cnt[0]>5 && cnt[0]<20 && cnt[1]>10 && cnt[1]<45 && cnt[2]>0&&track.stdevLeft<70)) R100 = true;
          else R100 = false;
          ringStep = RingStep::Verifing;
          ringType = RingType::RingLeft;
        }
      }


      if (track.stdevLeft <15 && track.stdevRight >20) // 判右环
      {
        uint8_t left_sum = 0;
        uint8_t right_cnt = 0;
        uint8_t right_state = 0;
        uint8_t right_edge = 0;
        int cnt[3]={0,0,0};
        for (int i = 2; i < track.widthBlock.size(); i += 2)
        {
        //  if(track.pointsEdgeLeft[i].y - track.pointsEdgeLeft[i-2].y > 10) {left_sum=0;break;}

          if (track.pointsEdgeLeft[i].y >= track.pointsEdgeLeft[i - 2].y && track.pointsEdgeLeft[i].y>3)
          {
            left_sum ++;
          }
          else left_sum=0;
          
          if(right_state==0)
          {
            if (track.pointsEdgeRight[i].y >= COLSIMAGE-3)  //丢线
            {
              cnt[0] ++;
            }
            if(track.pointsEdgeRight[i-2].y >= COLSIMAGE-3 && track.pointsEdgeRight[i].y >= COLSIMAGE-20 && track.pointsEdgeRight[i].y < COLSIMAGE-3) {right_state++;}
          }
          else if(right_state==1)
          {
            if (track.pointsEdgeRight[i].y < track.pointsEdgeRight[i - 2].y - 1)  //右边线向中间增
            {
              cnt[1] ++; 
            }
            if(track.pointsEdgeRight[i].y > track.pointsEdgeRight[i - 2].y + 1) {right_state++;}
          }
          else if(right_state==2)
          {
             if (track.pointsEdgeRight[i].y > track.pointsEdgeRight[i - 2].y + 1)  //右边线向右边减
            {
              cnt[2] ++;
            }
          }
          
        }
        
 //       printf("%d %d %d %d %d\n",right_cnt, right_state,right_cnt,left_sum,right_edge );
  // params[0]=right_state;params[1]=right_cnt;params[2]=left_sum;params[3]=right_edge;
        if (left_sum > 60 && ((cnt[0]>5 && cnt[0]<20 && cnt[1]>10 && cnt[1]<45 && cnt[2]>0&&track.stdevRight<70)||(cnt[0]>25 && cnt[1]>10 && cnt[2]>3&&track.stdevRight>80)))
        {
          if((cnt[0]>5 && cnt[0]<20 && cnt[1]>10 && cnt[1]<45 && cnt[2]>0&&track.stdevRight<70)) R100 = true;
          else R100 = false;
          ringStep = RingStep::Verifing;
          ringType = RingType::RingRight;
        }
      }
    }

    else if(ringStep == RingStep::Verifing)
    {
      if(ringType == RingType::RingLeft)
      {
        if (track.stdevRight >15)
        {
          ringStep=RingStep::None;
          return false;
        }
        int fixline = 0;
        int state = 0;
        int cnt[3]={0,0,0};
        for(int i=2;i<track.pointsEdgeLeft.size(); i++)
        {
          if(state==0)
          {
             if (track.pointsEdgeLeft[i].y > track.pointsEdgeLeft[i - 2].y)  //左边线向中间增
             {
                cnt[0]++;
             }
             if(track.pointsEdgeLeft[i].y < track.pointsEdgeLeft[i - 2].y) state ++;
          }  
          if(state==1)
          {
            if(track.pointsEdgeLeft[i].y < track.pointsEdgeLeft[i - 2].y)//减 //3
            {
              cnt[1]++;
            }
            if(track.pointsEdgeLeft[i].y<=3) state ++;
          }
          if(state==2)//丢线
          {
            if(track.pointsEdgeLeft[i].y<=3) cnt[2]++;
          }
        }
        printf("%d %d %d\n", cnt[0],cnt[1],cnt[2]);
        if(!R100 && cnt[0]>5 && cnt[1]>3 && cnt[2]>=2) ringStep = RingStep::Waiting;
        if(R100 && cnt[0]>40 && cnt[1]>20 && cnt[2]>=1) ringStep = RingStep::Inside;

        if (ringStep == RingStep::Verifing)
        {
          for(int i=0;i<track.pointsEdgeLeft.size();i++)
          {
            track.pointsEdgeLeft[i].y = track.pointsEdgeRight[i].y - RoadWidth[i];
          }
        }
      }
      else if(ringType==RingType::RingRight)
      {
         if (track.stdevLeft >15)
        {
          ringStep=RingStep::None;
          return false;
        }
        int fixline = 0;
        int state = 0;
        int cnt[3]={0,0,0};
        for(int i=2;i<track.pointsEdgeRight.size(); i++)
        {
          if(state==0)
          {
             if (track.pointsEdgeRight[i].y < track.pointsEdgeRight[i - 2].y)  //右边线向中间减
             {
                cnt[0]++;
             }
             if(track.pointsEdgeRight[i].y > track.pointsEdgeRight[i - 2].y) state ++;
          }  
         if(state==1)
          {
            if(track.pointsEdgeRight[i].y > track.pointsEdgeRight[i - 2].y)
            {
              cnt[1]++;
            }
            if(track.pointsEdgeRight[i].y>=COLSIMAGE-3) state ++;
          }
          if(state==2)
          {
            if(track.pointsEdgeRight[i].y>=COLSIMAGE-3) cnt[2]++;
          }
        }
        printf("%d %d %d\n", cnt[0],cnt[1],cnt[2]);
        if(!R100 && cnt[0]>5 && cnt[1]>3 && cnt[2]>=2) ringStep = RingStep::Waiting;
        if(R100 && cnt[0]>40 && cnt[1]>20 && cnt[2]>=1) ringStep = RingStep::Inside;

        if (ringStep == RingStep::Verifing)
        {
          for(int i=0;i<track.pointsEdgeRight.size();i++)
          {
            track.pointsEdgeRight[i].y = track.pointsEdgeLeft[i].y + RoadWidth[i];
          }
        }
      }
    }

    else if (ringStep == RingStep::Waiting)
    {
      if (ringType == RingType::RingLeft)
      {
         if (track.stdevRight >15)
        {
          ringStep=RingStep::None;
          return false;
        }
        int fixline = 0;
        int whitecnt = 0;
        bool search = true;
        POINT endpoint = POINT(0, 0);

        for (int i = 0; i < track.pointsEdgeLeft.size(); i ++)
        {
          if (track.pointsEdgeLeft[i].y < 200 && track.pointsEdgeLeft[i].y > track.pointsEdgeLeft[fixline].y) fixline = i;
          if (search && whitecnt > 5 && track.pointsEdgeLeft[i].y >= 3) search = false;
          if (search && track.pointsEdgeLeft[i].x < 85 && track.pointsEdgeLeft[i].y < 3)
          {
            endpoint = track.pointsEdgeLeft[i];
            whitecnt ++;
          }
        }
        
        if (!search && endpoint.x > ringEnter[ringNum])
        {
          ringStep = RingStep::Entering;
          EnteringCounter = 0;
        }
        
        if (ringStep == RingStep::Waiting)
        {
          for(int i=0;i<track.pointsEdgeLeft.size();i++)
          {
            track.pointsEdgeLeft[i].y = track.pointsEdgeRight[i].y - RoadWidth[i];
          }
        }
      }
      else if (ringType == RingType::RingRight)
      {
         if (track.stdevLeft >15)
        {
          ringStep=RingStep::None;
          return false;
        }
        int fixline = 0;
        int whitecnt = 0;
        bool search = true;
        POINT endpoint = POINT(0, COLSIMAGE - 1);

        for (int i = 0; i < track.pointsEdgeRight.size(); i ++)
        {
          if (track.pointsEdgeRight[i].y >= 120 && track.pointsEdgeRight[i].y < track.pointsEdgeRight[fixline].y) fixline = i;
          if (search && whitecnt > 5 && track.pointsEdgeRight[i].y <= COLSIMAGE - 4) search = false;
          if (search && track.pointsEdgeRight[i].x < 85 && track.pointsEdgeRight[i].y > COLSIMAGE - 4)
          {
            endpoint = track.pointsEdgeRight[i];
            whitecnt ++;
          }
        }
        
        if (!search && endpoint.x > ringEnter[ringNum])
        {
          ringStep = RingStep::Entering;
          EnteringCounter = 0;
        }

        if (ringStep == RingStep::Waiting)
        {
          for(int i=0;i<track.pointsEdgeRight.size();i++)
          {
            track.pointsEdgeRight[i].y = track.pointsEdgeLeft[i].y + RoadWidth[i];
          }
        }
      }
    }
    else if (ringStep == RingStep::Entering)
    {
      if (ringType == RingType::RingLeft)
      {
   //     EnteringCounter ++;  //入环计数器
    //   printf("%d %d\n", track.pointsEdgeRight.size(), track.pointsEdgeRight[10].y);
        if ((track.pointsEdgeRight.size() < 180 && track.pointsEdgeRight[10].y < COLSIMAGE - 20)/* || track.pointsEdgeRight.size() < 140*//*|| EnteringCounter > 40*/)
        {
          ringStep = RingStep::Inside;
        }

        //补线
        if (ringStep == RingStep::Entering)
        {
          int whitecnt = 0;
          POINT endpoint = POINT(0, 0);
          for (int i = 0; i < track.pointsEdgeLeft.size(); i ++)  
          {
            if (whitecnt > 50 && track.pointsEdgeLeft[i].y >= 3) break;
            if (track.pointsEdgeLeft[i].x < 160 && track.pointsEdgeLeft[i].y < 3)
            {
              endpoint = track.pointsEdgeLeft[i];
              whitecnt ++;
            }
          }
            POINT start = POINT(ROWSIMAGE - 31, COLSIMAGE - 31);
            POINT end = POINT(endpoint.x, endpoint.y);
          //  POINT end = POINT(track.pointsEdgeLeft[i].x, track.pointsEdgeLeft[i].y);
            POINT middle = POINT((start.x + end.x) * 0.3, (start.y + end.y) * 0.7);
            vector<POINT> input = {start, middle, end};
            track.pointsEdgeRight = Bezier(0.02, input);
            
            track.pointsEdgeLeft.clear();
            for (int i = 0; i < track.pointsEdgeRight.size(); i ++)
            {
              track.pointsEdgeLeft.push_back(POINT(track.pointsEdgeRight[i].x, 0));
            }
        }
      }
      else if (ringType == RingType::RingRight)
      {
       // EnteringCounter ++;
    //    printf("%d %d\n", track.pointsEdgeLeft.size(), track.pointsEdgeLeft[10].y);
        if ((track.pointsEdgeLeft.size() < 180 && track.pointsEdgeLeft[10].y > 20)/*|| track.pointsEdgeLeft.size() < 140*//* || EnteringCounter > 40*/)
        {
          ringStep = RingStep::Inside;
        }
        
        if (ringStep == RingStep::Entering)
        {
          int whitecnt = 0;
          POINT endpoint = POINT(0, COLSIMAGE - 1);
  
          for (int i = 0; i < track.pointsEdgeRight.size(); i ++)
          {
            if (whitecnt > 50 && track.pointsEdgeRight[i].y <= COLSIMAGE - 4) break;
            if (track.pointsEdgeRight[i].x < 160 && track.pointsEdgeRight[i].y > COLSIMAGE - 4)
            {
              endpoint = track.pointsEdgeRight[i];
              whitecnt ++;
            }
          }
          
          POINT start = POINT(ROWSIMAGE - 31, 30);
          POINT end = POINT(endpoint.x, endpoint.y);
          POINT middle = POINT((start.x + end.x) * 0.3, (start.y + end.y) * 0.3);
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
    else if (ringStep == RingStep::Inside)
    {
    //  printf("Inside\n");
      if (ringType == RingType::RingLeft)
      {
        if(R100)
        {
          static bool In = false;
          if(!In && track.stdevRight >28) In = true;
          if(In && track.stdevRight > 1 && track.stdevRight < 10)
          {
            In = false;
            ringStep = RingStep::None;
            ringType = RingType::RingNone;
            ringNum = (ringNum + 1) % ringSum;
          }
        }
        else
        {
          uint8_t right_cnt = 0;
          uint8_t right_state = 0;

          for (int i = 2; i < track.pointsEdgeRight.size(); i += 2)
          {
            if (!right_state)
            {
              if (track.pointsEdgeRight[i].y > track.pointsEdgeRight[i - 2].y)  
              {
                right_cnt ++;
                right_state = 1;
              }
            }
            else
            {
              if (track.pointsEdgeRight[i].y > track.pointsEdgeRight[i - 2].y ||
                  track.pointsEdgeRight[i].y > COLSIMAGE - 5)
              {
                right_cnt ++;
              }
            }
          }
          if (right_cnt > 20)
          {
            ringStep = RingStep::Exiting;
          }
        }
    //    printf("%d %d\n",right_state,  right_cnt);
      }
      else if (ringType == RingType::RingRight)
      {
        if(R100)
        {
          static bool In = false;
          if(!In && track.stdevLeft>28) In = true;
          if(In && track.stdevLeft > 1 && track.stdevLeft < 10)
          {
            In = false;
            ringStep = RingStep::None;
            ringType = RingType::RingNone;
            ringNum = (ringNum + 1) % ringSum;
          }
        }
        else
        {
          uint8_t left_cnt = 0;
          uint8_t left_state = 0;

          for (int i = 2; i < track.pointsEdgeLeft.size(); i += 2)
          {
            if (!left_state)
            {
              if (track.pointsEdgeLeft[i].y < track.pointsEdgeLeft[i - 2].y)
              {
                left_cnt ++;
                left_state = 1;
              }
            }
            else
            {
              if (track.pointsEdgeLeft[i].y < track.pointsEdgeLeft[i - 2].y ||
                  track.pointsEdgeLeft[i].y < 4)
              {
                left_cnt ++;
              }
            }
          }
          if (left_cnt > 20)
          {
            ringStep = RingStep::Exiting;
          }
        }
    //    printf("%d %d\n",left_state,  left_cnt);
      }
    }
    else if (ringStep == RingStep::Exiting)
    {
   //   printf("Exiting\n");
      if (ringType == RingType::RingLeft)
      {
        int whitecnt = 0;
        int stage=0;
        bool left_leave=false;
        for (int i = 2; i < track.pointsEdgeRight.size(); i += 2)
        {
          if (track.pointsEdgeRight[i].y > COLSIMAGE - 4) whitecnt ++; //右丢线
          if(stage==0&&track.pointsEdgeRight[i].y<track.pointsEdgeRight[i-2].y) stage++;  //
          if(stage==1&&track.pointsEdgeRight[i].y>track.pointsEdgeRight[i-2].y) {stage++;break;}
        }
        for(int i = 2; i < track.pointsEdgeLeft.size(); i += 2)
        {
          if(!left_leave && track.pointsEdgeLeft[i].y>3 && track.pointsEdgeLeft[i-2].y<3) left_leave=true;
        }

        if (stage==1 && track.stdevRight < 50 && track.stdevRight > 0 && (whitecnt < 10 || left_leave))
        {
          ringStep = RingStep::None;
          ringType = RingType::RingNone;
          ringNum = (ringNum + 1) % ringSum;
        }
        
        if (ringStep == RingStep::Exiting)
        {
          POINT endpoint = POINT(120, 0); // tag
          for (int i = track.pointsEdgeLeft.size() - 1; i >= 0; i --)
          {
            if (track.pointsEdgeLeft[i].y < 3)
            {
              endpoint.x = track.pointsEdgeLeft[i].x;
              break;
            }
          }
          
          POINT start = POINT(ROWSIMAGE - 30, COLSIMAGE - 31); // tag
          POINT end = POINT(endpoint.x, endpoint.y);
          POINT middle = POINT((start.x + end.x) * 0.4, (start.y + end.y) * 0.6);
          vector<POINT> input = {start, middle, end};
          track.pointsEdgeRight = Bezier(0.02, input);
          
          track.pointsEdgeLeft.clear();
          for (int i = 0; i < track.pointsEdgeRight.size(); i ++)
          {
            track.pointsEdgeLeft.push_back(POINT(track.pointsEdgeRight[i].x, 0));
          }
        }
      }
      else if (ringType == RingType::RingRight)
      {
        int whitecnt = 0;
        int stage=0;
        int right_leave=false;
        for (int i = 2; i < track.pointsEdgeLeft.size(); i += 2)
        {
          if (track.pointsEdgeLeft[i].y < 3)
            whitecnt ++;
            if(stage==0&&track.pointsEdgeLeft[i].y>track.pointsEdgeLeft[i-2].y) stage++;
            if(stage==1&&track.pointsEdgeLeft[i].y<track.pointsEdgeLeft[i-2].y){stage++;break;}
        }
        
        for(int i = 2; i < track.pointsEdgeRight.size(); i += 2)
        {
          if(!right_leave && track.pointsEdgeRight[i].y<COLSIMAGE-3 && track.pointsEdgeRight[i-2].y>COLSIMAGE-3) right_leave=true;
        }

        if (stage==1 &&track.stdevLeft < 50 && track.stdevLeft > 0 && (whitecnt < 10 || right_leave))
        {
          ringStep = RingStep::None;
          ringType = RingType::RingNone;
          ringNum = (ringNum + 1) % ringSum;
        }

        if (ringStep == RingStep::Exiting)
        {
          POINT endpoint = POINT(120, COLSIMAGE - 1); // tag
          for (int i = track.pointsEdgeRight.size() - 1; i >= 0; i --)
          {
            if (track.pointsEdgeRight[i].y > COLSIMAGE - 4)
            {
              endpoint.x = track.pointsEdgeRight[i].x;
              break;
            }
          }
          
          POINT start = POINT(ROWSIMAGE - 30, 30); // tag
          POINT end = POINT(endpoint.x, endpoint.y);
          POINT middle = POINT((start.x + end.x)*0.4, (start.y+end.y)*0.4);
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

    if (ringStep == RingStep::None)
      return false;
    else
    {
//      printf("-----------环岛 %d %d %d\n", ringStep, track.pointsEdgeRight.size(), track.pointsEdgeRight[10].y);
       return true;
    }
  }
  
    /**
     * @brief 绘制环岛识别图像
     *
     * @param ringImage 需要叠加显示的图像
     */
    void drawImage(Tracking track, Mat &ringImage)
    {
        // for (int i = 0; i < track.pointsEdgeLeft.size(); i++)
        // {
        //     circle(ringImage, Point(track.pointsEdgeLeft[i].y, track.pointsEdgeLeft[i].x), 2,
        //            Scalar(0, 255, 0), -1); // 绿色点
        // }
        // for (int i = 0; i < track.pointsEdgeRight.size(); i++)
        // {
        //     circle(ringImage, Point(track.pointsEdgeRight[i].y, track.pointsEdgeRight[i].x), 2,
        //            Scalar(0, 255, 255), -1); // 黄色点
        // }

        // for (int i = 0; i < track.spurroad.size(); i++)
        // {
        //     circle(ringImage, Point(track.spurroad[i].y, track.spurroad[i].x), 5,
        //            Scalar(0, 0, 255), -1); // 红色点
        // }

        // putText(ringImage, to_string(_ringStep) + " " + to_string(_ringEnable) + " " + to_string(_tmp_ttttt),
        //         Point(COLSIMAGE - 80, ROWSIMAGE - 20), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 0, 255), 1, CV_AA);

        // putText(ringImage, to_string(_index), Point(80, ROWSIMAGE - 20), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 0, 255), 1, CV_AA);

        // putText(ringImage, to_string(track.validRowsRight) + " | " + to_string(track.stdevRight),
        //         Point(COLSIMAGE - 100, ROWSIMAGE - 50),
        //         FONT_HERSHEY_TRIPLEX, 0.3, Scalar(0, 0, 255), 1, CV_AA);
        // putText(ringImage, to_string(track.validRowsLeft) + " | " + to_string(track.stdevLeft),
        //         Point(30, ROWSIMAGE - 50), FONT_HERSHEY_TRIPLEX, 0.3, Scalar(0, 0, 255), 1, CV_AA);

        // putText(ringImage, "[7] RING - ENABLE", Point(COLSIMAGE / 2 - 30, 10), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA);
        // circle(ringImage, Point(_ringPoint.y, _ringPoint.x), 4, Scalar(255, 0, 0), -1); // 红色点
    }

private:
    float distance=0; 
    uint16_t counterSpurroad = 0; // 岔路计数器
    // 临时测试用参数
    int _ringStep;
    int _ringEnable;
    int _tmp_ttttt;
    int _index = 0;
    POINT _ringPoint = POINT(0, 0);

    /**
     * @brief 环岛运行步骤/阶段
     *
     */

    int rowRepairLine = 0;                  // 用于环补线的点（行号）
    int colRepairLine = 0;                  // 用于环补线的点（列号）

      uint8_t EnteringCounter = 0;
};
