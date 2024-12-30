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
 * @file motion.cpp
 * @author Leo
 * @brief 运动控制器：PD姿态控制||速度控制
 * @version 0.1
 * @date 2023-12-26
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <fstream>
#include <iostream>
#include <cmath>
#include "../include/common.hpp"
#include "../include/json.hpp"
#include "controlcenter.cpp"
#include "./recognition/ring.cpp"
#include "./recognition/tracking.cpp"

using namespace std;
using namespace cv;

/**
 * @brief 运动控制器
 *
 */
class Motion
{
private:
    int countShift = 0; // 变速计数器

public:
    /**
     * @brief 初始化：加载配置文件
     *
     */
    Motion()
    {
        string jsonPath = "../src/config/config.json";
        std::ifstream config_is(jsonPath);
        if (!config_is.good())
        {
            std::cout << "Error: Params file path:[" << jsonPath
                      << "] not find .\n";
            exit(-1);
        }

        nlohmann::json js_value;
        config_is >> js_value;

        try
        {
            params = js_value.get<Params>();
        }
        catch (const nlohmann::detail::exception &e)
        {
            std::cerr << "Json Params Parse failed :" << e.what() << '\n';
            exit(-1);
        }

        speed = params.speedRing;
        cout << "--- NormP:" << params.NormP << " | NromD:" << params.NormD << endl;
     //   cout << "--- turnP:" << params.turnP << " | turnD:" << params.turnD << endl;
        cout << "--- speedRing:" << params.speedRing << "m/s  |  speedRescue:" << params.speedRescue << "m/s" << endl;
    };

    /**
     * @brief 控制器核心参数
     *
     */
    struct Params
    {
        float speedNorm = 0.8;   //寻常道路速度
        float speedTurn_Left = 1.9;
        float speedTurn_Right = 1.9;
        float speedRing = 0.8;                              // 智能车最低速
        float speedRescue = 0.8;                             // 智能车最高速
        float speedBridge = 0.6;                           // 坡道速度
        float speedDanger = 0.3;                             // 特殊区域降速速度
        float speedSafe = 0.6;
        float speedSpy1 = 0.6;
        float speedSpy2 = 0.6;
        float speedRacing_Dan = 0.6;
        
        float speedBase = 3.0;
        float DecreaseRatio = 0.007;

        float a = -0.0001;
        float x0 = 20;
        float c = 2.4;

        float pl = 0.01;
        float ph = 4;

        float pd_P=0;
        float pd_D=0;

        float alpha=0.5;

        float left_scale = 1;
        float right_scale = 1;

        float NormP = 1.8;                                  //直道P
        float NormD = 4.0;                                  //直道D

        float Danger_Start_P = 1.7;
        float Danger_Start_D = 3.3;
        float DangerP1 = 8.0;
        float DangerD1 = 16.0;
        float DangerP2 = 4.0;
        float DangerD2 = 15.0; 
        int DisLeaving = 30;
        float UpScale = 0.15;
        float block_scale=2;
        float distance_block=50;
        uint8_t DangerMode = 0;
        
        float WaitP = 3.2;  
        float WaitD = 16.0;  
        float RescueP = 3;
        float RescueD = 6;
        float dis_waiting = 0.2;
        float dis_entering = 0.25;
        float Cone_Scale_Down = 0.5;
        int RescueMode = 0;

        float SafeP = 3;
        float SafeD = 6;
        float SpyP = 3;
        float SpyD = 6;
        float Racing_DanP = 3;
        float Racing_DanD = 6;

        bool debug = false;                                // 调试模式使能
        bool saveImg = false;                              // 存图使能
        uint16_t rowCutUp = 10;                            // 图像顶部切行
        uint16_t rowCutBottom = 10;                        // 图像顶部切行

        uint8_t ringSum = 0;
        uint8_t ringEnter0 = 0;
        float ringP0 = 0.0;
        float ringD0 = 0.0;
        uint8_t ringEnter1 = 0;
        float ringP1 = 0.0;
        float ringD1 = 0.0;
        uint8_t ringEnter2 = 0;
        float ringP2 = 0.0;
        float ringD2 = 0.0;
        uint8_t ringEnter3 = 0;
        float ringP3 = 0.0;
        float ringD3 = 0.0;      

        float aim_distance = 0.6; 
        
        uint8_t whitelinethres = 180;   //最长白列限制
        uint8_t calstartline = 100;  //计算起始行
        uint8_t startline_danger = 100;
         
        bool bridge = true;                                // 坡道区使能
        bool danger = true;                                // 危险区使能
        bool rescue = true;                                // 救援区使能
        bool racing = true;                                // 追逐区使能
        bool parking = true;                               // 停车区使能
        bool ring = true;                                  // 环岛使能
        bool cross = true;                                 // 十字道路使能
        float score = 0.5;                                 // AI检测置信度
        string model = "../res/model/yolov3_mobilenet_v1"; // 模型路径
        string video = "../res/samples/demo.mp4";          // 视频路径

        NLOHMANN_DEFINE_TYPE_INTRUSIVE(Params, 
                speedNorm, speedTurn_Left, speedTurn_Right, speedRing, speedRescue, speedBridge, speedDanger, speedSafe, speedSpy1, speedSpy2, speedRacing_Dan, 
                speedBase, DecreaseRatio, a, x0, c,
                pl, ph, pd_P, pd_D, alpha, 
                NormP, NormD, 
                Danger_Start_P, Danger_Start_D,DangerP1, DangerD1, DangerP2, DangerD2, DisLeaving, UpScale, block_scale, distance_block, DangerMode,
                WaitP, WaitD, RescueP, RescueD, dis_waiting, dis_entering, Cone_Scale_Down, RescueMode, 
                SafeP, SafeD, SpyP, SpyD, Racing_DanP, Racing_DanD,
                ringSum, ringEnter0, ringP0, ringEnter1, ringP1, ringEnter2, ringP2, ringEnter3, ringP3,
                debug, saveImg, rowCutUp, rowCutBottom, aim_distance, whitelinethres, calstartline, startline_danger,
                bridge, danger, rescue, racing, parking, ring, cross, score, model, video); // 添加构造函数
    };

    Params params;                   // 读取控制参数
    uint16_t servoPwm = PWMSERVOMID; // 发送给舵机的PWM
    float speed = 0.3;               // 发送给电机的速度
    float pure_angle, pure_angleLast=0;
    float pure_angle_1;
    float err, last_err=0;

    float K_Centerline_Sum = 0;
    int Centerline_Turn_Row = 0;
    float speed_pure_angle = 0;
    float speed_max=0,real_angle_zero=0,speed_error=0;
    /**
     * @brief 姿态PD控制器
     *
     * @param controlCenter 智能车控制中心
     */
    void poseCtrl(ControlCenter ctrlCenter, float P, float D, Scene scene, Tracking tracking, float pd_P, float pd_D, float alpha)
    {
        if(scene==Scene::NormalScene || scene==Scene::CrossScene || scene==Scene::ParkingScene || scene==Scene::RingScene || scene==Scene::RescueScene)
        {
            // 纯跟随
            POINT car(ROWSIMAGE-1, COLSIMAGE/2-5);
             // 计算远锚点偏差值 delta
            float dy = ctrlCenter.centerEdge[ctrlCenter.aim_idx].x - car.x;
            float dx = car.y - ctrlCenter.centerEdge[ctrlCenter.aim_idx].y;
            float dn = sqrt(dx * dx + dy * dy);
            pure_angle = atanf(pixel_per_meter * 2 * motion.params.aim_distance * dx / dn / dn) / PI * 180;
            // pure_angle -= 1;
            // if(pure_angle>0) pure_angle += 2;//pure_angle *= left_scale;
            // else pure_angle -= 1;//pure_angle *= right_scale;
            // cout << pure_angle << endl;
            int pwmDiff = 0;
            pwmDiff = (pure_angle * P) + (pure_angle - pure_angleLast) * D;  //舵机PWM偏移量  
            pure_angleLast = pure_angle;

            float pd_error =  COLSIMAGE / 2 - ctrlCenter.controlCenter ; // 图像控制中心转换偏差
            static int pd_errorLast = 0;                    // 记录前一次的偏差
            int pd_pwmDiff = 0;
            pd_pwmDiff = (pd_error * pd_P) + (pd_error - pd_errorLast) * pd_D;  //舵机PWM偏移量  
            pd_errorLast = pd_error;
           
            servoPwm = (uint16_t)(PWMSERVOMID + alpha*pwmDiff + (1-alpha)*pd_pwmDiff); // PWM转换
            cout << servoPwm << endl;
            //else servoPwm = (uint16_t)(PWMSERVOMID + pwmDiff);
        }
        else
        {
            float error =  COLSIMAGE / 2 - ctrlCenter.controlCenter ; // 图像控制中心转换偏差
            static int errorLast = 0;                    // 记录前一次的偏差
            int pwmDiff = 0;
            pwmDiff = (error * P) + (error - errorLast) * D;  //舵机PWM偏移量  
            errorLast = error;
            servoPwm = (uint16_t)(PWMSERVOMID + pwmDiff); // PWM转换
        }   
    }

    /**
     * @brief 变加速控制
     *
     * @param enable 加速使能
     * @param control
     */
    void speedCtrl(ControlCenter control)
    {
        // float S_triangle,S_all=0;
        // int real_size = control.speed_centeredge.size();
        // S_triangle=0.5*((control.speed_centeredge[real_size-1].y-control.speed_centeredge[0].y)>0?(control.speed_centeredge[real_size-1].y-control.speed_centeredge[0].y):(control.speed_centeredge[0].y-control.speed_centeredge[real_size-1].y))
        // *((control.speed_centeredge[real_size-1].x-control.speed_centeredge[0].x)>0?(control.speed_centeredge[real_size-1].x-control.speed_centeredge[0].x):(control.speed_centeredge[0].x-control.speed_centeredge[real_size-1].x));
        // // printf("%f\n",S_triangle);
        // for(int j=0;j<real_size-1;j++)
        // {
        //     S_all+=0.5*(control.speed_centeredge[0].x-control.speed_centeredge[j].x+control.speed_centeredge[0].x-control.speed_centeredge[j+1].x)*((control.speed_centeredge[j].y-control.speed_centeredge[j+1].y)>0?(control.speed_centeredge[j].y-control.speed_centeredge[j+1].y):(control.speed_centeredge[j+1].y-control.speed_centeredge[j].y));
        //    // printf("%f ",S_all);
        // }

        // S_all-=S_triangle;
        // S_all /= real_size;
        // cout << S_all << endl;

        POINT car(ROWSIMAGE-1, COLSIMAGE/2);
        // car.x = control.speed_centeredge[0].x;
        // car.y = control.speed_centeredge[0].y;
        // 计算远锚点偏差值 delta
        float dy = control.centerEdge[control.speed_aim_idx].x - car.x;
        float dx = car.y - control.centerEdge[control.speed_aim_idx].y;
        float dn = sqrt(dx * dx + dy * dy);
        speed_pure_angle = atanf(pixel_per_meter * 2 * 0.2 * dx / dn / dn) / PI * 180;//角度误差(赛道中点算的)

        speed_max = control.centerEdge_show[control.speed_aim_idx].y - 160;
        // cout << speed_max << endl;
        // speed = params.speedBase - speed_pure_angle * speed_pure_angle / 10 * params.DecreaseRatio;//实际速度

        // speed_max=0,real_angle_zero=0,speed_error=0;
        // if (speed_pure_angle>real_angle_zero)
        // {
        //     if(160>car.y)
        //     {
        //         speed_error = 160-car.y;
        //     }
        //     else speed_error = 0;
        // }/*左拐*/
        // else
        // {
        //     if(160<car.y)
        //     {
        //         speed_error = 160-car.y;
        //     }
        //     else speed_error = 0;
        // }/*右拐*/
        // speed_max = params.a*(abs(speed_pure_angle-real_angle_zero)-params.x0)*(abs(speed_pure_angle-real_angle_zero)-params.x0)*(abs(speed_pure_angle-real_angle_zero)-params.x0) + params.c;
        // speed = speed_max - speed_error * (speed_error - params.speedBase) * params.DecreaseRatio;
        // speed = 2.0;
        // if(control.sigmaCenter>4000) speed = 0.5;
        // speed = params.a*(abs(speed_pure_angle)-params.x0)*(abs(speed_pure_angle)-params.x0)*(abs(speed_pure_angle)-params.x0) + params.c;
        speed = 0.2 + 5.351e-07*(abs(speed_max)-5)*(abs(speed_max)-5)*(abs(speed_max)-5) - 0.0001062*(abs(speed_max)-5)*(abs(speed_max)-5) - 0.005315*(abs(speed_max)-5) + 3.3;
        
        // if(abs(speed_max)>120) speed_max = 120;
        // speed = (2.304*abs(speed_max)*abs(speed_max) -305.2*abs(speed_max) + 2.144e+04)/(abs(speed_max)*abs(speed_max) -107*abs(speed_max) + 7174);
        // if(abs(S_all)>100) speed = params.speedTurn_Left;
        // else speed = params.speedNorm;
        if(speed>3.0) speed = 3.0;
        if(speed<1.8) speed = 1.8;
        // vector<int> K_Center_Line;
        // K_Centerline_Sum = 0; Centerline_Turn_Row = 0;

        // /*得到中线每行差值*/
        // for(uint8_t i=1;i<control.speed_centeredge.size();i++){
        //     int K = control.speed_centeredge[i].y - control.speed_centeredge[i-1].y;
        //     K_Center_Line.push_back(abs(K));
        //     K_Centerline_Sum+=K; //S弯的值会小于单纯的弯道
        // }
        // Centerline_Turn_Row = K_Centerline_Sum / control.speed_centeredge.size();

        // for(uint8_t i=2;i<K_Center_Line.size();i++)
        // {
        //     if( ( (K_Center_Line[i]>0 && K_Center_Line[i-1]>0 && K_Center_Line[i-2]>0) || (K_Center_Line[i]<0 && K_Center_Line[i-1]<0 && K_Center_Line[i-2]<0) ) 
        //         && abs(K_Center_Line[i] + K_Center_Line[i-1] + K_Center_Line[i-2])>= 20 )
        //     {
        //         if(control.centerEdge[i].x<180)
        //         {
        //             Centerline_Turn_Row = control.centerEdge[i].x;
        //             break;
        //         }
        //     }
        // }


    }
};
