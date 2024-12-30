#include "../include/common.hpp"     //公共类方法文件
#include "../include/detection.hpp"  //百度Paddle框架移动端部署
#include "../include/uart.hpp"       //串口通信驱动
#include "controlcenter.cpp"         //控制中心计算类
#include "detection/bridge.cpp"      //AI检测：坡道区
#include "detection/danger.cpp"      //AI检测：危险区
#include "detection/parking.cpp"     //AI检测：停车区
#include "detection/racing.cpp"      //AI检测：追逐区
#include "detection/rescue.cpp"      //AI检测：救援区
#include "motion.cpp"                //智能车运动控制类
#include "preprocess.cpp"            //图像预处理类
#include "recognition/crossroad.cpp" //十字道路识别与路径规划类
#include "recognition/ring.cpp"      //环岛道路识别与路径规划类
#include "recognition/tracking.cpp"  //赛道识别基础类
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <future>
#include <opencv2/highgui.hpp> //OpenCV终端部署
#include <opencv2/opencv.hpp>  //OpenCV终端部署
#include <signal.h>
#include <unistd.h>
#include <chrono>

using namespace std;
using namespace cv;

double params[10]={0};
bool RescueFlag=false, DangerFlag=false, RacingFlag=false;
void ModelSearch(Mat image, Tracking track, int* colorcnt);

// 色调变冷，解决偏黄问题
cv::Mat reduceYellow(const cv::Mat& input) {
    cv::Mat result = input.clone();
    cv::Mat channels[3];
    cv::split(result, channels);

    // 调整红色和绿色通道的比例，增加蓝色通道
    channels[2] = channels[2] * 0.8; // 减少红色通道的强度
    channels[1] = channels[1] * 0.8; // 减少绿色通道的强度
    channels[0] = channels[0] * 1.1; // 增加蓝色通道的强度

    cv::merge(channels, 3, result);
    return result;
}

int main() {
    Preprocess preprocess;    // 图像预处理类
    Motion motion;            // 运动控制类
    Tracking tracking;        // 赛道识别类
    Crossroad crossroad;      // 十字道路识别类
    Ring ring;                // 环岛识别类
    Bridge bridge;            // 坡道区检测类
    Parking parking;          // 停车区检测类
    Danger danger;            // 危险区检测类
    Rescue rescue;            // 救援区检测类
    Racing racing;            // 追逐区检测类
    ControlCenter ctrlCenter; // 控制中心计算类
    Display display(4);       // 初始化UI显示窗口
    VideoCapture capture;     // Opencv相机类
    bool ModelFlag = false;
    float model_dis=0;
    bool Straight=true;
    vector<int> RoadWidth; // 存储参数的一维数组
    int colorcnt[4]={0};
    bool AIFlag = false, AIrenew=false;
    float protect_dis=0;
    bool protect_flag=false;
    float dis_parking_protect=0, dis_racing_protect=0;

    VideoWriter wr;						////
    wr.open("../res/samples/video.avi", cv::VideoWriter::fourcc('M','J','P','G'), 10, Size(320,240), false); ////
    
    // 目标检测类(AI模型文件)
    shared_ptr<Detection> detection = make_shared<Detection>(motion.params.model);
    detection->score = motion.params.score; // AI检测置信度

    //USB转串口初始化： /dev/ttyUSB0
    shared_ptr<Uart> uart = make_shared<Uart>("/dev/ttyUSB0"); // 初始化串口驱动
    int ret = uart->open();
    if (ret != 0) {
        printf("[Error] Uart Open failed!\n");
        return -1;
    }
    uart->startReceive(); // 启动数据接收子线程

    // USB摄像头初始化
    if (motion.params.debug)
        capture = VideoCapture("/dev/video0"/*"../res/samples/sample.mp4"*//*"/home/edgeboard/workspace/autocar_06_20/res/samples/sample.mp4"*/); // 打开摄像头
    else
        capture = VideoCapture("/dev/video0"); // 打开摄像头
    if (!capture.isOpened()) {
        printf("can not open video device!!!\n");
        return 0;
    }

    capture.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));  //改变编码方式
    capture.set(cv::CAP_PROP_FPS, 120);  //设置帧率
    capture.set(CAP_PROP_FRAME_WIDTH, 320);  // 设置图像分辨率
    capture.set(CAP_PROP_FRAME_HEIGHT, 240); 

    cout << capture.get(cv::CAP_PROP_FPS) << endl;


    // //读取路宽
    ifstream inFile("../res/samples/parameters.txt");
        if (!inFile.is_open()) {
            cerr << "Failed to open the input file." << endl;
            return -1;
        }
        int value;
        // 逐行读取文件，并将参数存储到数组中
        while (inFile >> value) {
            RoadWidth.push_back(value);
        }
        // 关闭文件流
        inFile.close();


    //选择是否debug
    std::cout << "0:run,  1:debug: ";
    cin >> motion.params.debug;


    //延时发车
    if (!motion.params.debug) {
        printf("--------------3s后发车!!!-------------------\n");
        uart->buzzerSound(uart->BUZZER_OK); // 祖传提示音效
        while (ret < 10) // 延时3s
        {
          uart->carControl(0, PWMSERVOMID); // 通信控制车辆停止运动
          waitKey(200);
          ret++;
        }
        uart->buzzerSound(uart->BUZZER_START); // 祖传提示音效
    }

    // 初始化参数
    Scene scene = Scene::NormalScene;     // 初始化场景：常规道路
    Scene sceneLast = Scene::NormalScene; // 记录上一次场景状态
    uint64_t preTime, frameTime;
    Mat img;
    std::future<void> future;
    vector<PredictResult> AIresults;
    //发车初始化保护
    /*motion.params.cross = */motion.params.parking = false;
    bool first=true;

    while(1) {

      if (!capture.read(img))
        continue;
      
      // 颜色偏黄，将色调变冷
      // img = reduceYellow(img);
      preTime = chrono::duration_cast<chrono::milliseconds>(
                  chrono::system_clock::now().time_since_epoch())
                  .count(); 

      if(motion.params.saveImg) savePicture(img); //存图

      Mat imgBinary = preprocess.binaryzation(img); // 图像二值化
      Mat AIimg = img;
      // if(motion.params.RescueMode==0)
      // {
          if(!future.valid()&&(scene==Scene::NormalScene||scene==Scene::CrossScene||(scene == Scene::RingScene && (ring.ringStep==Ring::RingStep::Verifing ||ring.ringStep==Ring::RingStep::Waiting))))  future = std::async(std::launch::async, [&detection](const cv::Mat image) {detection->inference(image);}, AIimg);
          if (future.wait_for(std::chrono::seconds(0)) == std::future_status::ready &&(scene==Scene::NormalScene||scene==Scene::CrossScene||(scene == Scene::RingScene && (ring.ringStep==Ring::RingStep::Verifing ||ring.ringStep==Ring::RingStep::Waiting))))
          {
            // 如果异步任务已经完成
            // 获取结果并处理
            AIresults = detection->results;
            AIFlag = true;
            future = std::async(std::launch::async, [&detection](const cv::Mat image) {detection->inference(image);}, AIimg);
          } 
          else 
          {
            AIresults.clear();
            AIFlag = false;
          }
      // }
       
      //[04] 赛道识别
      tracking.rowCutUp = motion.params.rowCutUp; // 图像顶部切行（巡线处理顶部）
      tracking.rowCutBottom =
          motion.params.rowCutBottom; // 图像底部切行（盲区距离）
      tracking.trackRecognition(imgBinary);

      if(!DangerFlag && !RescueFlag && (scene==Scene::NormalScene || scene==Scene::CrossScene))  ModelSearch(img, tracking, colorcnt);
      if(!motion.params.rescue) RescueFlag = false;
      if(!motion.params.danger) DangerFlag = false;
      // if(!motion.params.racing) RacingFlag = false;

      // ofstream outFile("../res/samples/parameters.txt");
      // if (!outFile.is_open()) {
      //     cerr << "Failed to open the output file." << endl;
      //     return -1;
      // }
      // for (int row = 0; row < tracking.pointsEdgeLeft.size(); ++row) {
      //     int x = tracking.pointsEdgeRight[row].y - tracking.pointsEdgeLeft[row].y ;
      //     outFile << x << endl;
      // }
      // outFile.close();
      // cout << "Parameters saved to parameters.txt" << endl;


      //[05] 停车区检测
      if ((scene==Scene::NormalScene || scene==Scene::ParkingScene) && motion.params.parking) {
        if (parking.process(tracking, frameTime, motion.params.speedNorm)) {
          scene = Scene::ParkingScene;
          if (parking.park) {  
            uart->carControl(0, PWMSERVOMID); // 控制车辆停止运动
            sleep(1);
            printf("-----> STOP !!! <-----\n");
            exit(0); // 程序退出
          }
        }
        else scene = Scene::NormalScene;
      }

      //[06] 救援区检测
      if ((scene == Scene::NormalScene || scene == Scene::RescueScene) && 
          motion.params.rescue && !protect_flag) {
        if (rescue.process(AIFlag, img, tracking, AIresults, frameTime, motion.params.speedRescue, motion.params.dis_waiting, motion.params.dis_entering, motion.params.Cone_Scale_Down, motion.params.RescueMode, RescueFlag, uart))
        {
          if(scene==Scene::NormalScene)
          {
            if(motion.params.RescueMode!=0) RescueFlag=false;
            uart->buzzerSound(Uart::BUZZER_WARNNING);  
          }
          scene = Scene::RescueScene;
        }
        else
        {
          if(motion.params.RescueMode==0 && scene==Scene::RescueScene)  future = std::async(std::launch::async, [&detection](const cv::Mat image) {detection->inference(image);}, AIimg);
          if(scene==Scene::RescueScene && motion.params.RescueMode!=0) 
          {
            protect_flag=true;
            protect_dis=0;
          }
          scene = Scene::NormalScene;   
        }
      }

      //[07] 追逐区检测
      if ((scene == Scene::NormalScene || scene==Scene::CrossScene || scene == Scene::RacingScene || (scene == Scene::RingScene && (ring.ringStep==Ring::RingStep::Verifing ||ring.ringStep==Ring::RingStep::Waiting))) &&
          motion.params.racing) {
        if (racing.process(AIFlag, tracking, AIresults, RoadWidth))
        {
          if(scene==Scene::NormalScene||scene==Scene::CrossScene || (scene == Scene::RingScene && (ring.ringStep==Ring::RingStep::Verifing || ring.ringStep==Ring::RingStep::Waiting)))
          {
            if(scene==Scene::RingScene) ring.ringStep = Ring::RingStep::None;
            RacingFlag = false;
            uart->buzzerSound(Uart::BUZZER_WARNNING);  
          }
          scene = Scene::RacingScene;
        }
        else
        {
          if(scene==Scene::RacingScene)  future = std::async(std::launch::async, [&detection](const cv::Mat image) {detection->inference(image);}, AIimg);  
          scene = Scene::NormalScene;
        }
          
      }

      //[08] 坡道区检测
      if ((scene == Scene::NormalScene || scene == Scene::BridgeScene) &&
          motion.params.bridge) {
        if (bridge.process(tracking, frameTime, motion.params.speedBridge, RoadWidth, ctrlCenter.left_num, ctrlCenter.right_num))
        {
          if(scene==Scene::NormalScene) uart->buzzerSound(Uart::BUZZER_WARNNING);
          scene = Scene::BridgeScene;
        }
        else
        {
          if(scene==Scene::BridgeScene) uart->buzzerSound(Uart::BUZZER_WARNNING);
          scene = Scene::NormalScene;
        }
      }
  
      // [09] 危险区检测
      if ((scene == Scene::NormalScene || scene == Scene::DangerScene) &&
          motion.params.danger) {
        if (danger.process(imgBinary, img, tracking, frameTime, motion.params.speedDanger, motion.params.DisLeaving, RoadWidth, motion.params.UpScale, motion.params.block_scale, motion.params.distance_block, DangerFlag, motion.params.DangerMode)) 
        {
          if(scene == Scene::NormalScene || scene==Scene::CrossScene)
          {
            DangerFlag = false;
            uart->buzzerSound(Uart::BUZZER_WARNNING);  
          } 
          scene = Scene::DangerScene;
        } 
        else
        {
          scene = Scene::NormalScene;
        }
      }

      //[11] 环岛识别与路径规划
      if ((scene == Scene::NormalScene || scene == Scene::RingScene || scene == Scene::CrossScene) &&
          motion.params.ring) 
      {
          uint8_t ringEnter[10] = {motion.params.ringEnter0, motion.params.ringEnter1,
                                  motion.params.ringEnter2, motion.params.ringEnter3};
          
          if (ring.ringRecognition(tracking, imgBinary, motion.params.ringSum, ringEnter, RoadWidth, motion.params.speedRing, frameTime))
          {
            if (scene == Scene::NormalScene) // 初次识别-蜂鸣器提醒
            {
              uart->buzzerSound(Uart::BUZZER_WARNNING);             // OK    
            }
            scene = Scene::RingScene;
          }   
          else
            scene = Scene::NormalScene;        
      }


      // [10] 十字道路识别与路径规划
      if ((scene == Scene::NormalScene || scene == Scene::CrossScene) &&
          motion.params.cross) {
        if (crossroad.crossRecognition(tracking))
          scene = Scene::CrossScene;
        else
          scene = Scene::NormalScene;
      } 

      //[12] 车辆控制中心拟合
      int calstartline = motion.params.calstartline;
      //if((scene==Scene::NormalScene || scene==Scene::CrossScene) && !Straight) calstartline = motion.params.startline_turn;
      if(scene == Scene::DangerScene) calstartline = motion.params.startline_danger;
      ctrlCenter.fitting(tracking, calstartline, RoadWidth, scene, motion.params.aim_distance, ring.R100, ring.ringStep, ring.ringType);
      if (scene != Scene::RescueScene) {
        if (ctrlCenter.derailmentCheck(tracking)) // 车辆冲出赛道检测（保护车辆）
        {
          uart->carControl(0, PWMSERVOMID); // 控制车辆停止运动
          sleep(1);
          printf("-----> System Exit!!! <-----\n");
          exit(0); // 程序退出
        }
      }

      motion.speedCtrl(ctrlCenter);
      //[13] 车辆运动控制(速度+方向)
      if (!motion.params.debug) // 非调试模式下
      {
        // 车速控制
        if(scene==Scene::NormalScene || scene==Scene::CrossScene || scene == Scene::ParkingScene)
            {
              motion.speedCtrl(ctrlCenter);
            //   POINT car(ROWSIMAGE-1, COLSIMAGE/2);
            //  // 计算远锚点偏差值 delta
            //   float dy = ctrlCenter.centerEdge[ctrlCenter.speed_aim_idx].x - car.x;
            //   float dx = car.y - ctrlCenter.centerEdge[ctrlCenter.speed_aim_idx].y;
            //   float dn = sqrt(dx * dx + dy * dy);
            //   float speed_pure_angle = atanf(pixel_per_meter * 2 * 0.2 * dx / dn / dn) / PI * 180;
            //   // motion.speed = motion.params.speedBase - speed_pure_angle * speed_pure_angle / 10 * motion.params.DecreaseRatio;
            //   motion.speed = motion.params.a*(abs(speed_pure_angle)-motion.params.x0)*(abs(speed_pure_angle)-motion.params.x0)*(abs(speed_pure_angle)-motion.params.x0) + motion.params.c;
            //   if(motion.speed>3.5) motion.speed = 3.5;
              //  motion.speed = motion.params.speedBase - abs(motion.pure_angle) * motion.params.DecreaseRatio;
              if(motion.speed>4.0|| motion.speed<-4.0) motion.speed = 0;
              // if(motion.speed<2.3) motion.speed = 2.3;
            //  if(motion.pure_angle>8) motion.speed = motion.params.speedTurn_Left;
            //  else if(motion.pure_angle<-8) motion.speed = motion.params.speedTurn_Right;
            //  else motion.speed = motion.params.speedNorm;
            }
        else if(scene==Scene::BridgeScene) motion.speed = motion.params.speedBridge; 
        else if(scene==Scene::RingScene)
        {
            if(ring.R100) motion.speed = motion.params.speedRing + 0.3;
            else motion.speed = motion.params.speedRing;
        }
        else if(scene== Scene::DangerScene)
            motion.speed = motion.params.speedDanger;  
        else if(scene==Scene::RescueScene)
        {
          if(rescue.carStoping) motion.speed = 0;
          else if(rescue.carExitting) motion.speed = -motion.params.speedRescue;
          else motion.speed = motion.params.speedRescue;
        }            
        else if(scene==Scene::RacingScene)
        {
          if(racing.carStoping) motion.speed = 0;
          else if(racing.typeRace==Racing::TypeRace::Safe) motion.speed = motion.params.speedSafe;
          else if(racing.typeRace==Racing::TypeRace::Danger) motion.speed = motion.params.speedRacing_Dan;
          else if(racing.typeRace==Racing::TypeRace::Spy)
          {
            if(racing.stepSpy==Racing::StepSpy::Bypass) motion.speed = motion.params.speedSpy1;
            else motion.speed = motion.params.speedSpy2;
          } 
        }
          
      }   
      else motion.speed = 0;               
      // 姿态控制
        float P, D;
        if(scene==Scene::NormalScene || scene==Scene::CrossScene || scene == Scene::ParkingScene || scene==Scene::BridgeScene)
        { 
           P = motion.params.pl*motion.pure_angle*motion.pure_angle/50 + motion.params.ph;
            //if(P>motion.params.pl*45*45/50 + motion.params.ph) P=motion.params.pl*45*45/50 + motion.params.ph;
           if(P>8.3) P=8.3;
           D = motion.params.NormD;
        } 
        else if(scene == Scene::RingScene)
        {
        //printf("-----------环岛\n");
          if(ring.ringStep==Ring::RingStep::Waiting || ring.ringStep==Ring::RingStep::Verifing) 
          {
            P = motion.params.pl*motion.pure_angle*motion.pure_angle/50 + motion.params.ph;
            if(P>9) P=9;
            D = motion.params.NormD;
          }
          else {
            if(ring.ringNum==0){P=motion.params.ringP0; D=motion.params.ringD0;} //R60
            else if(ring.ringNum==1){P=motion.params.ringP1; D=motion.params.ringD1;} //R50
            }
        }
        else if(scene== Scene::DangerScene)
        {
      //   printf("---------------危险区\n");
          if(danger.step==Danger::Step::Start && danger.first==0) {P=motion.params.Danger_Start_P; D=motion.params.Danger_Start_D;}
          else if(danger.step==Danger::Step::Start || danger.step==Danger::Step::Inside) {P=motion.params.DangerP1; D=motion.params.DangerD1;}
          else {P=motion.params.DangerP2; D=motion.params.DangerD2;}
        }
        else if(scene== Scene::RescueScene)
        {
    //    printf("---------------救援区\n");
        if(rescue.step==Rescue::Step::Enable || (rescue.step==Rescue::Step::Enter && rescue.waiting)) {P=motion.params.RescueP; D=motion.params.RescueD;}
        else  {P = motion.params.RescueP; D = motion.params.RescueD;}
        }
        else if(scene==Scene::RacingScene)
        {
          if(racing.typeRace==Racing::TypeRace::Safe) {P = motion.params.SafeP; D = motion.params.SafeD;} 
          else if(racing.typeRace==Racing::TypeRace::Danger) {P = motion.params.Racing_DanP; D = motion.params.Racing_DanD;} 
          else if(racing.typeRace==Racing::TypeRace::Spy) {P = motion.params.SpyP; D = motion.params.SpyD;} 
          
        }
         
        motion.poseCtrl(ctrlCenter, P, D, scene, tracking, motion.params.pd_P, motion.params.pd_D, motion.params.alpha);  
        if(rescue.carStoping)
        {
          if(rescue.entryLeft) motion.servoPwm = PWMSERVOMIN;
          else motion.servoPwm = PWMSERVOMAX;
        }
        if(racing.carStoping)
        {
          if(racing.sideLeft) motion.servoPwm = PWMSERVOMIN;
          else motion.servoPwm = PWMSERVOMAX;
        }
        uart->carControl(motion.speed, motion.servoPwm); // 串口通信控制车辆

        auto startTime = chrono::duration_cast<chrono::milliseconds>(
                            chrono::system_clock::now().time_since_epoch())
                            .count();
        frameTime = startTime - preTime;
        
        if(protect_flag)
        {
          protect_dis += frameTime*motion.speed/1000;
          if(protect_dis>2) {protect_flag = false;RescueFlag=false;}
        }

        if(!motion.params.parking)
        {
          dis_parking_protect += frameTime*motion.speed/1000;
          if(dis_parking_protect>1) /*motion.params.cross =*/ motion.params.parking=true;
        }
        // if(RacingFlag)
        // {
        //   dis_racing_protect += frameTime*motion.speed/1000;
        //   if(dis_racing_protect>1.5) RacingFlag = false;
        // }

        // printf(">> FrameTime: %ldms | %.2ffps \n", startTime - preTime,
        //        1000.0 / (startTime - preTime));
      if (motion.params.debug) {
          display.setNewWindow(1, "Binary", imgBinary);

          Mat hsv;
          cvtColor(img, hsv, COLOR_BGR2HSV);
          // 定义黄色的HSV范围
          Scalar lowerYellow(15, 100, 60);
          Scalar upperYellow(35, 255, 255);            
          Mat Mask;
          inRange(hsv, lowerYellow, upperYellow, Mask);
          display.setNewWindow(2, "Track", Mask);

          if(AIFlag) detection->drawBox(img); // 图像绘制AI结果
          display.setNewWindow(3, "AIimg",
                              img);
          
          ctrlCenter.drawImage(tracking,
                              img); // 图像绘制路径计算结果（控制中心）
          display.setNewWindow(4, "Ctrl", img);
          display.show(); // 显示综合绘图
          waitKey(1);    // 等待显示
          }
    
          sceneLast = scene; // 记录当前状态
          if (scene == Scene::CrossScene) scene = Scene::NormalScene;  //防止crossscene对下次判别有影响

          //[16] 按键退出程序
          if (uart->keypress) {
          uart->carControl(0, PWMSERVOMID); // 控制车辆停止运动
          sleep(1);
          printf("-----> System Exit!!! <-----\n");
          exit(0); // 程序退出
              }

          if (!first && motion.params.saveImg && !motion.params.debug)
          {
            // Mat frame_r = AIimg_copied;
            // Mat gray_frame;
            Mat frame_r = img;
            Mat gray_frame;
            
            // if(scene==Scene::NormalScene)
            // {
            putText(frame_r,
            "P:" + formatDoble2String((double)P, 2),
            Point(20, 40), FONT_HERSHEY_PLAIN, 1,
            Scalar(0, 0, 0), 1); // 车速
            putText(frame_r,
            "FPS:" + formatDoble2String(1000.0 / (startTime - preTime), 2),
            Point(20, 80), FONT_HERSHEY_PLAIN, 1,
            Scalar(0, 0, 0), 1); // 车速

            putText(frame_r,
            "Angle:" + formatDoble2String(motion.pure_angle, 2),
            Point(20, 100), FONT_HERSHEY_PLAIN, 1,
            Scalar(0, 0, 0), 1); // 车速

            putText(frame_r,
            "Speed:" + formatDoble2String(motion.speed, 2),
            Point(20, 120), FONT_HERSHEY_PLAIN, 1,
            Scalar(0, 0, 0), 1); // 车速

            // if(AIFlag)
            // {
            //   putText(frame_r,
            // "Speed:" + formatDoble2String(motion.speed_pure_angle, 2),
            // Point(20, 140), FONT_HERSHEY_PLAIN, 1,
            // Scalar(0, 0, 0), 1); // 车速
            // }

          //  }
            //  }
            // if(ctrlCenter.left_num>=ctrlCenter.right_num)
            // {
              putText(frame_r,
               "1m_error" + formatDoble2String(motion.speed_max, 2),
              Point(20, 140), FONT_HERSHEY_PLAIN, 1,
              Scalar(0, 0, 0), 1); // 车速

              // putText(frame_r,
              //  "speed_error" + formatDoble2String(motion.speed_error, 2),
              // Point(20, 160), FONT_HERSHEY_PLAIN, 1,
              // Scalar(0, 0, 0), 1); // 车速
              putText(frame_r,
               "PWM" + formatDoble2String(motion.servoPwm, 2),
              Point(20, 180), FONT_HERSHEY_PLAIN, 1,
              Scalar(0, 0, 0), 1); // 车速

              // putText(frame_r,
              // "White" + formatDoble2String(tracking.counter_white, 2),
              // Point(150, 120), FONT_HERSHEY_PLAIN, 1,
              // Scalar(0, 0, 0), 1); // 车速
            // }
            // else{
            //   putText(frame_r,
            //   "Right",
            //   Point(80, 80), FONT_HERSHEY_PLAIN, 1,
            //   Scalar(0, 0, 0), 1); // 车速
            // }

            // putText(frame_r,
            // "red: " + formatDoble2String(params[0], 2),
            // Point(80, 40), FONT_HERSHEY_PLAIN, 1,
            // Scalar(0, 0, 0), 1); // 车速
            // putText(frame_r,
            // "left: " + formatDoble2String(params[1], 2),
            // Point(80, 60), FONT_HERSHEY_PLAIN, 1,
            // Scalar(0, 0, 0), 1); // 车速
            // putText(frame_r,
            // "center: " + formatDoble2String(params[2], 2),
            // Point(80, 80), FONT_HERSHEY_PLAIN, 1,
            // Scalar(0, 0, 0), 1); // 车速
            // putText(frame_r,
            // "right: " + formatDoble2String(params[3], 2),
            // Point(80, 100), FONT_HERSHEY_PLAIN, 1,
            // Scalar(0, 0, 0), 1); // 车速
          //  line(frame_r, cv::Point(0, calstartline), cv::Point(img.cols - 1, calstartline), cv::Scalar(0, 0, 0), 2);
          //  line(frame_r, cv::Point(0, calstartline+20), cv::Point(img.cols - 1, calstartline+20), cv::Scalar(0, 0, 0), 2);

        
            ctrlCenter.drawImage(tracking, frame_r);
            cvtColor(frame_r, gray_frame, COLOR_RGB2GRAY);
            wr.write(gray_frame);

        } 
        if(first) {first = false; continue;}
    }
    uart->close(); // 串口通信关闭
    capture.release();
    return 0;
}

/**
 * @brief 搜寻颜色判断是否需要打开模型
 *
 * @param image 校正后图像
 * @param track
 */
void ModelSearch(Mat image, Tracking track, int* colorcnt)
{
      //图像边长压缩比例为2：1
      // 将图像从BGR转换为HSV颜色空间，以便更容易提取红色
      int red=0, yellow_l=0, yellow_r=0, yellow_c=0;
      Mat hsv_img;
      cvtColor(image, hsv_img, COLOR_BGR2HSV);
      int bound = (track.pointsEdgeLeft.size()>30)? 30 : 0;

      for(int i=bound;i<track.pointsEdgeLeft.size();i+=2)
      {
        for(int y=0;y<COLSIMAGE;y+=2)
        {
          cv::Vec3b hsv = hsv_img.at<cv::Vec3b>(track.pointsEdgeLeft[i].x, y);
          //找红色
          if (((hsv[0] >= 170 && hsv[0] <= 180) || (hsv[0] >= 0 && hsv[0] <= 10)) &&
            hsv[1] >= 100 && hsv[2] >= 60 && hsv[1] <= 255 && hsv[2] <= 255)
            {
              if(y>track.pointsEdgeRight[i].y) red++;
              continue;
            }
          if ((hsv[0] >= 15 && hsv[0] <= 35) &&
          hsv[1] >= 100 && hsv[2] >= 60 && hsv[1] <= 255 && hsv[2] <= 255)
          {
            if(y<track.pointsEdgeLeft[i].y) yellow_l++;
            else if(y>track.pointsEdgeRight[i].y) yellow_r++;
            else if(y>track.pointsEdgeLeft[i].y && y<track.pointsEdgeRight[i].y)yellow_c++;
          }
        }
      }
      colorcnt[0] = red;
      colorcnt[1] = yellow_l;
      colorcnt[2] = yellow_c;
      colorcnt[3] = yellow_r;
     // params[0]=red;params[1]=yellow_l;params[2]=yellow_c;params[3]=yellow_r;
      if( yellow_l>=20&&yellow_c<=3&&yellow_r>=20) {RescueFlag=true;return;}
      if(/*red>=30 &&*/yellow_c>=20) {DangerFlag = true; return;}
    //  printf("%d %d %d %d\n", red, yellow_l, yellow_c, yellow_r);


      // if(track.pointsEdgeLeft.size()<100 || track.pointsEdgeRight.size()<100) return;
      // int step=0, cnt=0, car_cnt=0;
      // int left_col, right_col, row;
      // bool left = true;
      // if(step==0)//小车尾部跳变
      // {
      //     for(int i=50;i<track.pointsEdgeLeft.size();i+=2)
      //     {
      //         if(track.pointsEdgeLeft[i].y-track.pointsEdgeLeft[i-2].y>10 && track.pointsEdgeLeft[i-2].y>3 && track.stdevRight<10) 
      //         {
      //           left = true;
      //           left_col = track.pointsEdgeLeft[i-2].y;
      //           right_col = track.pointsEdgeLeft[i].y;
      //           row = track.pointsEdgeLeft[i].x;
      //           step++;
      //           break;
      //         }
      //         else if(track.pointsEdgeRight[i].y-track.pointsEdgeRight[i-2].y<-10 && track.pointsEdgeRight[i-2].y<COLSIMAGE-3 && track.stdevLeft<10)
      //         {
      //           left = false;
      //           left_col = track.pointsEdgeRight[i].y;
      //           right_col = track.pointsEdgeRight[i-2].y;
      //           row = track.pointsEdgeRight[i].x;
      //           step++;
      //           break;
      //         }
      //     }
      // }
      // if(step==1)
      // {
      //   if(left)
      //   {
      //     for(int i=row;i<track.pointsEdgeLeft.size(); i+=2)
      //     {
      //       if(track.pointsEdgeLeft[i+2].y-track.pointsEdgeLeft[i].y<-10 && car_cnt>=5) {step ++;break;}
      //       if(track.pointsEdgeLeft[i+2].y - track.pointsEdgeLeft[i].y <=3) car_cnt ++;
      //       else car_cnt=0;
      //     } 
      //   }
      //   else
      //   {
      //     for(int i=row;i<track.pointsEdgeRight.size(); i+=2)
      //     {
      //       if(track.pointsEdgeRight[i+2].y-track.pointsEdgeRight[i].y > 10 && car_cnt>=5) {step ++; break;}
      //       if(track.pointsEdgeRight[i].y - track.pointsEdgeRight[i+2].y <=3) car_cnt ++;
      //       else car_cnt=0;
      //     } 
      //   }
      // }
      // if(step==2) //小车尾部蓝色
      // {
      //   for(int y=left_col;y<right_col;y++)
      //   {
      //     int x=row;
      //     cv::Vec3b hsv = hsv_img.at<cv::Vec3b>(x, y);
      //     cv::Vec3b hsv1 = hsv_img.at<cv::Vec3b>(x+20, y);
      //     if((hsv[0] >= 100 && hsv[0] <= 140 && hsv[1] >= 50 && hsv[1] <= 255 && hsv[2] >= 50 && hsv[2] <= 255) &&
      //             (hsv1[0] < 100 ||hsv1[0] > 140 || hsv1[1] < 50 || hsv[2] < 50)) cnt++;
      //     if(cnt>=0.7*(right_col-left_col)) {cnt=0; step++;break;}
      //   }
      // }
      // if(step==3) 
      // {
      //   for(int y=left_col;y<right_col;y++)
      //   {
      //     for(int x=row-20;x>=row-40;x-=5)
      //     {
      //       cv::Vec3b hsv = hsv_img.at<cv::Vec3b>(x, y);
      //       if(x>=0 && (hsv[0] < 100 ||hsv[0] > 140 || hsv[1] < 50 || hsv[2] < 50)) cnt++;
      //       if(cnt>=2) {RacingFlag = true;return;}
      //     }
      //   }
      // }
}