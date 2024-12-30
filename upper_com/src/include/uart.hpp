#pragma once
/**
 ********************************************************************************************************
 *                                               示例代码
 *                                             EXAMPLE  CODE
 *
 *                      (c) Copyright 2024; SaiShu.Lcc.; Leo;
 *https://bjsstech.com 版权所属[SASU-北京赛曙科技有限公司]
 *
 *            The code is for internal use only, not for commercial
 *transactions(开源学习,请勿商用). The code ADAPTS the corresponding hardware
 *circuit board(代码适配百度Edgeboard-智能汽车赛事版), The specific details
 *consult the professional(欢迎联系我们,代码持续更正，敬请关注相关开源渠道).
 *********************************************************************************************************
 * @file uart.hpp
 *
 * @author Leo
 * @brief 上下位机串口通信协议
 * @version 0.1
 * @date 2023-12-26
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "common.hpp"
#include <iostream>               // 输入输出类
#include <libserial/SerialPort.h> // 串口通信
#include <math.h>                 // 数学函数类
#include <stdint.h>               // 整型数据类
#include <string.h>
#include <thread>

using namespace LibSerial;
using namespace std;

// USB通信帧
#define USB_FRAME_HEAD 0x42 // USB通信帧头
#define USB_FRAME_LENMIN 4  // USB通信帧最短字节长度
#define USB_FRAME_LENMAX 12 // USB通信帧最长字节长度

// USB通信地址
#define USB_ADDR_CARCTRL 1 // 智能车速度+方向控制
#define USB_ADDR_BUZZER 4  // 蜂鸣器音效控制
#define USB_ADDR_LED 5     // LED灯效控制
#define USB_ADDR_KEY 6     // 按键信息

class Uart {
private:
  /**
   * @brief 串口通信结构体
   *
   */
  typedef struct {
    bool start;                           // 开始接收标志
    uint8_t index;                        // 接收序列
    uint8_t buffRead[USB_FRAME_LENMAX];   // 临时缓冲数据
    uint8_t buffFinish[USB_FRAME_LENMAX]; // 校验成功数据
  } SerialStruct;

  std::unique_ptr<std::thread> threadRec; // 串口接收子线程
  std::shared_ptr<SerialPort> serialPort = nullptr;
  std::string portName; // 端口名字
  bool isOpen = false;
  SerialStruct serialStr; // 串口通信数据结构体

  /**
   * @brief 32位数据内存对齐/联合体
   *
   */
  typedef union {
    uint8_t buff[4];
    float float32;
    int int32;
  } Bit32Union;

  /**
   * @brief 16位数据内存对齐/联合体
   *
   */
  typedef union {
    uint8_t buff[2];
    int int16;
    uint16_t uint16;
  } Bit16Union;

  /**
   * @brief 串口接收字节数据
   *
   * @param charBuffer
   * @param msTimeout
   * @return int
   */
  int receiveBytes(unsigned char &charBuffer, size_t msTimeout = 0) {
    /*try检测语句块有没有异常。如果没有发生异常,就检测不到。
    如果发生异常，則交给 catch 处理，执行 catch 中的语句* */
    try {
      /*从串口读取一个数据,指定msTimeout时长内,没有收到数据，抛出异常。
      如果msTimeout为0，则该方法将阻塞，直到数据可用为止。*/
      serialPort->ReadByte(charBuffer, msTimeout); // 可能出现异常的代码段
    } catch (const ReadTimeout &) // catch捕获并处理 try 检测到的异常。
    {
      // std::cerr << "The ReadByte() call has timed out." << std::endl;
      return -2;
    } catch (const NotOpen &) // catch()中指明了当前 catch 可以处理的异常类型
    {
      std::cerr << "Port Not Open ..." << std::endl;
      return -1;
    }
    return 0;
  };

  /**
   * @brief
   *
   * @param data
   * @return int
   */
  int transmitByte(unsigned char data) {
    // try检测语句块有没有异常
    try {
      serialPort->WriteByte(data); // 写数据到串口
    } catch (const std::runtime_error &) // catch捕获并处理 try 检测到的异常。
    {
      std::cerr << "The Write() runtime_error." << std::endl;
      return -2;
    } catch (const NotOpen &) // catch捕获并处理 try 检测到的异常。
    {
      std::cerr << "Port Not Open ..." << std::endl;
      return -1;
    }
    serialPort->DrainWriteBuffer(); // 等待，直到写缓冲区耗尽，然后返回。
    return 0;
  }

public:
  // 定义构造函数
  Uart(const std::string &port) : portName(port){};
  // 定义析构函数
  ~Uart() { close(); };
  bool keypress = false; // 按键
  bool reach = false;
  bool Have_send = false;

  /**
   * @brief 蜂鸣器音效
   *
   */
  enum Buzzer {
    BUZZER_OK = 0,   // 确认
    BUZZER_WARNNING, // 报警
    BUZZER_FINISH,   // 完成
    BUZZER_DING,     // 提示
    BUZZER_START,    // 开机
    COUNT,
    NONE,
  };

public:
  /**
   * @brief 启动串口通信
   *
   * @param port 串口号
   * @return int
   */
  int open(void) {
    serialPort = std::make_shared<SerialPort>();
    if (serialPort == nullptr) {
      std::cerr << "Serial Create Failed ." << std::endl;
      return -1;
    }
    // try检测语句块有没有异常
    try {
      serialPort->Open(portName);                     // 打开串口
      serialPort->SetBaudRate(BaudRate::BAUD_115200); // 设置波特率
      serialPort->SetCharacterSize(CharacterSize::CHAR_SIZE_8); // 8位数据位
      serialPort->SetFlowControl(FlowControl::FLOW_CONTROL_NONE); // 设置流控
      serialPort->SetParity(Parity::PARITY_NONE);                 // 无校验
      serialPort->SetStopBits(StopBits::STOP_BITS_1); // 1个停止位
    } catch (const OpenFailed &) // catch捕获并处理 try 检测到的异常。
    {
      std::cerr << "Serial port: " << portName << "open failed ..."
                << std::endl;
      isOpen = false;
      return -2;
    } catch (const AlreadyOpen &) // catch捕获并处理 try 检测到的异常。
    {
      std::cerr << "Serial port: " << portName << "open failed ..."
                << std::endl;
      isOpen = false;
      return -3;
    } catch (...) // catch捕获并处理 try 检测到的异常。
    {
      std::cerr << "Serial port: " << portName << " recv exception ..."
                << std::endl;
      isOpen = false;
      return -4;
    }

    serialStr.start = false;
    serialStr.index = 0;
    isOpen = true;

    return 0;
  }

  /**
   * @brief 启动接收子线程
   *
   */
  void startReceive(void) {
    if (!isOpen) // 串口是否正常打开
      return;

    // 启动串口接收子线程
    threadRec = std::make_unique<std::thread>([this]() {
      while (1) {
        receiveCheck(); // 串口接收校验
      }
    });
  }

  /**
   * @brief 关闭串口通信
   *
   */
  void close(void) {
    printf(" uart thread exit!\n");
    carControl(0, PWMSERVOMID);
    threadRec->join();
    if (serialPort != nullptr) {
      serialPort->Close();
      serialPort = nullptr;
    }
    isOpen = false;
  }

  /**
   * @brief 串口接收校验
   *
   */
  void receiveCheck(void) {
    if (!isOpen) // 串口是否正常打开
      return;

    uint8_t resByte = 0;
    int ret = receiveBytes(resByte, 0);
    if (ret == 0) {
      if (resByte == USB_FRAME_HEAD && !serialStr.start) // 监听帧头
      {
        serialStr.start = true;                   // 开始接收数据
        serialStr.buffRead[0] = resByte;          // 获取帧头
        serialStr.buffRead[2] = USB_FRAME_LENMIN; // 初始化帧长
        serialStr.index = 1;
      } else if (serialStr.index == 2) // 接收帧的长度
      {
        serialStr.buffRead[serialStr.index] = resByte;
        serialStr.index++;
        if (resByte > USB_FRAME_LENMAX ||
            resByte < USB_FRAME_LENMIN) // 帧长错误
        {
          serialStr.buffRead[2] = USB_FRAME_LENMIN; // 重置帧长
          serialStr.index = 0;
          serialStr.start = false; // 重新监听帧长
        }
      } else if (serialStr.start &&
                 serialStr.index < USB_FRAME_LENMAX) // 开始接收数据
      {
        serialStr.buffRead[serialStr.index] = resByte; // 读取数据
        serialStr.index++;                             // 索引下移
      }

      // 帧长接收完毕
      if ((serialStr.index >= USB_FRAME_LENMAX ||
           serialStr.index >= serialStr.buffRead[2]) &&
          serialStr.index > USB_FRAME_LENMIN) // 检测是否接收完数据
      {
        uint8_t check = 0; // 初始化校验和
        uint8_t length = USB_FRAME_LENMIN;
        length = serialStr.buffRead[2]; // 读取本次数据的长度
        for (int i = 0; i < length - 1; i++)
          check += serialStr.buffRead[i]; // 累加校验和

        if (check == serialStr.buffRead[length - 1]) // 校验和相等
        {
          memcpy(serialStr.buffFinish, serialStr.buffRead,
                 USB_FRAME_LENMAX); // 储存接收的数据
          dataTransform();
        }

        serialStr.index = 0;     // 重新开始下一轮数据接收
        serialStr.start = false; // 重新监听帧头
      }
    }
  }

  /**
   * @brief 串口通信协议数据转换
   */
  void dataTransform(void) {
    switch (serialStr.buffFinish[1]) {
    case USB_ADDR_KEY: // 接收按键信息
      reach = true;
      break;

    default:
      break;
    }
  }

  /**
   * @brief 车辆速度+方向控制
   *
   * @param speed 速度：m/s
   * @param servo 方向：PWM（500~2500）
   */
  void carControl(float speed, uint16_t servo) {
    if (!isOpen)
      return;

    uint8_t buff[11];  // 多发送一个字节
    uint8_t check = 0; // 校验位
    Bit32Union bit32U;
    Bit16Union bit16U;

    buff[0] = USB_FRAME_HEAD;   // 通信帧头
    buff[1] = USB_ADDR_CARCTRL; // 地址
    buff[2] = 10;               // 帧长

    bit32U.float32 = speed; // X轴线速度
    for (int i = 0; i < 4; i++)
      buff[i + 3] = bit32U.buff[i];

    bit16U.uint16 = servo; // Y轴线速度
    buff[7] = bit16U.buff[0];
    buff[8] = bit16U.buff[1];

    for (int i = 0; i < 9; i++)
      check += buff[i];
    buff[9] = check; // 校验位

    // 循环发送数据
    for (size_t i = 0; i < 11; i++)
      transmitByte(buff[i]);
  }

  /**
   * @brief 蜂鸣器音效控制
   *
   * @param sound
   */
  void buzzerSound(Buzzer sound, float dis=1) {
    if (!isOpen)
      return;
    uint8_t buff[6];   // 多发送一个字节
    uint8_t check = 0; // 校验位

    buff[0] = USB_FRAME_HEAD;  // 帧头
    buff[1] = USB_ADDR_BUZZER; // 地址
    buff[2] = 5;               // 帧长
    switch (sound) {
    case Buzzer::BUZZER_OK: // 确认
      buff[3] = 1;
      break;
    case Buzzer::BUZZER_WARNNING: // 报警
      buff[3] = 1;
      break;
    case Buzzer::BUZZER_FINISH: // 完成
      buff[3] = 1;
      break;
    case Buzzer::BUZZER_DING: // 提示
      buff[3] = 1;
      break;
    case Buzzer::BUZZER_START: // 开机
      buff[3] = 1;
      break;
    case Buzzer::COUNT: 
      {
        buff[3] = uint8_t(dis*100);
        reach = false;
        break;
      }
    case Buzzer::NONE: 
    {
      buff[3] = 2;
      reach = false;
      break;
    }
    }

    for (size_t i = 0; i < 4; i++)
      check += buff[i];
    buff[4] = check;

    // 循环发送数据
    for (size_t i = 0; i < 6; i++)
      transmitByte(buff[i]);
  }
};
