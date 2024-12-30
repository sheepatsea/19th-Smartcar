/*
 * param.h
 *
 *  Created on: 2024年1月25日
 *      Author: 86158
 */

#ifndef CODE_PARAM_H_
#define CODE_PARAM_H_


#include "headfile.h"
//舵机 180° 2200 400 - 2600
#define MIDSERVO 1625
#define MAXSERVO 2000  //1980
#define MINSERVO 1230     //1250
#define SERVOPIN ATOM3_CH3_P33_7 //ATOM0_CH1_P33_9
#define SERVO_FREQ 100
extern uint16 Servo;


#define BUZZER  P33_10//蜂鸣器

#define MaxLen 10

//速度
#define MOTOR_1 ATOM0_CH5_P02_5
#define MOTOR_2 ATOM0_CH4_P02_4
#define MOTOR_FREQ 15000
#define HALF_MAX_DUTY  5000
#define  MOTOR_PWM_MAX              7000
#define  MOTOR_PWM_MIN              -7000


#define OM_ENCODER 6773
#define PERIOD 0.001  //1ms
#define COUNTAVERAGE 5

typedef struct {
        float kp;
        float ki;
        float error;
        float last_error;
        long output;  //输出占空比
        float speed;  //实际速度
        float exp_speed;
        long encoder_num;  //脉冲数
}SPEED_PID;
extern SPEED_PID Motor;
extern uint8 speed_mode;
//按键

#define KEY_UP   (P33_13)
#define KEY_DOWN  ( P22_0)
#define KEY_LEFT    (P22_2)
#define KEY_RIGHT     (P22_1)
#define KEY_OK    (P33_12)

#define SWITCH1                 (P33_11)
#define SWITCH2                 (P33_12)

#define BUTTON_NONE 0
#define BUTTON_UP 1
#define BUTTON_DOWN 2
#define BUTTON_LEFT 3
#define BUTTON_RIGHT 4
#define BUTTON_OK 5

extern uint8 g_button;


//menu
#define MAXPAGE 2
#define MAXROW  7


//uart

#define UART_CHANNEL UART_2
#define UART_TX_PIT  UART2_TX_P02_0
#define UART_RX_PIT  UART2_RX_P02_1

#define USB_FRAME_HEAD               0x42
#define USB_FRAME_LENMIN             4
#define USB_FRAME_LENMAX             30

#define USB_ADDR_HEART               0x00
#define USB_ADDR_CONTROL             0x01
#define USB_ADDR_SPEEDMODE           0x02
#define USB_ADDR_SERVOTHRESHOLD      0x03鍊�
#define USB_ADDR_BUZZER              0x04
#define USB_ADDR_LIGHT               0x05
#define USB_ADDR_KEYINPUT            0x06
#define USB_ADDR_BATTERY             0x07



#define uartchannel_init()      uart_init(UART_CHANNEL, 115200, UART_TX_PIT, UART_RX_PIT)
//#define uart_get()              uart_query(UART_CHANNEL,&uart_get_buff[0])

typedef union
{
    uint8 U8_Buff[2];
    uint16 U16;
    int16 S16;
}Bint16_Union;

typedef union
{
    uint8 U8_Buff[4];
    float Float;
    unsigned long U32;
}Bint32_Union;

extern uint8 usb_receiveStart;
extern uint8 usb_receiveBuff[USB_FRAME_LENMAX];
extern uint8 usb_receiveIndex;
extern uint8 usb_receiveBuffFinished[USB_FRAME_LENMAX];
extern uint8 usb_receiveFinished;
extern uint32 g_time1;
extern uint8 g_beep_state;
extern uint8 g_beep_set;

extern uint32 uart_wait;

#define NONE 0
#define COUNT 1

extern uint8 count_mode;
extern float count_dis;
extern float exp_dis;
#endif /* CODE_PARAM_H_ */
