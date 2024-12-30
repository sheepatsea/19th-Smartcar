/*
 * param.c
 *
 *  Created on: 2024Äê1ÔÂ27ÈÕ
 *      Author: 86158
 */

#include "param.h"

//Servo
uint16 Servo = MIDSERVO;

//uart
uint8 usb_receiveStart = 0;
uint8 usb_receiveBuff[USB_FRAME_LENMAX] = {0};
uint8 usb_receiveIndex = 0;
uint8 usb_receiveBuffFinished[USB_FRAME_LENMAX] = {0};
uint8 usb_receiveFinished = 0;

//motor
uint8 speed_mode=0;
SPEED_PID Motor;

//button
uint8 g_button=0;

//buzzer
uint32 g_time1=0;
uint8 g_beep_state=0;
uint8 g_beep_set=0;

uint32 uart_wait=0;

uint8 count_mode = 0;
float count_dis = 0;
float exp_dis;
