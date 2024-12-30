/*
 * uart.c
 *
 *  Created on: 2024骞�1鏈�25鏃�
 *      Author: 86158
 */

#include "headfile.h"
#include "uart.h"
#include "servo.h"
#include "param.h"




void uart_send()
{
  /*  Bint16_Union bint16_Union;
    unsigned char sendBuff[150];
    unsigned char check = 0;

    sendBuff[0] = 0x42; // 鐢冦仈
    sendBuff[1] = 0x06; // 閸︽澘娼�
    sendBuff[2] = 132;   // 鐢囨毐
    for(int i = 0; i < 64; i ++)
    {
        bint16_Union.U16 = data_all[i];
        sendBuff[2 * i + 3] = bint16_Union.U8_Buff[0];
        sendBuff[2 * i + 4] = bint16_Union.U8_Buff[1];
//        sendBuff[2 * i + 3] = (data_all[i] << 8) >> 8;
//        sendBuff[2 * i + 4] = data_all[i] >> 8;
    }

    for (int i = 0; i < 131; i++)
    {
      check += sendBuff[i];
    }
    sendBuff[131] = check;

    // 瀵邦亞骞嗛崣鎴︹偓浣规殶閹癸拷
    for (int i = 0; i < 132; i++)
    {
        uart_putchar(UART_CHANNEL, sendBuff[i]);
    }*/
}

void uart_get()
{
    uint8 Uart1Res;
    if(uart_query(UART_CHANNEL, &Uart1Res))
    {
        if(Uart1Res == USB_FRAME_HEAD && !usb_receiveStart)
        {
            usb_receiveStart = 1;
            usb_receiveBuff[0] = Uart1Res;
            usb_receiveBuff[2] = USB_FRAME_LENMIN;
            usb_receiveIndex = 1;
        }
        else if(usb_receiveIndex == 2)
        {
            usb_receiveBuff[usb_receiveIndex] = Uart1Res;
            usb_receiveIndex++;

            if(Uart1Res > USB_FRAME_LENMAX || Uart1Res < USB_FRAME_LENMIN)
            {
                usb_receiveBuff[2] = USB_FRAME_LENMIN;
                usb_receiveIndex = 0;
                usb_receiveStart = 0;
            }
        }
        else if(usb_receiveStart && usb_receiveIndex < USB_FRAME_LENMAX)
        {
            usb_receiveBuff[usb_receiveIndex] = Uart1Res;
            usb_receiveIndex++;
        }

        if((usb_receiveIndex >= USB_FRAME_LENMAX || usb_receiveIndex >= usb_receiveBuff[2]) && usb_receiveIndex > USB_FRAME_LENMIN)
        {
            uint8 check = 0;
            uint8 length = USB_FRAME_LENMIN;

            length = usb_receiveBuff[2];
            for(int i = 0; i < length-1; i ++)
                check += usb_receiveBuff[i];

            if(check == usb_receiveBuff[length-1])
            {
                memcpy(usb_receiveBuffFinished, usb_receiveBuff, USB_FRAME_LENMAX);
                usb_receiveFinished = 1;

                if(USB_ADDR_CONTROL  == usb_receiveBuffFinished[1])
                {
                    Bint16_Union bint16_Union;
                    Bint32_Union bint32_Union;
                    for(int i = 0; i < 4; i ++)
                        bint32_Union.U8_Buff[i] = usb_receiveBuffFinished[3+i];

                    bint16_Union.U8_Buff[0] = usb_receiveBuffFinished[7];
                    bint16_Union.U8_Buff[1] = usb_receiveBuffFinished[8];

                    Servo = bint16_Union.U16;
                    Motor.exp_speed = bint32_Union.Float;
                }
            }

            usb_receiveIndex = 0;
            usb_receiveStart = 0;
        }
    }
}

void uart_getvofa()
{
    uint8 uart_get;
    if(uart_query(UART_CHANNEL, &uart_get))
    {
        Motor.exp_speed = (float)uart_get * 0.1;
    }
}

void USB_Edgeboard_Handle(void)
{
    if(usb_receiveFinished)
    {
        usb_receiveFinished = 0;
        switch(usb_receiveBuffFinished[1])
        {
                case USB_ADDR_BUZZER :
                    if(usb_receiveBuffFinished[3] == 1)          //OK
                        g_beep_set = 1;
//                    else if(usb_receiveBuffFinished[3] == 2)     //Warnning
//                        g_beep_set = 2;
//                    else if(usb_receiveBuffFinished[3] == 3)     //Finish
//                        g_beep_set = 3;
//                    else if(usb_receiveBuffFinished[3] == 4)     //Ding
//                        g_beep_set = 4;
//                    else if(usb_receiveBuffFinished[3] == 5)     //SystemStart
//                        g_beep_set = 6;
                    else if(usb_receiveBuffFinished[3] == 2)
                        {g_beep_set = 1;count_mode = NONE; count_dis = 0;}
                    else
                    {
                       if(count_mode==NONE) {g_beep_set = 1;count_mode = COUNT; count_dis = 0;  exp_dis = (float)(usb_receiveBuffFinished[3])/100;}
                    }
                    break;
        }
    }
}

void USB_Edgeboard_TransmitKey(void)
{
    uint8 check = 0;
    uint8 buff[8];
    Bint16_Union bint16_Union;

    buff[0] = 0x42;
    buff[1] = USB_ADDR_KEYINPUT;
    buff[2] = 0x06;

    bint16_Union.U16 = 115;
    buff[3] = bint16_Union.U8_Buff[0];
    buff[4] = bint16_Union.U8_Buff[1];

    for(int i=0;i<5;i++)
        check += buff[i];

    buff[5] = check;

    for(int i=0;i<8;i++)
        seekfree_wireless_send_buff(&buff[i], 1);
}
