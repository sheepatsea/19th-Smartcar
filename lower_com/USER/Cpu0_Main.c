/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ����Ⱥ��824575535
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		main
 * @company	   		�ɶ���ɿƼ����޹�˾
 * @author     		��ɿƼ�(QQ3184284598)
 * @version    		�鿴doc��version�ļ� �汾˵��
 * @Software 		ADS v1.2.2
 * @Target core		TC264D
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2020-3-23
 ********************************************************************************************************************/


#include "headfile.h"
#include "uart.h"
#include "servo.h"
#include "speed.h"
#include "button.h"
#include "SEEKFREE_WIRELESS.h"
#include "param.h"
#include "menu.h"
#pragma section all "cpu0_dsram"

#define BEEP_PIN P33_10
//���������#pragma section all restore���֮���ȫ�ֱ���������CPU0��RAM��


//���̵��뵽���֮��Ӧ��ѡ�й���Ȼ����refreshˢ��һ��֮���ٱ���
//����Ĭ������Ϊ�ر��Ż��������Լ��һ�����ѡ��properties->C/C++ Build->Setting
//Ȼ�����Ҳ�Ĵ������ҵ�C/C++ Compiler->Optimization->Optimization level�������Ż��ȼ�
//һ��Ĭ���½����Ĺ��̶���Ĭ�Ͽ�2���Ż�����˴��Ҳ��������Ϊ2���Ż�

//����TCϵ��Ĭ���ǲ�֧���ж�Ƕ�׵ģ�ϣ��֧���ж�Ƕ����Ҫ���ж���ʹ��enableInterrupts();�������ж�Ƕ��
//�򵥵�˵ʵ���Ͻ����жϺ�TCϵ�е�Ӳ���Զ�������disableInterrupts();���ܾ���Ӧ�κε��жϣ������Ҫ�����Լ��ֶ�����enableInterrupts();�������жϵ���Ӧ��
int core0_main(void)
{
	get_clk();//��ȡʱ��Ƶ��  ��ر���

	seekfree_wireless_init();

	para_init();
    button_init();

    Servo_Init();

    Speed_Init();
    Encoder_Init();

    pit_init(CCU6_0, PIT_CH0, 1000);//���������ж�1ms

    //������
    gpio_init(BEEP_PIN, GPO, 0, PUSHPULL);

  /*  lcd_init();
    lcd_clear(WHITE);*/

    IfxCpu_emitEvent(&g_cpuSyncEvent);
	IfxCpu_waitEvent(&g_cpuSyncEvent, 0xFFFF);
	enableInterrupts();
/*	int servo=1890;
	while(1)
	{
	    switch(g_button)
	    {
	        case BUTTON_LEFT:
	            servo += 10;
	            break;
	        case BUTTON_RIGHT:
	            servo-=10;
	            break;
	    }
	    pwm_duty(SERVOPIN, servo);
	}
*/
	uint8 page=1, row=1;
    while(1)
    {
//        Motor.exp_speed=2.0;
        //Servo = MAXSERVO;
       // Motor.exp_speed = 1.6;
        switch(g_button)
        {
            case(BUTTON_LEFT):
                 page--;
                 if(page<1) page = MAXPAGE;
                 page_show(page, row);
                 break;
            case(BUTTON_RIGHT):
                 page++;
                 if(page>MAXPAGE) page = 1;
                 page_show(page, row);
                 break;
            case(BUTTON_UP):
                 row--;
                 if(row<1) row = MAXROW;
                 page_show(page, row);
                 break;
            case(BUTTON_DOWN):
                row++;
                if(row>MAXROW) row=1;
                page_show(page, row);
                break;
            case(BUTTON_OK):
                if(page==1)
                {
                    switch(row)
                    {
                        case 1:
                           // USB_Edgeboard_TransmitKey();
                            break;
                        case 2:
                            speed_mode = !speed_mode;
                            break;
                    }

                }
                else para_change(page, row);
                page_show(page, row);
                break;
        }
    }
    return 0;
}

#pragma section all restore


