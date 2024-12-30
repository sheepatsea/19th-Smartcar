/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：三群：824575535
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file       		main
 * @company	   		成都逐飞科技有限公司
 * @author     		逐飞科技(QQ3184284598)
 * @version    		查看doc内version文件 版本说明
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
//将本语句与#pragma section all restore语句之间的全局变量都放在CPU0的RAM中


//工程导入到软件之后，应该选中工程然后点击refresh刷新一下之后再编译
//工程默认设置为关闭优化，可以自己右击工程选择properties->C/C++ Build->Setting
//然后在右侧的窗口中找到C/C++ Compiler->Optimization->Optimization level处设置优化等级
//一般默认新建立的工程都会默认开2级优化，因此大家也可以设置为2级优化

//对于TC系列默认是不支持中断嵌套的，希望支持中断嵌套需要在中断内使用enableInterrupts();来开启中断嵌套
//简单点说实际上进入中断后TC系列的硬件自动调用了disableInterrupts();来拒绝响应任何的中断，因此需要我们自己手动调用enableInterrupts();来开启中断的响应。
int core0_main(void)
{
	get_clk();//获取时钟频率  务必保留

	seekfree_wireless_init();

	para_init();
    button_init();

    Servo_Init();

    Speed_Init();
    Encoder_Init();

    pit_init(CCU6_0, PIT_CH0, 1000);//设置周期中断1ms

    //蜂鸣器
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


