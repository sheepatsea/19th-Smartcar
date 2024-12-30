/*
 * button.c
 *
 *  Created on: 2024年1月25日
 *      Author: 86158
 */


#include"button.h"
#include"zf_gpio.h"
#include"param.h"


void button_init(void)
{
    gpio_init(KEY_UP, GPI, GPIO_HIGH, PULLUP);           // 初始化 KEY1 输入 默认高电平 上拉输入
    gpio_init(KEY_DOWN, GPI, GPIO_HIGH, PULLUP);           // 初始化 KEY2 输入 默认高电平 上拉输入
    gpio_init(KEY_LEFT, GPI, GPIO_HIGH, PULLUP);           // 初始化 KEY3 输入 默认高电平 上拉输入
    gpio_init(KEY_RIGHT, GPI, GPIO_HIGH, PULLUP);           // 初始化 KEY4 输入 默认高电平 上拉输入
    gpio_init(KEY_OK, GPI, GPIO_HIGH, PULLUP);           // 初始化 KEY5 输入 默认高电平 上拉输入

   // gpio_init(SWITCH1, GPI, GPIO_HIGH, GPI_FLOATING_IN);    // 初始化 SWITCH1 输入 默认高电平 浮空输入
  //  gpio_init(SWITCH2, GPI, GPIO_HIGH, GPI_FLOATING_IN);    // 初始化 SWITCH2 输入 默认高电平 浮空输入
}


void button_scan(void)
{
    static uint8 key = 0;

    key<<=4;

    if(!gpio_get(KEY_UP))
    {
        key += BUTTON_UP;
    }
    else if(!gpio_get(KEY_DOWN))
    {
        key += BUTTON_DOWN;
    }
    else if(!gpio_get(KEY_LEFT))
    {
        key += BUTTON_LEFT;
    }
    else if(!gpio_get(KEY_RIGHT))
    {
        key += BUTTON_RIGHT;
    }
    else if(!gpio_get(KEY_OK))
    {
        key += BUTTON_OK;
    }

    g_button = (key&0x0f)? BUTTON_NONE: (key>>4);

}

