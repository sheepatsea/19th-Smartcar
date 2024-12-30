/*
 * menu.c
 *
 *  Created on: 2024年1月25日
 *      Author: 86158
 */

/*#include "SEEKFREE_18TFT.h"
#include "SEEKFREE_FONT.h"*/
#include "menu.h"
#include "param.h"
#include "button.h"

uint16 paralist[MAXROW][10];

void para_init(void)
{
    speed_mode=0;
    Motor.kp = 300.0;
    Motor.ki = 30.0;

    paralist[0][1] = speed_mode;
    paralist[0][2] = (uint16)Motor.kp*10;
    paralist[0][3] = (uint16)Motor.ki*10;
}

void para_save(void)
{
    speed_mode = (uint8)paralist[0][1];
    Motor.kp = (float)(paralist[0][2]/10);
    Motor.ki = (float)(paralist[0][3]/10);

}
void para_change(uint8 page, uint8 row)
{
    uint8  col=1;
    num_show(page, row, col);
    while(1)
    {
        button_scan();
        switch(g_button)
        {
            case BUTTON_UP:  //增
                if(col>=3) paralist[page][row-1] += (uint16) (pow(10,col-1)+1);
                else paralist[page][row-1] += (uint16) (pow(10,col-1));
                num_show(page, row, col);
                break;
            case BUTTON_DOWN:  //减
                if(col>=3) paralist[page][row-1] -= (uint16) (pow(10,col-1)+1);
                else paralist[page][row-1] -= (uint16) (pow(10,col-1));
                num_show(page, row, col);
                break;
            case BUTTON_LEFT: //列增
                col++;
               if(col>5) col=1;
               num_show(page, row, col);
               break;
            case BUTTON_RIGHT:  //列减
                col--;
                if(col<1) col=5;
                num_show(page, row, col);
                break;
            case BUTTON_OK:  //确定
                para_save();
                return;
        }
    }
}



void page_show(uint8 page, uint8 row)
{
    //目前字符串数组只有一维
    char para[MAXROW][10] = {"START","SPEEDMODE","SPEED_P","SPEED_I"};
    switch(page)
    {
        case 1:
            for(int row_now=1;row<=4;row++)
            {
                if(row_now==row)
                {
                    lcd_writedata_16bit(0XFE19);
                    lcd_showstr(20, row_now-1, para[row_now-1]);
                    lcd_writedata_16bit(BLACK);
                }
                else  lcd_showstr(20, row_now-1, para[row_now-1]);
            }
            switch(row)
            {
                case 2:  //开、闭环
                  if(!speed_mode) lcd_showstr(20, 1, "CLOSE");
                  else lcd_showstr(20, 1, "OPEN");
                  break;
                case 3:
                  lcd_showuint16(120, 2, (int)(Motor.kp*10), 5);
                  break;
                case 4:
                  lcd_showuint16(120, 2, (int)(Motor.ki*10), 5);
                  break;
            }
            break;
    }
}


void num_show(uint8 page, uint8 row, uint8 col)
{
    char str[10];

    sprintf(str, "%05d", paralist[page][row-1]);

    lcd_writedata_16bit(0XFE19);
    lcd_showstr(60, row-1, str);
    lcd_writedata_16bit(BLACK);

    lcd_writedata_16bit(0x07E0);
    lcd_showstr(60+8*(5-col), row-1, str[5-col]);
    lcd_writedata_16bit(BLACK);
}

