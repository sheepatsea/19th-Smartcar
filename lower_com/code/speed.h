/*
 * speed.h
 *
 *  Created on: 2024年1月25日
 *      Author: 86158
 */

#ifndef CODE_SPEED_H_
#define CODE_SPEED_H_

#include "headfile.h"

void Speed_Output(void); //输出
void Speed_Init(void);  //PWM初始化
void Speed_Control(void);
void Encoder_Init(void);
void Speed_Error_Get(void);
void Encoder_Read(void);





#endif /* CODE_SPEED_H_ */
