/*
 * servo.c
 *
 *  Created on: 2024��1��25��
 *      Author: 86158
 */


#include "servo.h"
#include "param.h"


void Servo_Init(void)
{
    gtm_pwm_init(SERVOPIN, SERVO_FREQ, MIDSERVO);
}

void Servo_Output(void)
{
    if(Servo>MAXSERVO) Servo = MAXSERVO;
    else if(Servo<MINSERVO) Servo = MINSERVO;
    pwm_duty(SERVOPIN, Servo);
}


