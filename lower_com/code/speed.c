/*
 * speed.c
 *
 *  Created on: 2024年1月25日
 *      Author: 86158
 */


#include "speed.h"
#include"param.h"

float speed[COUNTAVERAGE]={0};  //速度，平均
long encoder_num[COUNTAVERAGE]={0};  //脉冲数， 平均

void Speed_Error_Get(void)
{
    float sp_sum=0;
    long en_sum=0;
    //数据左移
    for(int i=COUNTAVERAGE-1;i>0;i--)
    {
        speed[i] = speed[i-1];
        encoder_num[i] = encoder_num[i-1];
    }
    //读取脉冲，计算速度
    Encoder_Read();
    speed[0] = (float) (encoder_num[0]) / (float) (OM_ENCODER * PERIOD);

    //均值滤波
    for(int i=0;i<COUNTAVERAGE;i++)
    {
        sp_sum += speed[i];
        en_sum += encoder_num[i];
    }
    Motor.encoder_num = (long) (en_sum /COUNTAVERAGE);
    Motor.speed = sp_sum / COUNTAVERAGE;
    //计算误差
    Motor.error = (float) (Motor.exp_speed * PERIOD *  OM_ENCODER - Motor.encoder_num);



}

void Speed_Init(void) //给电机初始占空比
{
    gtm_pwm_init(MOTOR_1, MOTOR_FREQ, HALF_MAX_DUTY);
    gtm_pwm_init(MOTOR_2, MOTOR_FREQ, HALF_MAX_DUTY);

}


void Speed_Output(void)  //输出电机占空比
{
    if(speed_mode==0)
    {
        pwm_duty(MOTOR_1,  HALF_MAX_DUTY+Motor.output/2);
        pwm_duty(MOTOR_2,  HALF_MAX_DUTY-Motor.output/2);
    }
    else
    {
        pwm_duty(MOTOR_1,  HALF_MAX_DUTY);
        pwm_duty(MOTOR_2,  HALF_MAX_DUTY);
    }

}

void Speed_Control(void)  //右后轮速度闭环
{

    static float P_out=0;
    static float I_out=0;

    P_out += Motor.kp * (Motor.error - Motor.last_error);
    I_out += Motor.ki * Motor.error;

    if(I_out>MOTOR_PWM_MAX) I_out=MOTOR_PWM_MAX;
    else if(I_out<MOTOR_PWM_MIN) I_out = MOTOR_PWM_MIN;

    Motor.output = (long) (P_out + I_out);
    if(Motor.output>MOTOR_PWM_MAX) Motor.output=MOTOR_PWM_MAX;
    else if(Motor.output<MOTOR_PWM_MIN) Motor.output = MOTOR_PWM_MIN;

    Motor.last_error = Motor.error;
}

void Encoder_Init(void)  //编码器初始化
{
    gpt12_init(GPT12_T5, GPT12_T5INB_P10_3, GPT12_T5EUDB_P10_1);
}

void Encoder_Read(void)
{
    encoder_num[0] = gpt12_get(GPT12_T5);
    gpt12_clear(GPT12_T5);
}

