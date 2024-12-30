/*
 * buzzer.c
 *
 *  Created on: 2024��1��27��
 *      Author: 86158
 */

#include "buzzer.h"
#include "headfile.h"

#define BEEP_PIN    P33_10

// ms
#define BEEP_TIME_REST    50
#define BEEP_TIME_SHORT   100
#define BEEP_TIME_LONG    250
#define BEEP_TIME_INTER   100
#define BEEP_TIME_WAIT    100
#define BEEP_TIME_ONCE    25
#define BEEP_TIME_LOW1    2
#define BEEP_TIME_LOW2    5

#define BEEP_HURRY_N      3
#define BEEP_DECAY_N      7
#define BEEP_LOW_N        80

static uint32 beep_clock = 0;
static uint16 beep_count = 0;

// void beep_init()
// {
//     gpio_init(BEEP_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
// }

void buzzer_spin()
{
    static uint32 beep_time = 0;
    // ������÷����������˳�
    // if (!g_beep)
    // {
    //     gpio_set(BEEP_PIN, GPIO_LOW);
    //     return;
    // }

    // δ������ʱ�䲻����
    if (g_time1 < beep_time)
    {
        return;
    }

    // ������÷�����ģʽ���ϴα����ѽ���������������״̬��
    if (g_beep_set && !g_beep_state)
    {
        switch (g_beep_set)
        {
        case BEEP_SHORT:
            g_beep_state = 1;
            beep_clock = g_time1 + BEEP_TIME_SHORT;
            g_beep_set = 0;
            break;
        case BEEP_LONG:
            g_beep_state = 1;
            beep_clock = g_time1 + BEEP_TIME_LONG;
            g_beep_set = 0;
            break;
        case BEEP_DOUBLE:
            g_beep_state = 2;
            beep_clock = g_time1 + BEEP_TIME_SHORT;
            g_beep_set = 0;
            break;
        case BEEP_HURRY:
            g_beep_state = 4;
            beep_clock = g_time1 + BEEP_TIME_REST;
            beep_count = 2 * BEEP_HURRY_N - 1;
            g_beep_set = 0;
            break;
        case BEEP_DECAY:
            g_beep_state = 5;
            beep_clock = g_time1 + BEEP_DECAY_N;
            beep_count = 2 * BEEP_DECAY_N - 1;
            g_beep_set = 0;
            break;
        case BEEP_ONCE:
            g_beep_state = 1;
            beep_clock = g_time1 + BEEP_TIME_ONCE;
            g_beep_set = 0;
            break;
        case BEEP_LOW:
            g_beep_state = 6;
            beep_count = BEEP_LOW_N * 2 - 1;
            beep_clock = g_time1 + BEEP_TIME_LOW1;
            g_beep_set = 0;
            break;
        default:
            break;
        }
    }

    // ����
    if (g_beep_state == 1)
    {
        if (g_time1 < beep_clock)
        {
            gpio_set(BEEP_PIN, GPIO_HIGH);
        }
        else
        {
            g_beep_state = 0;
            gpio_set(BEEP_PIN, GPIO_LOW);
            beep_time = g_time1 + BEEP_TIME_WAIT;
        }
    }
    // ������ʱ
    else if (g_beep_state == 2)
    {
        if (g_time1 < beep_clock)
        {
            gpio_set(BEEP_PIN, GPIO_HIGH);
        }
        else
        {
            g_beep_state = 3;
            beep_clock = g_time1 + BEEP_TIME_INTER;
            gpio_set(BEEP_PIN, GPIO_LOW);
        }
    }
    // ������ʱ
    else if (g_beep_state == 3)
    {
        if (g_time1 < beep_clock)
        {
            gpio_set(BEEP_PIN, GPIO_LOW);
        }
        else
        {
            g_beep_state = 1;
            beep_clock = g_time1 + BEEP_TIME_SHORT;
            gpio_set(BEEP_PIN, GPIO_HIGH);
        }
    }
    // ���ٶ���
    else if (g_beep_state == 4)
    {
        if (g_time1 < beep_clock)
        {
            if (beep_count % 2)
            {
                gpio_set(BEEP_PIN, GPIO_HIGH);
            }
            else
            {
                gpio_set(BEEP_PIN, GPIO_LOW);
            }
        }
        else
        {
            beep_count--;
            if (!beep_count)
            {
                g_beep_state = 0;
                gpio_set(BEEP_PIN, GPIO_LOW);
                beep_time = g_time1 + BEEP_TIME_WAIT;
            }
            else
            {
                beep_clock = g_time1 + BEEP_TIME_REST;
            }
        }
    }
    // ����˥����Ч
    else if (g_beep_state == 5)
    {
        if (g_time1 < beep_clock)
        {
            if (beep_count % 2)
            {
                gpio_set(BEEP_PIN, GPIO_HIGH);
            }
            else
            {
                gpio_set(BEEP_PIN, GPIO_LOW);
            }
        }
        else
        {
            beep_count--;
            if (!beep_count)
            {
                g_beep_state = 0;
                gpio_set(BEEP_PIN, GPIO_LOW);
                beep_time = g_time1 + BEEP_TIME_WAIT;
            }
            else
            {
                beep_clock = g_time1 + beep_count;
            }
        }
    }
    // ����ģʽ
    else if (g_beep_state == 6)
    {
        if (g_time1 < beep_clock)
        {
            if (beep_count % 2)
            {
                gpio_set(BEEP_PIN, GPIO_HIGH);
            }
            else
            {
                gpio_set(BEEP_PIN, GPIO_LOW);
            }
        }
        else
        {
            beep_count--;
            if (!beep_count)
            {
                g_beep_state = 0;
                gpio_set(BEEP_PIN, GPIO_LOW);
                beep_time = g_time1 + BEEP_TIME_WAIT;
            }
            else
            {
                if (beep_count % 2)
                {
                    beep_clock = g_time1 + BEEP_TIME_LOW1;
                }
                else
                {
                    beep_clock = g_time1 + BEEP_TIME_LOW2;
                }
            }
        }
    }
}
