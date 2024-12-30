/*
 * buzzer.h
 *
 *  Created on: 2024Äê1ÔÂ27ÈÕ
 *      Author: 86158
 */

#ifndef CODE_BUZZER_H_
#define CODE_BUZZER_H_

#include "common.h"
#include "headfile.h"
#include "param.h"

//·äÃùÆ÷
#define BEEP_SHORT  1
#define BEEP_LONG   2
#define BEEP_DOUBLE 3
#define BEEP_HURRY  4
#define BEEP_DECAY  5
#define BEEP_ONCE   6
#define BEEP_LOW    7

void buzzer_spin();

#endif /* CODE_BUZZER_H_ */
