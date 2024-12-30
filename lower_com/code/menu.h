/*
 * menu.h
 *
 *  Created on: 2024Äê1ÔÂ25ÈÕ
 *      Author: 86158
 */

#ifndef CODE_MENU_H_
#define CODE_MENU_H_

#include "headfile.h"

void para_init(void);
void para_save(void);
void para_change(uint8 page, uint8 row);
void page_show(uint8 page, uint8 row);
void num_show(uint8 page, uint8 row, uint8 col);
#endif /* CODE_MENU_H_ */
