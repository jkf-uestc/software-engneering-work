#ifndef __KEY_H
#define __KEY_H

#include "stm32f10x.h"


#define KEY_GPIO_PORT     GPIOA
#define KEY_GPIO_PIN      GPIO_Pin_5

//滑动窗口，多数表决
#define KEY_WIN_SIZE 4
#define KEY_THRESHOLD 3 

#define KEY_SCAN_MS 5  //采样周期下限
#define KEY_LONG_PRESS_MS 3000 //长按3s强制归零

#define KEY_GEAR_MAX 3  //档位上限

void    Key_Init(void);         
void    Key_Task(void);         

uint8_t Key_GetGear(void);      
int16_t Key_GetSpeed(void);     


#endif
