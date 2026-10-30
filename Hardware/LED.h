//封装LED的驱动程序
//存放驱动函数可以对外提供的函数和变量声明

#ifndef __LED_H   //if no define
#define __LED_H

void LED_Init(void);
void LED1_ON(void);
void LED1_OFF(void);
void LED2_ON(void);
void LED2_OFF(void);
void LED1_Turn(void);
void LED2_Turn(void);
#endif
