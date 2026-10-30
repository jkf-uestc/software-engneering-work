#ifndef __MOTOR_H
#define __MOTOR_H
#include "stm32f10x.h"

//PWM占空比范围：0 ~ 3599
void Motor_Init(void);

//四个电机设置占空比
void M1_SetPWM(u16 duty);	//左前
void M2_SetPWM(u16 duty);	//左后
void M3_SetPWM(u16 duty);	//右前
void M4_SetPWM(u16 duty);	//右后

//电机方向
typedef enum
{
	MOTOR_STOP,		//刹车 IN1=1 IN2=1
	MOTOR_FORWARD,	//正转 IN1=1 IN2=0
	MOTOR_BACKWARD,	//反转 IN1=0 IN2=1
	MOTOR_COAST		//滑行 IN1=0 IN2=0
}MotorDir_t;

void M1_SetDir(MotorDir_t dir);
void M2_SetDir(MotorDir_t dir);
void M3_SetDir(MotorDir_t dir);
void M4_SetDir(MotorDir_t dir);

#endif
