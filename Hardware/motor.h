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
 // speed范围：0 ~ 3599
 void Car_GoStraight(uint16_t speed);    //直行
 void Car_GoBack(uint16_t speed);        //后退
 void Car_Stop(void);                    //全部刹车停止
 void Car_TurnLeft(uint16_t speed);      //原地左转（左轮反向，右轮前进）
 void Car_TurnRight(uint16_t speed);     //原地右转（右轮反向，左轮前进）
 void Car_SpinLeft(uint16_t speed);      //差速左转（左轮慢，右轮快，不原地打转）
 void Car_SpinRight(uint16_t speed);     //差速右转


#endif
