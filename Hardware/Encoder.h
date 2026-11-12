#ifndef __ENCODER_H
#define __ENCODER_H

#include "stm32f10x.h"

// 测速参数
#define PULSE_PER_REV   1320.0f
#define WHEEL_D_CM      6.5f
#define SAMPLE_T        0.02f

extern float v_LF;
extern float v_LB;
extern float v_RF;
extern float v_RR;
extern float g_speed_coeff;

// 系统节拍标志
extern volatile uint32_t g_tick;
extern volatile uint8_t  flag_20ms;

void TIM2_Encoder_Init(void);
void TIM3_Encoder_Init(void);
void TIM4_Encoder_Init(void);
void EXTI_RR_Encoder_Init(void);
void Speed_Calc_20ms(void);

#endif
