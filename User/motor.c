#include "motor.h"

//GPIO方向引脚初始化
static void Motor_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

	//关闭JTAG，释放PA15、PB3、PB4
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

	//PA12 PA15：M1 IN1 IN2
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_15;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStruct);

	//PB0 PB1：M2 IN1 IN2
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_Init(GPIOB, &GPIO_InitStruct);

	//PB8 PB9：M3 IN1 IN2
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_Init(GPIOB, &GPIO_InitStruct);

	//PB14 PB15：M4 IN1 IN2
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
}

//TIM1 PWM初始化 CH1 PA8 CH2 PA9 CH3 PA10 CH4 PA11
static void Motor_TIM1_PWM_Init(void)
{
	GPIO_InitTypeDef        GPIO_InitStruct;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
	TIM_OCInitTypeDef       TIM_OCInitStruct;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_TIM1, ENABLE);

	//PA8 PA9 PA10 PA11 复用推挽输出PWM
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStruct);

	//72M /(0+1)/(3599+1) =20KHz
	TIM_TimeBaseStruct.TIM_Prescaler = 0;
	TIM_TimeBaseStruct.TIM_Period = 3599;
	TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStruct.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStruct);

	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStruct.TIM_Pulse = 0;
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;

	TIM_OC1Init(TIM1, &TIM_OCInitStruct);
	TIM_OC2Init(TIM1, &TIM_OCInitStruct);
	TIM_OC3Init(TIM1, &TIM_OCInitStruct);
	TIM_OC4Init(TIM1, &TIM_OCInitStruct);

	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);

	TIM_CtrlPWMOutputs(TIM1, ENABLE); //高级定时器必须开启主输出MOE
	TIM_Cmd(TIM1, ENABLE);
}

void Motor_Init(void)
{
	Motor_GPIO_Init();
	Motor_TIM1_PWM_Init();
}

//设置PWM占空比 0~3599
void M1_SetPWM(u16 duty){ TIM_SetCompare1(TIM1, duty); }
void M2_SetPWM(u16 duty){ TIM_SetCompare2(TIM1, duty); }
void M3_SetPWM(u16 duty){ TIM_SetCompare3(TIM1, duty); }
void M4_SetPWM(u16 duty){ TIM_SetCompare4(TIM1, duty); }

//左前M1 PA12(IN1) PA15(IN2)
void M1_SetDir(MotorDir_t dir)
{
	switch(dir)
	{
	case MOTOR_FORWARD:	GPIO_SetBits(GPIOA,GPIO_Pin_12); GPIO_ResetBits(GPIOA,GPIO_Pin_15); break;
	case MOTOR_BACKWARD:GPIO_ResetBits(GPIOA,GPIO_Pin_12); GPIO_SetBits(GPIOA,GPIO_Pin_15); break;
	case MOTOR_COAST:	GPIO_ResetBits(GPIOA,GPIO_Pin_12); GPIO_ResetBits(GPIOA,GPIO_Pin_15); break;
	case MOTOR_STOP:	GPIO_SetBits(GPIOA,GPIO_Pin_12); GPIO_SetBits(GPIOA,GPIO_Pin_15); break;
	}
}

//左后M2 PB0(IN1) PB1(IN2)
void M2_SetDir(MotorDir_t dir)
{
	switch(dir)
	{
	case MOTOR_FORWARD:	GPIO_SetBits(GPIOB,GPIO_Pin_0); GPIO_ResetBits(GPIOB,GPIO_Pin_1); break;
	case MOTOR_BACKWARD:GPIO_ResetBits(GPIOB,GPIO_Pin_0); GPIO_SetBits(GPIOB,GPIO_Pin_1); break;
	case MOTOR_COAST:	GPIO_ResetBits(GPIOB,GPIO_Pin_0); GPIO_ResetBits(GPIOB,GPIO_Pin_1); break;
	case MOTOR_STOP:	GPIO_SetBits(GPIOB,GPIO_Pin_0); GPIO_SetBits(GPIOB,GPIO_Pin_1); break;
	}
}

//右前M3 PB8(IN1) PB9(IN2)
void M3_SetDir(MotorDir_t dir)
{
	switch(dir)
	{
	case MOTOR_FORWARD:	GPIO_SetBits(GPIOB,GPIO_Pin_8); GPIO_ResetBits(GPIOB,GPIO_Pin_9); break;
	case MOTOR_BACKWARD:GPIO_ResetBits(GPIOB,GPIO_Pin_8); GPIO_SetBits(GPIOB,GPIO_Pin_9); break;
	case MOTOR_COAST:	GPIO_ResetBits(GPIOB,GPIO_Pin_8); GPIO_ResetBits(GPIOB,GPIO_Pin_9); break;
	case MOTOR_STOP:	GPIO_SetBits(GPIOB,GPIO_Pin_8); GPIO_SetBits(GPIOB,GPIO_Pin_9); break;
	}
}

//右后M4 PB14(IN1) PB15(IN2)
void M4_SetDir(MotorDir_t dir)
{
	switch(dir)
	{
	case MOTOR_FORWARD:	GPIO_SetBits(GPIOB,GPIO_Pin_14); GPIO_ResetBits(GPIOB,GPIO_Pin_15); break;
	case MOTOR_BACKWARD:GPIO_ResetBits(GPIOB,GPIO_Pin_14); GPIO_SetBits(GPIOB,GPIO_Pin_15); break;
	case MOTOR_COAST:	GPIO_ResetBits(GPIOB,GPIO_Pin_14); GPIO_ResetBits(GPIOB,GPIO_Pin_15); break;
	case MOTOR_STOP:	GPIO_SetBits(GPIOB,GPIO_Pin_14); GPIO_SetBits(GPIOB,GPIO_Pin_15); break;
	}
}
