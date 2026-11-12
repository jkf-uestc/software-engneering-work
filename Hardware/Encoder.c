#include "encoder.h"

volatile uint32_t g_tick = 0;
volatile uint8_t  flag_20ms = 0;

// 硬件编码器历史计数值，仅本文件内使用
static int16_t last_LF = 0;
static int16_t last_LB = 0;
static int16_t last_RF = 0;

// 右后软件编码器
volatile int32_t g_pulse_RR = 0;
static int32_t   last_RR = 0;
static uint8_t   old_state = 0;

// 对外速度结果
float v_LF = 0.0f;
float v_LB = 0.0f;
float v_RF = 0.0f;
float v_RR = 0.0f;

float g_speed_coeff;


// TIM2 左前编码器 PA0 PA1
void TIM2_Encoder_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
    TIM_ICInitTypeDef TIM_ICInitStruct;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    TIM_TimeBaseStruct.TIM_Prescaler = 0;
    TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStruct.TIM_Period = 65535;
    TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStruct.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStruct);

    TIM_ICInitStruct.TIM_ICFilter = 6;
    TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    TIM_ICInit(TIM2, &TIM_ICInitStruct);

    TIM_SetCounter(TIM2, 0);
    TIM_Cmd(TIM2, ENABLE);
}

// TIM3 左后编码器 PA6 PA7
void TIM3_Encoder_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
    TIM_ICInitTypeDef TIM_ICInitStruct;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    TIM_TimeBaseStruct.TIM_Prescaler = 0;
    TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStruct.TIM_Period = 65535;
    TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStruct);

    TIM_ICInitStruct.TIM_ICFilter = 6;
    TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    TIM_ICInit(TIM3, &TIM_ICInitStruct);

    TIM_SetCounter(TIM3, 0);
    TIM_Cmd(TIM3, ENABLE);
}

// TIM4 右前编码器 PB6 PB7
void TIM4_Encoder_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
    TIM_ICInitTypeDef TIM_ICInitStruct;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    TIM_TimeBaseStruct.TIM_Prescaler = 0;
    TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStruct.TIM_Period = 65535;
    TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStruct);

    TIM_ICInitStruct.TIM_ICFilter = 6;
    TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    TIM_ICInit(TIM4, &TIM_ICInitStruct);

    TIM_SetCounter(TIM4, 0);
    TIM_Cmd(TIM4, ENABLE);
}

// 右后 PB12/PB13 EXTI软件编码器初始化
void EXTI_RR_Encoder_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    EXTI_InitTypeDef EXTI_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource12);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource13);

    EXTI_InitStruct.EXTI_Line = EXTI_Line12 | EXTI_Line13;
    EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTI_InitStruct.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStruct);

    NVIC_InitStruct.NVIC_IRQChannel = EXTI15_10_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
}

// 20ms测速计算函数，主循环调用
void Speed_Calc_20ms(void)
{
    int16_t now_LF = (int16_t)TIM_GetCounter(TIM2);
    int16_t delta_LF = now_LF - last_LF;
    last_LF = now_LF;
    v_LF = delta_LF * g_speed_coeff;

    int16_t now_LB = (int16_t)TIM_GetCounter(TIM3);
    int16_t delta_LB = now_LB - last_LB;
    last_LB = now_LB;
    v_LB = delta_LB * g_speed_coeff;

    int16_t now_RF = (int16_t)TIM_GetCounter(TIM4);
    int16_t delta_RF = now_RF - last_RF;
    last_RF = now_RF;
    v_RF = delta_RF * g_speed_coeff;

    int32_t now_RR = g_pulse_RR;
    int32_t delta_RR = now_RR - last_RR;
    last_RR = now_RR;
    v_RR = delta_RR * g_speed_coeff;
}


// 右后软件编码器中断
void EXTI15_10_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line12) || EXTI_GetITStatus(EXTI_Line13))
    {
        uint8_t A = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12);
        uint8_t B = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_13);
        uint8_t new_state = (A << 1) | B;

        switch((old_state << 4) | new_state)
        {
            case 0x01: case 0x13: case 0x32: case 0x20:
                g_pulse_RR ++; break;
            case 0x10: case 0x31: case 0x23: case 0x02:
                g_pulse_RR --; break;
            default: break;
        }
        old_state = new_state;

        EXTI_ClearITPendingBit(EXTI_Line12);
        EXTI_ClearITPendingBit(EXTI_Line13);
    }
}
