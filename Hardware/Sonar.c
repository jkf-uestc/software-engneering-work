#include "Sonar.h"
#include "Delay.h"
#include "Tick.h"

/* ============================================================================
 * Sonar.c —— 超声波测距实现
 * ----------------------------------------------------------------------------
 * 整个文件只做三件事：
 *   1) Sonar_Trigger()：拉高 TRIG 10us，然后等 ECHO 的两个边沿；
 *   2) EXTI4_IRQHandler()：中断里只做"记时间戳"，不做除法不做浮点；
 *   3) Sonar_Task()：在主循环里把时间差换算成毫米，做超时判断和三取中值。
 * ========================================================================== */

#define SONAR_CYC_PER_US   72u    
#define SONAR_TIMEOUT_CYC  (SONAR_TIMEOUT_MS * 1000u * SONAR_CYC_PER_US) //30*1000*72

//volatile表示该变量的值随时可能被主线程以外的东西（中断）改掉，须每次从内存读，不要用寄存器里缓存的旧值。
static volatile uint32_t s_rise_cy  = 0; /*上升沿时刻（周期数）*/
static volatile uint32_t s_width_cy = 0; /*本次高电平宽度（周期数）*/
static volatile uint8_t  s_wait     = 0; /*1 = 已发波，正在等回波*/
static volatile uint8_t  s_new      = 0; /*1 = 测量结束，主循环还没取走*/

static uint32_t s_trig_cy = 0; /*发波时刻，用于算超时*/
static uint16_t s_ring[3];     /*存最近三次的原始结果，用来取中值*/
static uint8_t  s_n       = 0; /*s_ring 里已经有几个数*/
static uint16_t s_dist_mm = 0;
static uint8_t  s_valid   = 0;
static uint8_t  s_status  = SONAR_ST_WAIT;
static uint32_t s_last_task = 0; /*Sonar_Task自己的20ms节拍*/

/*三个数取中值*/
static uint16_t Sonar_Median3(const uint16_t *v)
{
    uint16_t a = v[0], b = v[1], c = v[2];
    if ((a >= b && a <= c) || (a <= b && a >= c)) { return a; }
    if ((b >= a && b <= c) || (b <= a && b >= c)) { return b; }
    return c;
}

void Sonar_Init(void)
{
    GPIO_InitTypeDef gpio;
    EXTI_InitTypeDef exti;
    NVIC_InitTypeDef nvic;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

    /* PB4 上电默认是 JTAG 的 NJTRST，不关 JTAG 的话它永远读不到 ECHO。
       关掉 JTAG、保留 SWD（SWJ_CFG=010），这样 PB3/PB4/PA15才能正常使用。
       注意：这一句是全局副作用，但它必须在这里做——不然超声波没法工作。
       顺便解放了 PA15（左前轮 IN2）。 */
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

    /*TRIG = PB5，推挽输出，先拉低*/
    GPIO_ResetBits(GPIOB, GPIO_Pin_5);
    gpio.GPIO_Pin   = GPIO_Pin_5;
    gpio.GPIO_Mode  = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpio);

    /*ECHO = PB4，下拉输入（HC-SR04 空闲时是低电平）*/
    gpio.GPIO_Pin  = GPIO_Pin_4;
    gpio.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(GPIOB, &gpio);

    /*把EXTI4的输入源接到GPIOB的Pin4上*/
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource4);

    exti.EXTI_Line    = EXTI_Line4;
    exti.EXTI_Mode    = EXTI_Mode_Interrupt;
    exti.EXTI_Trigger = EXTI_Trigger_Rising_Falling; /*两边沿*/
    exti.EXTI_LineCmd = ENABLE;
    EXTI_Init(&exti);

    /* 抢占优先级 2：低于编码器(1)，高于两个串口(3/4)。见引脚表V2.0中断优先级表 */
    nvic.NVIC_IRQChannel                   = EXTI4_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 2;
    nvic.NVIC_IRQChannelSubPriority        = 0;
    nvic.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&nvic);

    s_wait   = 0;
    s_new    = 0;
    s_n      = 0;
    s_valid  = 0;
    s_status = SONAR_ST_WAIT;
    s_dist_mm = 0;
}

void Sonar_Trigger(void)
{
    if (s_wait)
    {
        return;
    }

    s_new     = 0;
    s_wait    = 1;
    s_trig_cy = Tick_GetCycles();

    GPIO_SetBits(GPIOB, GPIO_Pin_5);
    Delay_us(10);                       
    GPIO_ResetBits(GPIOB, GPIO_Pin_5);
}

void EXTI4_IRQHandler(void)
{
    uint32_t now;

    if (EXTI_GetITStatus(EXTI_Line4) != RESET)
    {
        now = Tick_GetCycles();         /*先抢时间戳，再判断电平*/

        if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_4))
        {
            s_rise_cy = now;            
        }
        else if (s_wait)
        {
            s_width_cy = now - s_rise_cy;
            s_wait     = 0;
            s_new      = 1; //CPU有待处理数据             
        }

        EXTI_ClearITPendingBit(EXTI_Line4); 
    }
}

void Sonar_Task(void)
{
    uint32_t now;
    uint32_t us;
    uint16_t mm;

	/*没到20ms自动返回*/
    if (!Tick_Every(&s_last_task, SONAR_PERIOD_MS))
    {
        return;
    }

    now = Tick_GetCycles();

    if (s_wait&&((now - s_trig_cy) > SONAR_TIMEOUT_CYC))
    {
        /*等了 30ms 还没有下降沿*/
        s_wait   = 0;
        s_valid  = 0;
        s_status = SONAR_ST_TIMEOUT;
    }
    else if (s_new)
    {
        /*有新的高电平宽度可以换算*/
        s_new = 0;
        us = s_width_cy / SONAR_CYC_PER_US;

        if (us < SONAR_MIN_VALID_US)
        {
            /*脉冲太短，当噪声丢掉，保留上一次的距离*/
            s_status = SONAR_ST_NOISE;
        }
        else
        {
            mm = (uint16_t)(us * 10u / 58u);    /*10/58近似0.17*/
            if (mm > SONAR_MAX_MM)
            {
                mm = SONAR_MAX_MM;
            }

            /*三次取终止，没有三次就取最新*/
            if (s_n < 3)
            {
                s_ring[s_n++] = mm;
            }
            else
            {
                s_ring[0] = s_ring[1];
                s_ring[1] = s_ring[2];
                s_ring[2] = mm;
            }

            s_dist_mm = (s_n >= 3) ? Sonar_Median3(s_ring) : mm;
            s_valid   = 1;
            s_status  = SONAR_ST_OK;
        }
    }
    /* 收完结果再发下一发。以防把自己刚设定的等待标志冲掉。*/
    Sonar_Trigger();
}

uint16_t Sonar_GetMm(void)
{
    return s_dist_mm;
}

uint8_t Sonar_IsValid(void)
{
    return s_valid;
}

uint8_t Sonar_GetStatus(void)
{
    return s_status;
}
