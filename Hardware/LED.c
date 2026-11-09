//封装LED的驱动程序
//存放驱动程序的主体代码

#include "stm32f10x.h"                  // Device header

#include "LED.h"

/*---------------------------- 引脚映射（只在本文件出现） ----------------------------*/
/* 业务代码一律不碰 GPIO，改脚只改这里 */
#define LED_RDY_PORT        GPIOC
#define LED_RDY_PIN         GPIO_Pin_13     /* 右排19 */

#define LED_RUN_PORT        GPIOC
#define LED_RUN_PIN         GPIO_Pin_14     /* 右排18 */

#define LED_ERR_PORT        GPIOB
#define LED_ERR_PIN         GPIO_Pin_3      /* 左排11，需先关 JTAG */

/*---------------------------- 内部状态 ----------------------------*/
static LED_Mode_t s_mode         = LED_MODE_READY;
static uint8_t    s_mode_changed = 1u;      /* 1：下次 Update 重置闪烁相位 */
static uint32_t   s_phase_start  = 0u;      /* 进入当前状态的时刻(ms) */

/**
  * @brief  驱动一盏灯（自动处理高/低电平有效）
  * @param  GPIOx: GPIOB / GPIOC
  * @param  pin:   引脚号
  * @param  on:    1 点亮，0 熄灭
  * @retval 无
  */
static void LED_Set(GPIO_TypeDef *GPIOx, uint16_t pin, uint8_t on)
{
#if (LED_ACTIVE_LOW != 0)
    GPIO_WriteBit(GPIOx, pin, (on != 0u) ? Bit_RESET : Bit_SET);
#else
    GPIO_WriteBit(GPIOx, pin, (on != 0u) ? Bit_SET : Bit_RESET);
#endif
}

/**
  * @brief  灯光初始化
  * @param  无
  * @retval 无
  * @note   PC13/PC14 属备份域，PC14 还与 OSC32_IN 复用；PB3 复位后是 JTDO。
  *         这三件事不处理，后面的代码怎么写都不会亮。
  */
void LED_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* 1. 开时钟：GPIOC(PC13/PC14)、GPIOB(PB3)、AFIO(PB3 要从 JTAG 手里抢回来) */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);

    /* 2. 关 LSE，释放 PC14/PC15（引脚表要求"LSE 必须关闭"）
     *    RCC_BDCR 在备份域，必须先开 PWR 时钟并解锁 DBP 位，否则写入会被硬件忽略。
     *    复位后 LSE 本来就是关的，这里重复一次是为了防止 RTC 或别的例程打开过它。 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
    PWR_BackupAccessCmd(ENABLE);
    RCC_LSEConfig(RCC_LSE_OFF);
    {
        uint32_t timeout = 100000u;
        while ((RCC_GetFlagStatus(RCC_FLAG_LSERDY) != RESET) && (timeout != 0u))
        {
            timeout--;                          /* 等 LSE 真正停振，带超时防止死等 */
        }
    }
    PWR_BackupAccessCmd(DISABLE);               /* 写完恢复备份域写保护 */

    /* 3. 关 JTAG（SWJ_CFG=010），释放 PB3/PB4/PA15，同时保留 PA13/PA14 的 SWD。
     *    否则 PB3 永远是 JTDO，写 ODR 也不会有电平变化。
     *    千万别用 GPIO_Remap_SWJ_Disable，那会把 SWD 一起关掉，ST-Link 就再也连不上。 */
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

    /* 4. 先把 ODR 置高（=熄），再切成输出，避免初始化那一瞬间 LED 闪一下 */
    GPIO_SetBits(LED_RDY_PORT, LED_RDY_PIN | LED_RUN_PIN);
    GPIO_SetBits(LED_ERR_PORT, LED_ERR_PIN);

    /* 5. PC13/PC14：推挽输出。
     *    PC13~PC15 由电源开关供电，只有 3mA 驱动能力，输出速度最高也只能 2MHz，
     *    配 50MHz 属于超规格使用（数据手册的 output speed 限制）。 */
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Pin   = LED_RDY_PIN | LED_RUN_PIN;
    GPIO_Init(LED_RDY_PORT, &GPIO_InitStructure);

    /* 6. PB3：普通推挽输出 */
    GPIO_InitStructure.GPIO_Pin   = LED_ERR_PIN;
    GPIO_Init(LED_ERR_PORT, &GPIO_InitStructure);

    s_mode         = LED_MODE_READY;
    s_mode_changed = 1u;

    LED_AllOff();
}

/**
  * @brief  切换灯光状态
  * @param  mode: LED_MODE_READY / LED_MODE_RUN / LED_MODE_FAULT
  * @retval 无
  */
void LED_SetMode(LED_Mode_t mode)
{
    if ((mode != LED_MODE_READY) && (mode != LED_MODE_RUN) && (mode != LED_MODE_FAULT))
    {
        return;                                 /* 非法值直接忽略，状态机不会被带坏 */
    }

    if (mode != s_mode)
    {
        s_mode         = mode;
        s_mode_changed = 1u;
    }
}

/**
  * @brief  灯光刷新（非阻塞，靠 tick 比较推进）
  * @param  now_ms: 系统毫秒节拍
  * @retval 无
  * @note   本函数不等待、不占用 CPU，放在主循环 1ms 任务里调用即可。
  */
void LED_Update(uint32_t now_ms)
{
    uint32_t phase;

    if (s_mode_changed != 0u)
    {
        s_mode_changed = 0u;
        s_phase_start  = now_ms;                /* 换状态瞬间，新灯从"亮"开始 */
    }

    phase = (uint32_t)(now_ms - s_phase_start); /* 无符号减法，毫秒计数回绕也安全 */

    switch (s_mode)
    {
        case LED_MODE_RUN:                      /* 运行：RUN 1Hz，其余灭 */
            LED_Set(LED_RDY_PORT, LED_RDY_PIN, 0u);
            LED_Set(LED_ERR_PORT, LED_ERR_PIN, 0u);
            LED_Set(LED_RUN_PORT, LED_RUN_PIN, (((phase / LED_BLINK_RUN_MS) & 1u) != 0u) ? 0u : 1u);
            break;

        case LED_MODE_FAULT:                    /* 故障：ERR 2Hz，其余灭 */
            LED_Set(LED_RDY_PORT, LED_RDY_PIN, 0u);
            LED_Set(LED_RUN_PORT, LED_RUN_PIN, 0u);
            LED_Set(LED_ERR_PORT, LED_ERR_PIN, (((phase / LED_BLINK_FAULT_MS) & 1u) != 0u) ? 0u : 1u);
            break;

        case LED_MODE_READY:                    /* 就绪：RDY 常亮，其余灭 */
        default:
            LED_Set(LED_RUN_PORT, LED_RUN_PIN, 0u);
            LED_Set(LED_ERR_PORT, LED_ERR_PIN, 0u);
            LED_Set(LED_RDY_PORT, LED_RDY_PIN, 1u);
            break;
    }
}

/**
  * @brief  读当前灯光状态
  * @retval LED_MODE_READY / LED_MODE_RUN / LED_MODE_FAULT
  */
LED_Mode_t LED_GetMode(void)
{
    return s_mode;
}

/**
  * @brief  三盏灯全灭（不改变状态变量）
  * @param  无
  * @retval 无
  */
void LED_AllOff(void)
{
    LED_Set(LED_RDY_PORT, LED_RDY_PIN, 0u);
    LED_Set(LED_RUN_PORT, LED_RUN_PIN, 0u);
    LED_Set(LED_ERR_PORT, LED_ERR_PIN, 0u);
}

/*------------------------- 三个便捷接口 -------------------------*/
void LED_Ready(void)
{
    LED_SetMode(LED_MODE_READY);
}

void LED_Run(void)
{
    LED_SetMode(LED_MODE_RUN);
}

void LED_Fault(void)
{
    LED_SetMode(LED_MODE_FAULT);
}
