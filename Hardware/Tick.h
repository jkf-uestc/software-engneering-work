#ifndef __TICK_H
#define __TICK_H

#include "stm32f10x.h"

/* ===== 新增文件（2026-10-27）=====================================
 * 作用：提供 1ms 系统节拍。
 *
 * 为什么不用 SysTick 中断？
 *   1) 教程自带的 System\Delay.c 里 Delay_us() 是"轮询 SysTick"实现的，
 *      它会在运行时改写 SysTick->LOAD / VAL / CTRL。如果再开 SysTick 中断
 *      做节拍，两边会互相破坏（延时不准、节拍丢拍）。
 *      为了"不动已有实现"，这里改用 Cortex-M3 内核自带的 DWT 周期计数器。
 *   2) 引脚表 V2.0 里 TIM1~TIM4 已被 4 路电机 PWM + 4 路编码器占满，
 *      没有任何空闲定时器可以做节拍，DWT 正好绕开这个死结。
 *
 * 为什么下面用地址宏而不是 DWT 结构体？
 *   本工程的 Start\core_cm3.h 是旧版 CMSIS，只定义了 CoreDebug，
 *   没有 DWT_Type / DWT 宏，直接写 DWT->CYCCNT 会编译不过。
 *   为了不动库文件，这里按《Cortex-M3 内核手册》的固定地址直接访问：
 *       DEMCR   0xE000EDFC   bit24 TRCENA = 调试外设总开关
 *       DWT_CTRL 0xE0001000  bit0  CYCCNTENA = 周期计数器使能
 *       DWT_CYCCNT 0xE0001004 32 位周期计数，72MHz 下 72 个 = 1us
 * 精度：1 个计数 = 13.9ns，1ms = 72000 计数。
 * 注意：DWT 是 32 位，59.6 秒回绕一次；Tick_GetMs() 内部用差值累加，
 *       只要主循环每 59 秒内调用一次，对外就是连续递增的毫秒数。
 * ================================================================ */

#define TICK_DEMCR              (*((volatile uint32_t *)0xE000EDFCUL))
#define TICK_DWT_CTRL           (*((volatile uint32_t *)0xE0001000UL))
#define TICK_DWT_CYCCNT         (*((volatile uint32_t *)0xE0001004UL))

#define TICK_DEMCR_TRCENA       (1UL << 24)
#define TICK_DWT_CTRL_CYCCNTENA (1UL << 0)

void     Tick_Init(void);          /* 使能 DWT 并清零基准，放在 main 最前面 */
uint32_t Tick_GetMs(void);         /* 上电以来的毫秒数，主循环里反复调用 */
uint32_t Tick_GetCycles(void);     /* 原始周期计数，72 个 = 1us（中断里也能调） */
void     Tick_DelayUs(uint32_t us);/* 不碰 SysTick 的微秒忙等 */

/* 到点了没有？到点返回 1 并把 *last_ms 更新成现在，没到返回 0。
 * 这是"每个模块自己管自己的节奏"用的，用法：
 *
 *     static uint32_t s_last = 0;
 *     void Xxx_Task(void)
 *     {
 *         if (!Tick_Every(&s_last, 10)) { return; }   // 10ms 才往下走
 *         ...真正要干的事...
 *     }
 *
 * 这样主循环里可以无脑每轮都调 Xxx_Task()，不用在 main 里写 if (now % 10 == 0)。
 * 注意：第一次调用会立刻返回 1（因为 *last_ms 初值 0，差值早超了）。 */
uint8_t  Tick_Every(uint32_t *last_ms, uint32_t period_ms);

#endif
