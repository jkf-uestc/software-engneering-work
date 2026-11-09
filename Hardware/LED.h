//封装LED的驱动程序
//存放驱动函数可以对外提供的函数和变量声明

#ifndef __LED_H   //if no define
#define __LED_H

#include "stm32f10x.h"

/*==============================================================================
 *  灯光指示模块（就绪 / 运行 / 故障  三态）
 *  依据《STM32F103C8T6 引脚分配表 V2.0》
 *
 *   网络名     引脚   板卡位置   方向   工作模式        备注
 *   LED_RDY    PC13   右排19     输出   推挽(灌)        低电平点亮，1kΩ，仅 3mA
 *   LED_RUN    PC14   右排18     输出   推挽(灌)        低电平点亮，LSE 必须关闭
 *   LED_ERR    PB3    左排11     输出   GPIO 推挽       低电平点亮，故障 2Hz，需关 JTAG
 *
 *  PC15 / LED_STOP 本模块不使用：需求只保留"就绪/运行/故障"三态，
 *  如以后要加"停止灯"，在 LED_Mode_t 里加一项、在 LED_Update() 里加一个 case 即可。
 *
 *  显示约定（同一时刻只亮一盏，避免误读）：
 *    就绪 READY  →  LED_RDY 常亮
 *    运行 RUN    →  LED_RUN 1Hz 闪烁（亮 500ms / 灭 500ms）
 *    故障 FAULT  →  LED_ERR 2Hz 闪烁（亮 250ms / 灭 250ms，与《任务调度表》的 250ms 翻转一致）
 *    上电默认     →  全灭，直到调用 LED_Ready()
 *
 *  用法（全程非阻塞，主循环里不要用 Delay 等灯）：
 *    LED_Init();
 *    LED_Ready();                    // 或 LED_Run() / LED_Fault()
 *    while (1)
 *    {
 *        LED_Update(Tick_Get());     // 建议每 1ms 调一次（任意 <=5ms 周期也够）
 *        ...
 *    }
 *
 *  注意：LED_Update() 的参数是"毫秒单调节拍"，内部用无符号减法算相位，
 *        计数回绕（49.7 天）不会出错；但不允许传入来回跳变的数。
 *============================================================================*/

/* 1 = 低电平点亮（灌电流，本板即如此）；0 = 高电平点亮（阳极限流后接引脚） */
#define LED_ACTIVE_LOW          1

/* 闪烁半周期(ms)：亮/灭各一半 */
#define LED_BLINK_RUN_MS        500u    /* 运行灯 1Hz */
#define LED_BLINK_FAULT_MS      250u    /* 故障灯 2Hz（表 3：tick % 250 == 9 翻转） */

/* 三种状态 */
typedef enum
{
    LED_MODE_READY = 0,     /* 就绪 */
    LED_MODE_RUN,           /* 运行 */
    LED_MODE_FAULT          /* 故障 */
} LED_Mode_t;

/* 初始化（含关 LSE、关 JTAG，只需调用一次） */
void LED_Init(void);

/* 切换状态：下一次 LED_Update() 生效，新灯从"亮"开始计时 */
void LED_SetMode(LED_Mode_t mode);

/* 周期刷新：now_ms 传系统毫秒节拍（Tick_Get()），每 1ms 调一次 */
void LED_Update(uint32_t now_ms);

/* 读当前状态 */
LED_Mode_t LED_GetMode(void);

/* 全部熄灭（不动状态机，断电/停机时可用） */
void LED_AllOff(void);

/* 三个便捷接口，等价于 LED_SetMode(LED_MODE_xxx) */
void LED_Ready(void);
void LED_Run(void);
void LED_Fault(void);

#endif
