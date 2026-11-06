#ifndef __SONAR_H
#define __SONAR_H

#include "stm32f10x.h"

/* ============================================================================
 * Sonar —— 超声波测距驱动（HC-SR04）
 * ----------------------------------------------------------------------------
 * 这个文件管什么：
 *   把"引脚上的电平变化"变成"一个以毫米为单位的距离"。
 * 它不管：该不该停车（那是 Avoid.c 的事）、距离怎么显示（那是 Bt.c 的事）。
 *
 *main 主循环里每轮调一次 Sonar_Task()（它自己管 20ms 节奏和自己触发）
 *Tick.c（取时间戳）、Delay.c（教程自带的 Delay_us，只在触发时用 10us）

 * 引脚（引脚分配表 V2.0）：
 *   PB5  TRIG  推挽输出，给 >=10us 高电平就开始测
 *   PB4  ECHO  输入，高电平持续多久 = 声波往返时间；走 EXTI4 双边沿
 *
 * 为什么用 EXTI + DWT 而不是"定时器输入捕获"：
 *   TIM1~TIM4 已经被 4 路电机 PWM + 4 路编码器占满，没有空闲捕获通道。
 *   一次测量只产生 2 次中断、20ms 一次，开销可以忽略。
 *   （组长 2026-10-27 定案：以后要加第二路，挂树莓派，不要再加 EXTI。）
 *
 * 距离怎么算：
 *   声速约 340m/s，往返 1cm 需要 58us（去 29 + 回 29）。
 *   所以   距离(mm) = 高电平宽度(us) * 10 / 58
 *   例：测到 5800us -> 1000mm = 1m
 
 * 特殊情况处理：
 *   1) 没有回波时 ECHO 一直是低，永远等不到下降沿 -> 必须有超时（30ms）
 *   2) 上一次还没测完就再触发，会把等待标志冲掉，超时永远不触发 -> Trigger 里加 if(s_wait) return
 *   3) 单次测量会抖，尤其打在地面/斜面上 -> 三次取中值
 * ========================================================================== */

#define SONAR_PERIOD_MS     20      /*20ms测一次*/
#define SONAR_TIMEOUT_MS    30      /*判定无效的阈值30ms*/
#define SONAR_MIN_VALID_US 200      /*短于200us的脉冲判定为噪声（约 3.4cm 以内）*/
#define SONAR_MAX_MM      4000      /* 量程上限4m，超出就截断 */

#define SONAR_ST_WAIT      0        /*已发波，正在等回波 */
#define SONAR_ST_OK        1        /*本次拿到有效距离 */
#define SONAR_ST_TIMEOUT   2        /*30ms 没回波*/
#define SONAR_ST_NOISE     3        /*回波太短，当噪声丢弃*/

void     Sonar_Init(void);          
void     Sonar_Task(void);          /*主循环每轮调。内部自己限速 20ms*/
void     Sonar_Trigger(void);       /*手动发一次 10us 触发脉冲（调试用；平时 Sonar_Task 自己会调，不要重复调）*/

uint16_t Sonar_GetMm(void);         /*最近一次距离，单位毫米（三次取中值）*/
uint8_t  Sonar_IsValid(void);       /*1 = 这个距离可信；0 = 超时/噪声*/
uint8_t  Sonar_GetStatus(void);     /*检查状态SONAR_ST_xxx*/

#endif
