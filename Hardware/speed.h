#ifndef __SPEED_H
#define __SPEED_H

#include "stm32f10x.h"

/* ==================================================================
 * 速度闭环（FN-05 Speed_Control）
 * 职责：目标速度 + 实测速度 -> 四路带符号占空比
 * 不负责：编码器读取、脉冲换算（那是 encoder.c 的事）
 *
 * ---------------- 集成方式 ----------------
 * 1) main 初始化段，Motor_Init 之后：
 *        Speed_Init();
 *        Speed_SetVbat(12000);     // 没接 ADC 前写死；接了再每 200ms 喂真实值
 *        Speed_Enable(1);
 *
 * 2) 10 ms 周期任务里调（建议用 Tick_Every 驱动）：
 *        if (Tick_Every(&t, 10)) Speed_Tick();
 *
 * 3) 20 ms 周期任务里调（encoder.c 提供）：
 *        if (Tick_Every(&t2, 20)) Speed_Calc_20ms();
 *
 * 4) 设置目标：
 *        Speed_SetTarget(500);     // 直行 500 mm/s（单位 mm/s，范围 100~800）
 *        Speed_SetTurn(1, 300);    // 原地左转
 *        Speed_SetWheelTarget(w);  // 任意四轮，带符号
 *
 * 5) 停车 / 故障：
 *        Speed_Stop();             // 目标清零 + 清 PID
 *        Speed_GetFault();         // 0 正常，2 堵转
 *        Speed_ClearFault();       // 长按复位时调用
 * ================================================================== */

/* ---------------- 可调参数：以下全部为初始值，待实车标定 ---------------- */

/* 死区占空比：电机开始转动的临界值。实测区间约 800~1000，先取中值 */
#define SPEED_DEAD_ZONE         900

/* 前馈比例增益，Q10 定点（真实值 = 宏值 / 1024）
 * 含义：目标速度每 1 mm/s，需要额外给多少占空比
 * 由 2026-11-12 实测反推：300 mm/s 稳态需要 1790，死区 900，
 * (1790 - 900) / 300 = 2.97，取 3.0 → 3.0 × 1024 = 3072
 * 换地面、换电池电压时要重标 */
#define SPEED_FF_GAIN_Q10       3072        /* 3.0 —— 2026-11-12 实测值，已验证 */

/* 电压归一化基准，单位 mV。电池电压低于此值时自动放大占空比 */
#define SPEED_VOLT_REF_MV       12000

/* PID 系数，Q16 定点（真实值 = 宏值 / 65536）
 * Kp = 3.0，起点按 1/静态增益估算，实车整定范围约 2~6
 * Ki = 0.1，先给小值，避免一上来就振荡
 * Kd = 0  ，速度反馈含量化噪声，微分只会放大抖动 */
#define SPEED_KP_Q16            131072      /* 2.0 —— 2026-11-12 实测值，已验证 */
#define SPEED_KI_Q16            6554        /* 0.1  */
#define SPEED_KD_Q16            0           /* 0.0  */

/* 占空比上限：TIM1 的 ARR = 3599 */
#define SPEED_DUTY_MAX          3599

/* 堵转判定（E-08）：占空比超过 30% 而轮速长时间为 0 */
#define SPEED_STALL_DUTY        1080        /* 30% × 3600 */
#define SPEED_STALL_MS          500
#define SPEED_STALL_SPEED       30          /* mm/s，低于此值视为静止 */

/* 目标速度范围，mm/s（FR-02） */
#define SPEED_TARGET_MIN        100
#define SPEED_TARGET_MAX        800

/* 单次闭环调用的时间间隔，ms（FR-12） */
#define SPEED_TICK_MS           10

/* 闭环使能掩码：bit0=左前, bit1=左后, bit2=右前, bit3=右后
 * 0x0F = 四轮全闭环（正常状态）
 * 0x0C = 只闭环右前 + 右后（左侧编码器未接入时的临时验证） */
#define SPEED_CLOSED_MASK       0x0F

/* 未闭环轮子的固定开环占空比。0 = 不转（架起来单测时用） */
#define SPEED_OPEN_DUTY         0

/* 测速输入的单位换算系数
 *
 * encoder.c 的 g_speed_coeff 在 main 里这样算：
 *     g_speed_coeff = (3.1415926f * WHEEL_D_CM) / (PULSE_PER_REV * SAMPLE_T);
 *                   = (π × 6.5) / (1320 × 0.02) = 0.7735
 * 因为周长用的是 cm，所以系数单位是 "cm/s per count"，
 * v_LF / v_LB / v_RF / v_RR 的物理单位是 cm/s。
 * 乘 10 换算成 mm/s。若以后改成 m/s，这里改成 1000.0f。 */
#define SPEED_SRC_SCALE         10.0f

/* ---------------- 编码器符号归一化 ----------------
 *
 * 左右两侧电机是镜像安装的，编码器 A/B 相的相序相反，
 * 于是"轮子物理前进"时，左侧两路和右后那一路的读数会是负数。
 * 本模块内部统一约定"物理前进为正"，所以在入口把符号翻过来。
 *
 * 实测依据（2026-11-12，四轮同时给正转占空比 1000）：
 *     LF = -154   LB = -154   RF = +154   RR = -146
 * 即 LF / LB / RR 需要取反，RF 不需要。
 *
 * ！！注意！！
 * 这个修正是记在 speed.c 里的，没有改动 encoder.c（那是别人的模块）。
 * 如果以后 encoder.c 自己把符号改对了（比如 v_LF = -delta_LF * coeff），
 * 必须把对应的宏改回 1，否则会二次取反，PID 立刻变正反馈失控。
 * 判断方法：四轮同时正转，看串口打印的 LF/LB/RF/RR 是否全为正数。 */
#define SPEED_SIGN_LF           (-1)
#define SPEED_SIGN_LB           (-1)
#define SPEED_SIGN_RF           ( 1)
#define SPEED_SIGN_RR           (-1)

/* 速度滑动平均点数，每点对应一次测速更新（20 ms）
 * 1 = 不平滑（默认，先跑通）
 * 3 = 60 ms 窗，可把 0.10 m/s 处量化误差从 7.7% 压到 2.6%
 * 注意：本模块按 10 ms 调用，内部会自动按 2 分频对齐到 20 ms 采样节奏 */
#define SPEED_AVG_N             1

/* 故障码，与概要设计 E-08 对应 */
#define SPEED_FAULT_NONE        0
#define SPEED_FAULT_STALL       2

/* 轮序索引，与 motor.c 的 M1~M4 一一对应 */
#define WHEEL_LF                0           /* M1 左前 */
#define WHEEL_LB                1           /* M2 左后 */
#define WHEEL_RF                2           /* M3 右前 */
#define WHEEL_RR                3           /* M4 右后 */

/* ---------------- 对外接口 ---------------- */

void    Speed_Init(void);                       /* 初始化，Motor_Init 之后调用 */

/* 直行：四轮目标同值，单位 mm/s，自动限幅到 100~800 */
void    Speed_SetTarget(int16_t mmps);

/* 转向：dir = 1 原地左转，-1 原地右转，0 直行；单位 mm/s */
void    Speed_SetTurn(int8_t dir, int16_t mmps);

/* 任意四轮目标：w[i] 为各轮线速度 mm/s，正=该轮物理前进，负=后退 */
void    Speed_SetWheelTarget(const int16_t w[4]);

void    Speed_Enable(uint8_t on);               /* 1 使能闭环，0 断开输出 */
void    Speed_Stop(void);                       /* 目标清零 + 清 PID 状态 */
void    Speed_ClearFault(void);                 /* 清故障标志，供长按复位调用 */
void    Speed_SetVbat(uint16_t mv);             /* 喂电池电压，用于前馈归一化 */

/* 周期任务：每 10 ms 调用一次（FR-12） */
void    Speed_Tick(void);

/* ---------------- 查询接口（给调试输出 / 指示灯用） ---------------- */

int16_t Speed_GetTarget(void);                  /* 当前目标车速 mm/s */
int32_t Speed_GetWheelSpeed(uint8_t i);         /* 第 i 轮实测 mm/s */
int16_t Speed_GetDuty(uint8_t i);               /* 第 i 轮当前输出占空比，带符号 */
uint8_t Speed_GetFault(void);                   /* 0 正常，2 堵转 */
uint8_t Speed_IsEnabled(void);

#endif
