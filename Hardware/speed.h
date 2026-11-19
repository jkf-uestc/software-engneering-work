#ifndef __SPEED_H
#define __SPEED_H

#include "stm32f10x.h"

/* ==================================================================
 * 速度闭环（FN-05 Speed_Control）
 *
 * 职责：目标速度 + 实测速度 -> 四路带符号占空比
 * 边界：只做控制运算，不读编码器、不写 PWM 寄存器
 *
 * ---------------- 在系统中的位置 ----------------
 *
 *   按键(FN-07) ─┐
 *   蓝牙(FN-14) ─┼→ 状态机(FN-09) → 本模块 → 电机驱动(FN-02)
 *   避障(FN-11) ─┘
 *
 * 上游只调 Speed_SetTarget / Speed_SetTurn / Speed_SetWheelTarget，
 * 不直接操作 PWM。这样换控制策略不影响上层，换指令源也不影响本模块。
 *
 * ---------------- 集成步骤 ----------------
 *
 * 1) 初始化，Motor_Init 之后：
 *        Speed_Init();
 *        Speed_SetVbat(12000);            // 接入 ADC 后改为每 200 ms 喂实测值
 *
 * 2) 20 ms 任务，读编码器并换算速度：
 *        if (Tick_Every(&t_spd, 20)) Speed_Calc_20ms();
 *
 * 3) 10 ms 任务，闭环运算：
 *        if (Tick_Every(&t_ctl, 10)) Speed_Tick();
 *
 * 4) 接收上层指令（状态机或主循环）：
 *        int16_t v = Key_GetSpeed();      // 按键档位，0 表示停止
 *        if (v == 0) Speed_Stop();
 *        else        { Speed_SetTarget(v); Speed_Enable(1); }
 *
 *    蓝牙通道同理：指令解析出 mm/s 后调 Speed_SetTarget。
 *
 * 5) 停车与故障：
 *        Speed_Stop();                    // 目标清零 + 清 PID
 *        Speed_GetFault();                // 0 正常，2 堵转
 *        Speed_ClearFault();              // 复位时调用
 * ================================================================== */

/* ---------------- 标定参数 ----------------
 * 数值为 12 V 电池、贴地直行、空载工况下的实车整定结果。
 * 更换地面材质、增加载重或长期偏离 12 V 时，死区与前馈增益需重标；
 * Kp / Ki 对负载变化相对不敏感，一般可沿用。 */

/* 起转死区占空比：电机开始连续转动的临界值，实测区间 800 ~ 1000 */
#define SPEED_DEAD_ZONE         900

/* 前馈比例增益，Q10 定点（真实值 = 宏值 / 1024）
 * 含义：目标速度每 1 mm/s 需要额外给出的占空比。
 * 300 mm/s 下前馈输出 900 + 3.5 x 300 = 1950，高于稳态工作点 1800，
 * 多出的 8% 用于抵消起步静摩擦，残差由 PID 修正。 */
#define SPEED_FF_GAIN_Q10       3584

/* 电压归一化基准，单位 mV */
#define SPEED_VOLT_REF_MV       12000

/* PID 系数，Q16 定点（真实值 = 宏值 / 65536）
 * Kp = 1.5：增量式中 Kp 作用于误差变化量，取值过大会在速度上升段
 *           产生反向修正，反而拖慢收敛。
 * Ki = 0.1：消除稳态残差，再大会引起低频振荡。
 * Kd = 0  ：速度由脉冲计数得到，微分项会放大量化噪声。 */
#define SPEED_KP_Q16            98304
#define SPEED_KI_Q16            6554
#define SPEED_KD_Q16            0

/* ---------------- 运行期参数 ----------------
 * 上面的宏是上电默认值，实际参与运算的是下面这组变量。
 * Speed_SetParam() 修改后下一个 10 ms 周期即生效，无需重新烧录。
 * 参数掉电不保存，需要固化时把标定值写回上面的宏。 */
extern int32_t g_speed_dead_zone;
extern int32_t g_speed_ff_gain_q10;
extern int32_t g_speed_kp_q16;
extern int32_t g_speed_ki_q16;
extern int32_t g_speed_kd_q16;

/* ---------------- 直行航向修正 ----------------
 * 四路独立闭环只能保证各轮转速一致，保证不了车体走直线：
 * 轮径差异、安装角度、打滑都会让轨迹偏移，而速度环测不到航向。
 * 以下两级补偿仅在四轮目标完全一致（直行）时生效，转向不介入。
 */

/* 一级：动态同步。按左右平均速度差反向分配目标，
 * 总速度不变，只压制动态不一致。 */
#define SPEED_SYNC_MAX_ADJ      30          /* 单侧补偿上限 mm/s，防发散 */
#define SPEED_SYNC_GAIN_Q8      256         /* 256 = 1.0 全补偿；128 = 半补偿 */

extern int32_t g_speed_sync_on;             /* 1 = 开启 */

/* 二级：固定偏置。跑偏方向恒定时给左右一个固定差值，
 * 正值 = 左加右减（车往右修），负值相反，单位 mm/s。 */
extern int32_t g_speed_trim;

/* 参数编号，供 Speed_SetParam / Speed_GetParam 使用 */
#define SPEED_PARAM_DEAD_ZONE   0
#define SPEED_PARAM_FF_GAIN     1
#define SPEED_PARAM_KP          2
#define SPEED_PARAM_KI          3
#define SPEED_PARAM_KD          4
#define SPEED_PARAM_SYNC        5           /* 0 = 关，1 = 开 */
#define SPEED_PARAM_TRIM        6           /* 直行偏置 mm/s */

/* 占空比上限：TIM1 的 ARR = 3599 */
#define SPEED_DUTY_MAX          3599

/* 堵转判定（E-08）：单路占空比足够大而该路长时间无转速 */
#define SPEED_STALL_DUTY        1080        /* 30% x 3600 */
#define SPEED_STALL_MS          500
#define SPEED_STALL_SPEED       30          /* mm/s，低于此值视为静止 */

/* 目标速度范围，mm/s（FR-02） */
#define SPEED_TARGET_MIN        100
#define SPEED_TARGET_MAX        800

/* 闭环调用间隔，ms（FR-12） */
#define SPEED_TICK_MS           10

/* 闭环使能掩码：bit0=左前, bit1=左后, bit2=右前, bit3=右后
 * 0x0F = 四轮全闭环
 * 0x0C = 只闭环右侧两轮，用于左侧编码器未接入时单独验证 */
#define SPEED_CLOSED_MASK       0x0F

/* 未闭环轮子的固定开环占空比，0 表示不转 */
#define SPEED_OPEN_DUTY         0

/* 测速输入的单位换算系数
 *
 * encoder.c 的换算系数在 main 中按下式给出：
 *     g_speed_coeff = (pi x WHEEL_D_CM) / (PULSE_PER_REV x SAMPLE_T)
 *                   = (pi x 6.5) / (1320 x 0.02) = 0.7735
 * 周长以 cm 计，故系数单位为 cm/s per count，
 * v_LF / v_LB / v_RF / v_RR 的物理单位是 cm/s，乘 10 得 mm/s。 */
#define SPEED_SRC_SCALE         10.0f

/* ---------------- 编码器符号归一化 ----------------
 *
 * 左右电机镜像安装，编码器 A/B 相序相反：轮子物理前进时，
 * 左侧两路与右后轮的读数为负。本模块统一按「物理前进为正」处理。
 *
 * 标定依据：四轮同时施加正转占空比，四路读数为
 *     LF = -154   LB = -154   RF = +154   RR = -146
 *
 * 若 encoder.c 后续自行修正了符号，须把对应宏改回 1，
 * 否则二次取反会使闭环变成正反馈。 */
#define SPEED_SIGN_LF           (-1)
#define SPEED_SIGN_LB           (-1)
#define SPEED_SIGN_RF           ( 1)
#define SPEED_SIGN_RR           (-1)

/* 速度滑动平均点数，每点对应一次测速更新（20 ms）
 * 1 = 不平滑
 * 3 = 60 ms 窗，可把 0.10 m/s 处量化误差从 7.7% 压到 2.6%
 * 注意：本模块按 10 ms 调用，内部按 2 分频对齐到 20 ms 采样节奏 */
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

/* 任意四轮目标：w[i] 为各轮线速度 mm/s，正 = 该轮物理前进，负 = 后退 */
void    Speed_SetWheelTarget(const int16_t w[4]);

void    Speed_Enable(uint8_t on);               /* 1 使能闭环，0 断开输出 */
void    Speed_Stop(void);                       /* 目标清零 + 清 PID 状态 */
void    Speed_ClearFault(void);                 /* 清故障标志，供复位调用 */
void    Speed_SetVbat(uint16_t mv);             /* 喂电池电压，用于前馈归一化 */

/* 在线改参：id 取 SPEED_PARAM_xxx
 * 死区为原始占空比；前馈增益为 Q10；Kp/Ki/Kd 为 Q16；trim 为 mm/s */
void    Speed_SetParam(uint8_t id, int32_t value);
int32_t Speed_GetParam(uint8_t id);

/* 周期任务：每 10 ms 调用一次（FR-12） */
void    Speed_Tick(void);

/* ---------------- 查询接口 ---------------- */

int16_t Speed_GetTarget(void);                  /* 当前目标车速 mm/s */
int32_t Speed_GetWheelSpeed(uint8_t i);         /* 第 i 轮实测 mm/s */
int16_t Speed_GetDuty(uint8_t i);               /* 第 i 轮当前占空比，带符号 */
uint8_t Speed_GetFault(void);                   /* 0 正常，2 堵转 */
uint8_t Speed_IsEnabled(void);

#endif
