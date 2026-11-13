#include "speed.h"
#include "motor.h"
#include "encoder.h"

/* ==================================================================
 * 速度闭环实现
 *
 * 结构：
 *   目标 -> [前馈 u_ff] -> [增量式 PI] -> [限幅/抗饱和] -> [符号映射] -> 电机
 *
 * 为什么用增量式：算的是"这次比上次多给多少"，输出顶到限幅时直接丢弃
 * 这次增量即可，不必回退已经累积的积分项，天然抗饱和（T-01）。
 * ================================================================== */

/* 单轮 PID 运行状态 */
typedef struct
{
    int32_t err_prev;       /* e(k-1) */
    int32_t err_prev2;      /* e(k-2)，当前 Kd=0 用不到，留着便于以后开微分 */
    int32_t duty;           /* 当前输出，带符号，范围 -SPEED_DUTY_MAX ~ +SPEED_DUTY_MAX */
    int32_t speed;          /* 最近一次实测速度 mm/s */
    uint16_t stall_ms;      /* 堵转累计时间 */
    uint8_t started;        /* 1 = 已用前馈起头，避免输出过零时反复重置 */
} PidState_t;

static PidState_t s_pid[4];

static int16_t  s_target;               /* 直行目标车速 mm/s */
static int16_t  s_wheel_target[4];      /* 四轮目标，带符号 */
static uint8_t  s_enable;               /* 1 闭环工作，0 输出断开 */
static uint8_t  s_fault;                /* 故障码 */
static uint16_t s_vbat_mv;              /* 电池电压 mV */

/* 速度滑动平均缓冲（SPEED_AVG_N = 1 时不会被用到） */
#if (SPEED_AVG_N > 1)
static int32_t s_avg_buf[4][SPEED_AVG_N];
static uint8_t s_avg_idx;
static uint8_t s_avg_cnt;
static uint8_t s_avg_div;       /* 10 ms -> 20 ms 分频计数 */
#endif

/* ---------------- 私有函数 ---------------- */

/* 把 encoder.c 的 float 速度统一换算成 mm/s 整数，并按 sign 归一化方向
 * 对方输出的是 cm/s（见 speed.h 里 SPEED_SRC_SCALE 的说明）
 * sign 的取值见 speed.h 的 SPEED_SIGN_*：镜像安装的轮子传 -1 */
static int32_t Speed_ToMmps(float v, int8_t sign)
{
    return (int32_t)(v * SPEED_SRC_SCALE) * (int32_t)sign;
}

/* 前馈：死区补偿 + 与目标速度成正比的基础占空比 + 电压归一化
 * 只给一个"大概对"的起点，剩下的残差由 PID 修正。
 * 比例项的作用是让起步直接落在工作点附近，否则积分要从死区爬到
 * 工作点，收敛时间会被拖到 2 秒以上（FR-13 要求 1.5 秒）。 */
static int32_t Speed_FeedForward(int16_t target)
{
    int32_t base;
    int32_t u;

    if (target < 0) target = -target;

    /* 死区 + 比例项 */
    base = (int32_t)SPEED_DEAD_ZONE
         + (((int32_t)SPEED_FF_GAIN_Q10 * (int32_t)target) >> 10);

    /* 电压归一化：电池电压低于基准时按比例放大 */
    if (s_vbat_mv < 6000)
    {
        u = base;
    }
    else
    {
        u = base * SPEED_VOLT_REF_MV / (int32_t)s_vbat_mv;
    }

    if (u > SPEED_DUTY_MAX) u = SPEED_DUTY_MAX;
    if (u < 0)              u = 0;

    return u;
}

/* 输出到电机：这里负责"物理前进为正"到"左右轮镜像"的翻译
 * motor.c 的约定：左轮 FORWARD 是前进，右轮 BACKWARD 才是前进 */
static void Speed_Output(uint8_t i, int32_t duty)
{
    uint16_t pwm;
    MotorDir_t dir_fwd;
    MotorDir_t dir_bwd;

    if (duty > SPEED_DUTY_MAX)  duty = SPEED_DUTY_MAX;
    if (duty < -SPEED_DUTY_MAX) duty = -SPEED_DUTY_MAX;

    if (duty >= 0) pwm = (uint16_t)duty;
    else           pwm = (uint16_t)(-duty);

    /* 右前、右后是镜像安装，前进要用 BACKWARD */
    if (i == WHEEL_RF || i == WHEEL_RR)
    {
        dir_fwd = MOTOR_BACKWARD;
        dir_bwd = MOTOR_FORWARD;
    }
    else
    {
        dir_fwd = MOTOR_FORWARD;
        dir_bwd = MOTOR_BACKWARD;
    }

    switch (i)
    {
    case WHEEL_LF:
        M1_SetDir(duty >= 0 ? dir_fwd : dir_bwd);
        M1_SetPWM(pwm);
        break;
    case WHEEL_LB:
        M2_SetDir(duty >= 0 ? dir_fwd : dir_bwd);
        M2_SetPWM(pwm);
        break;
    case WHEEL_RF:
        M3_SetDir(duty >= 0 ? dir_fwd : dir_bwd);
        M3_SetPWM(pwm);
        break;
    case WHEEL_RR:
        M4_SetDir(duty >= 0 ? dir_fwd : dir_bwd);
        M4_SetPWM(pwm);
        break;
    default:
        break;
    }
}

/* 堵转判定 E-08：占空比够大但轮子不转，累计到 500 ms 就报故障
 * 按"单轮"判定，不按整车车速，这样原地转向时四轮都在转，不会误报 */
static void Speed_StallCheck(uint8_t i)
{
    int32_t duty = s_pid[i].duty;
    int32_t spd  = s_pid[i].speed;

    if (duty < 0) duty = -duty;
    if (spd  < 0) spd  = -spd;

    if (duty >= SPEED_STALL_DUTY && spd < SPEED_STALL_SPEED)
    {
        s_pid[i].stall_ms += SPEED_TICK_MS;
        if (s_pid[i].stall_ms >= SPEED_STALL_MS)
        {
            s_fault = SPEED_FAULT_STALL;
        }
    }
    else
    {
        s_pid[i].stall_ms = 0;
    }
}

/* 清空一路 PID 状态（换向、停止、故障时用） */
static void Speed_ResetPid(uint8_t i)
{
    s_pid[i].err_prev  = 0;
    s_pid[i].err_prev2 = 0;
    s_pid[i].duty      = 0;
    s_pid[i].stall_ms  = 0;
    s_pid[i].started   = 0;
}

/* ---------------- 对外函数 ---------------- */

void Speed_Init(void)
{
    uint8_t i;

    for (i = 0; i < 4; i++)
    {
        Speed_ResetPid(i);
        s_pid[i].speed     = 0;
        s_wheel_target[i]  = 0;
    }

    s_target   = 0;
    s_enable   = 0;
    s_fault    = SPEED_FAULT_NONE;
    s_vbat_mv  = SPEED_VOLT_REF_MV;

#if (SPEED_AVG_N > 1)
    s_avg_idx = 0;
    s_avg_cnt = 0;
    s_avg_div = 0;
#endif
}

void Speed_SetTarget(int16_t mmps)
{
    uint8_t i;

    if (mmps < 0) mmps = -mmps;
    if (mmps > SPEED_TARGET_MAX) mmps = SPEED_TARGET_MAX;
    if (mmps != 0 && mmps < SPEED_TARGET_MIN) mmps = SPEED_TARGET_MIN;

    s_target = mmps;

    for (i = 0; i < 4; i++)
    {
        s_wheel_target[i] = mmps;
    }
}

void Speed_SetTurn(int8_t dir, int16_t mmps)
{
    if (mmps < 0) mmps = -mmps;
    if (mmps > SPEED_TARGET_MAX) mmps = SPEED_TARGET_MAX;
    if (mmps != 0 && mmps < SPEED_TARGET_MIN) mmps = SPEED_TARGET_MIN;

    if (dir > 0)                    /* 原地左转：左侧后退，右侧前进 */
    {
        s_wheel_target[WHEEL_LF] = (int16_t)(-mmps);
        s_wheel_target[WHEEL_LB] = (int16_t)(-mmps);
        s_wheel_target[WHEEL_RF] = mmps;
        s_wheel_target[WHEEL_RR] = mmps;
        s_target = mmps;
    }
    else if (dir < 0)               /* 原地右转：左侧前进，右侧后退 */
    {
        s_wheel_target[WHEEL_LF] = mmps;
        s_wheel_target[WHEEL_LB] = mmps;
        s_wheel_target[WHEEL_RF] = (int16_t)(-mmps);
        s_wheel_target[WHEEL_RR] = (int16_t)(-mmps);
        s_target = mmps;
    }
    else
    {
        Speed_SetTarget(mmps);
    }
}

void Speed_SetWheelTarget(const int16_t w[4])
{
    uint8_t i;

    for (i = 0; i < 4; i++)
    {
        s_wheel_target[i] = w[i];
    }

    /* 直行时把标量目标同步过去，供调试输出与状态查询使用 */
    if (w[0] == w[1] && w[1] == w[2] && w[2] == w[3] && w[0] >= 0)
    {
        s_target = w[0];
    }
}

void Speed_Enable(uint8_t on)
{
    uint8_t i;

    if (on)
    {
        s_enable = 1;
        return;
    }

    s_enable = 0;

    for (i = 0; i < 4; i++)
    {
        Speed_ResetPid(i);
        Speed_Output(i, 0);
    }
}

void Speed_Stop(void)
{
    uint8_t i;

    s_target = 0;

    for (i = 0; i < 4; i++)
    {
        s_wheel_target[i] = 0;
        Speed_ResetPid(i);
        Speed_Output(i, 0);
    }
}

void Speed_ClearFault(void)
{
    uint8_t i;

    s_fault = SPEED_FAULT_NONE;

    for (i = 0; i < 4; i++)
    {
        s_pid[i].stall_ms = 0;
    }
}

void Speed_SetVbat(uint16_t mv)
{
    if (mv != 0)
    {
        s_vbat_mv = mv;
    }
}

void Speed_Tick(void)
{
    uint8_t i;
    int16_t tgt;
    int32_t spd;
    int32_t e;
    int32_t du;
    int32_t u;
    int32_t limit;
    int32_t u_ff;

    /* ---------- 1. 取四路实测速度，换算成 mm/s 并归一化方向 ---------- */
    s_pid[WHEEL_LF].speed = Speed_ToMmps(v_LF, SPEED_SIGN_LF);
    s_pid[WHEEL_LB].speed = Speed_ToMmps(v_LB, SPEED_SIGN_LB);
    s_pid[WHEEL_RF].speed = Speed_ToMmps(v_RF, SPEED_SIGN_RF);
    s_pid[WHEEL_RR].speed = Speed_ToMmps(v_RR, SPEED_SIGN_RR);

#if (SPEED_AVG_N > 1)
    /* 滑动平均：对速度值做窗口累加，等效于拉长测速闸门，
     * 可以压低低速段的量化误差，代价是额外滞后。
     *
     * 测速是 20 ms 更新一次，而本函数 10 ms 调一次，
     * 所以按 2 分频入缓冲，避免同一个速度值被记两次导致窗口失真 */
    s_avg_div++;
    if (s_avg_div >= 2)
    {
        s_avg_div = 0;

        s_avg_buf[0][s_avg_idx] = s_pid[0].speed;
        s_avg_buf[1][s_avg_idx] = s_pid[1].speed;
        s_avg_buf[2][s_avg_idx] = s_pid[2].speed;
        s_avg_buf[3][s_avg_idx] = s_pid[3].speed;

        if (s_avg_cnt < SPEED_AVG_N) s_avg_cnt++;

        s_avg_idx++;
        if (s_avg_idx >= SPEED_AVG_N) s_avg_idx = 0;
    }

    for (i = 0; i < 4; i++)
    {
        int32_t sum = 0;
        uint8_t k;
        for (k = 0; k < s_avg_cnt; k++)
        {
            sum += s_avg_buf[i][k];
        }
        s_pid[i].speed = sum / (int32_t)s_avg_cnt;
    }
#endif

    /* ---------- 2. 故障 / 未使能：全部断开输出 ---------- */
    if (!s_enable || s_fault != SPEED_FAULT_NONE)
    {
        for (i = 0; i < 4; i++)
        {
            Speed_ResetPid(i);
            Speed_Output(i, 0);
        }
        return;
    }

    /* ---------- 3. 逐轮增量式 PI ---------- */
    for (i = 0; i < 4; i++)
    {
        tgt = s_wheel_target[i];
        spd = s_pid[i].speed;

        /* 目标为零：滑行，同时清状态，下次启动重新用前馈起头 */
        if (tgt == 0)
        {
            Speed_ResetPid(i);
            Speed_Output(i, 0);
            continue;
        }

        /* 该轮未接入编码器（SPEED_CLOSED_MASK 未置位）：不做闭环，
         * 给固定开环输出。否则会拿浮空引脚上的噪声当反馈，
         * 表现为占空比乱跳、或者一路顶到限幅 */
        if (!(SPEED_CLOSED_MASK & (1 << i)))
        {
            Speed_Output(i, (tgt > 0) ? SPEED_OPEN_DUTY : -SPEED_OPEN_DUTY);
            continue;
        }

        /* 换向保护：目标符号与当前输出符号相反时，先归零停一拍，
         * 避免增量式把"从 +2000 到 -2000"的差值当成需要补偿的量 */
        if (s_pid[i].duty != 0 && ((tgt > 0) != (s_pid[i].duty > 0)))
        {
            Speed_ResetPid(i);
            Speed_Output(i, 0);
            continue;
        }

        /* 刚启动或刚从零恢复：用前馈值作为输出初值，PID 只修残差
         * 用 started 而不是 duty == 0 判断，否则小目标时输出过零会反复重置 */
        if (!s_pid[i].started)
        {
            u_ff = Speed_FeedForward(tgt);
            s_pid[i].duty      = (tgt > 0) ? u_ff : -u_ff;
            s_pid[i].err_prev  = (int32_t)tgt - spd;
            s_pid[i].err_prev2 = s_pid[i].err_prev;
            s_pid[i].started   = 1;
        }

        e = (int32_t)tgt - spd;

        /* 抗饱和：已经顶到限幅且误差还要求继续加力，就丢弃本次增量 */
        if ((s_pid[i].duty >=  SPEED_DUTY_MAX && e > 0) ||
            (s_pid[i].duty <= -SPEED_DUTY_MAX && e < 0))
        {
            du = 0;
        }
        else
        {
            du = ((int32_t)SPEED_KP_Q16 * (e - s_pid[i].err_prev)) >> 16;
            du += ((int32_t)SPEED_KI_Q16 * e) >> 16;

#if (SPEED_KD_Q16 != 0)
            du += ((int32_t)SPEED_KD_Q16 *
                   (e - 2 * s_pid[i].err_prev + s_pid[i].err_prev2)) >> 16;
#endif
        }

        u = s_pid[i].duty + du;

        /* 输出限幅 */
        if (tgt > 0)
        {
            if (u < 0) u = 0;
        }
        else
        {
            if (u > 0) u = 0;
        }

        limit = SPEED_DUTY_MAX;
        if (u >  limit) u =  limit;
        if (u < -limit) u = -limit;

        s_pid[i].duty      = u;
        s_pid[i].err_prev2 = s_pid[i].err_prev;
        s_pid[i].err_prev  = e;

        Speed_Output(i, u);
        Speed_StallCheck(i);
    }
}

/* ---------------- 查询 ---------------- */

int16_t Speed_GetTarget(void)
{
    return s_target;
}

int32_t Speed_GetWheelSpeed(uint8_t i)
{
    if (i > 3) return 0;
    return s_pid[i].speed;
}

int16_t Speed_GetDuty(uint8_t i)
{
    if (i > 3) return 0;
    return (int16_t)s_pid[i].duty;
}

uint8_t Speed_GetFault(void)
{
    return s_fault;
}

uint8_t Speed_IsEnabled(void)
{
    return s_enable;
}
