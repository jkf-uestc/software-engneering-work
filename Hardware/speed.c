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
 *
 * ---------------- 单位约定 ----------------
 * 速度量：一律 mm/s（带符号，物理前进为正）。
 *         编码器送进来的是 cm/s，在 Speed_ToMmps 里乘 10 换成 mm/s。
 * 占空比：一律是 TIM1 比较值（0 ~ SPEED_DUTY_MAX = 3599），不是百分数。
 * 时间  ：一律 ms。
 * ================================================================== */

/* 单轮 PID 运行状态 */
typedef struct
{
    int32_t err_prev;       /* e(k-1) */
    int32_t err_prev2;      /* e(k-2)，Kd 非零时参与计算 */
    int32_t duty;           /* 当前输出，带符号，TIM1 比较值，范围 -SPEED_DUTY_MAX ~ +SPEED_DUTY_MAX */
    int32_t speed;          /* 最近一次实测速度 mm/s */
    uint16_t stall_ms;      /* 堵转累计时间，单位 ms */
    uint8_t started;        /* 1 = 已用前馈起头，避免输出过零时反复重置 */
} PidState_t;

static PidState_t s_pid[4];

static int16_t  s_target;               /* 直行目标车速 mm/s */
static int16_t  s_wheel_target[4];      /* 四轮目标线速度，单位 mm/s，带符号 */
static uint8_t  s_enable;               /* 1 闭环工作，0 输出断开 */
static uint8_t  s_fault;                /* 故障码 */
static uint16_t s_vbat_mv;              /* 电池电压 mV */

/* 运行期参数：初值取自 speed.h 的默认宏，可在线修改 */
int32_t g_speed_dead_zone    = SPEED_DEAD_ZONE;
int32_t g_speed_ff_gain_q10  = SPEED_FF_GAIN_Q10;
int32_t g_speed_kp_q16       = SPEED_KP_Q16;
int32_t g_speed_ki_q16       = SPEED_KI_Q16;
int32_t g_speed_kd_q16       = SPEED_KD_Q16;
int32_t g_speed_sync_on      = 1;       /* 默认开启左右同步补偿 */
int32_t g_speed_trim         = 0;       /* 直行偏置，单位 mm/s，默认 0，跑偏时手动调 */

/* 速度滑动平均缓冲（SPEED_AVG_N = 1 时不会被用到） */
#if (SPEED_AVG_N > 1)
static int32_t s_avg_buf[4][SPEED_AVG_N];
static uint8_t s_avg_idx;
static uint8_t s_avg_cnt;
static uint8_t s_avg_div;       /* 10 ms -> 20 ms 分频计数 */
#endif

/* ---------------- 私有函数 ---------------- */

/**
 * @brief  把编码器输出的浮点速度换算成整数 mm/s，并按安装方向统一符号
 * @param  v    该轮实测线速度，单位 cm/s（encoder.c 按 cm 周长换算得到）
 * @param  sign 方向系数，取 speed.h 的 SPEED_SIGN_xx：+1 保持，-1 取反（镜像安装的轮）
 * @return 该轮实测线速度，单位 mm/s，带符号（物理前进为正）
 */
static int32_t Speed_ToMmps(float v, int8_t sign)
{
    return (int32_t)(v * SPEED_SRC_SCALE) * (int32_t)sign;
}

/**
 * @brief  前馈估算：由目标速度直接给出一个"大致正确"的占空比起步值
 * @param  target 目标线速度，单位 mm/s；内部取绝对值，负目标按正转幅值处理
 * @return 前馈占空比，单位是 TIM1 比较值（0 ~ SPEED_DUTY_MAX）
 * @note   含死区补偿 + 与目标速度成正比的项 + 电池电压归一化；残差交给 PID
 */
static int32_t Speed_FeedForward(int16_t target)
{
    int32_t base;
    int32_t u;

    if (target < 0) target = -target;

    /* 死区 + 比例项 */
    base = g_speed_dead_zone
         + ((g_speed_ff_gain_q10 * (int32_t)target) >> 10);

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

/**
 * @brief  把带符号占空比翻译成方向 + PWM，写到对应轮的 M1~M4
 * @param  i    轮序，取 WHEEL_LF / WHEEL_LB / WHEEL_RF / WHEEL_RR
 * @param  duty 带符号占空比，单位是 TIM1 比较值，范围 -SPEED_DUTY_MAX ~ +SPEED_DUTY_MAX；
 *              正 = 该轮物理前进
 * @return 无
 * @note   motor.c 的约定：左轮 FORWARD 是前进，右轮 BACKWARD 才是前进（镜像安装）
 */
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

/**
 * @brief  单轮堵转判定（E-08）：占空比够大而该轮转速近零，累计 500 ms 置堵转故障
 * @param  i 轮序，取 WHEEL_xx；占空比与转速取自 s_pid[i]
 * @return 无（故障码写入模块内的 s_fault）
 * @note   按单轮判定而非整车车速，原地转向时四轮都在转，不会误报
 */
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

/**
 * @brief  清空一路 PID 状态（误差、输出、堵转计时、启动标志），回到"未启动"
 * @param  i 轮序，取 WHEEL_xx
 * @return 无
 * @note   换向、停车、故障时调用
 */
static void Speed_ResetPid(uint8_t i)
{
    s_pid[i].err_prev  = 0;
    s_pid[i].err_prev2 = 0;
    s_pid[i].duty      = 0;
    s_pid[i].stall_ms  = 0;
    s_pid[i].started   = 0;
}

/* ---------------- 对外函数 ---------------- */

/**
 * @brief  速度闭环初始化：四路 PID 清零、目标置 0、闭环关闭、电池电压取基准值
 * @param  无
 * @return 无
 * @note   须在 Motor_Init() 之后调用
 */
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

/**
 * @brief  设置直行目标速度，四轮同值
 * @param  mmps 目标线速度，单位 mm/s；内部取绝对值，非零时限幅到
 *              SPEED_TARGET_MIN ~ SPEED_TARGET_MAX
 * @return 无
 */
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

/**
 * @brief  设置原地转向目标：左右两侧速度反向，绕车体中心原地转
 * @param  dir  转向：1 = 原地左转（左侧后退、右侧前进），-1 = 原地右转，0 = 直行
 * @param  mmps 单侧轮速，单位 mm/s；内部取绝对值并限幅到 SPEED_TARGET_MIN ~ SPEED_TARGET_MAX
 * @return 无
 */
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

/**
 * @brief  直接指定四轮目标速度，供差速/全向上层使用
 * @param  w 长度 4 的数组，按 WHEEL_LF / WHEEL_LB / WHEEL_RF / WHEEL_RR 排列，
 *           单位 mm/s，带符号：正 = 该轮物理前进，负 = 后退
 * @return 无
 */
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

/**
 * @brief  开 / 关闭环输出
 * @param  on 1 = 使能闭环；0 = 断开输出（四路 PID 清零并把占空比写 0）
 * @return 无
 */
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

/**
 * @brief  停车：目标清零 + 四路 PID 清零 + 占空比写 0，不改闭环使能状态
 * @param  无
 * @return 无
 */
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

/**
 * @brief  清故障码与各路堵转累计时间，供故障复位时调用
 * @param  无
 * @return 无
 */
void Speed_ClearFault(void)
{
    uint8_t i;

    s_fault = SPEED_FAULT_NONE;

    for (i = 0; i < 4; i++)
    {
        s_pid[i].stall_ms = 0;
    }
}

/**
 * @brief  更新电池电压，供前馈做电压归一化
 * @param  mv 电池电压，单位 mV；传 0 视为无效值，忽略本次调用
 * @return 无
 */
void Speed_SetVbat(uint16_t mv)
{
    if (mv != 0)
    {
        s_vbat_mv = mv;
    }
}

/**
 * @brief  在线修改闭环参数，立即生效，下一个 10 ms 周期就按新值计算
 * @param  id    参数编号，取 SPEED_PARAM_DEAD_ZONE / FF_GAIN / KP / KI / KD / SYNC / TRIM
 * @param  value 参数值，单位随 id 而定：
 *               死区 = TIM1 比较值；FF_GAIN = Q10；Kp/Ki/Kd = Q16；
 *               SYNC = 0/1；TRIM = mm/s（允许负值）
 * @return 无
 * @note   除 TRIM 外的负值按 0 处理；超上限按各参数上限截断，仅防误发
 */
void Speed_SetParam(uint8_t id, int32_t value)
{
    /* trim 允许负数（往左修），其余参数不允许 */
    if (value < 0 && id != SPEED_PARAM_TRIM) value = 0;

    switch (id)
    {
    case SPEED_PARAM_DEAD_ZONE:
        if (value > 3000) value = 3000;
        g_speed_dead_zone = value;
        break;

    case SPEED_PARAM_FF_GAIN:
        if (value > 20480) value = 20480;       /* 20.0 × 1024 */
        g_speed_ff_gain_q10 = value;
        break;

    case SPEED_PARAM_KP:
        if (value > 1310720) value = 1310720;   /* 20.0 × 65536 */
        g_speed_kp_q16 = value;
        break;

    case SPEED_PARAM_KI:
        if (value > 655360) value = 655360;     /* 10.0 × 65536 */
        g_speed_ki_q16 = value;
        break;

    case SPEED_PARAM_KD:
        if (value > 655360) value = 655360;
        g_speed_kd_q16 = value;
        break;

    case SPEED_PARAM_SYNC:
        g_speed_sync_on = (value != 0) ? 1 : 0;
        break;

    case SPEED_PARAM_TRIM:
        if (value >  100) value =  100;
        if (value < -100) value = -100;
        g_speed_trim = value;
        break;

    default:
        break;
    }
}

/**
 * @brief  读取闭环参数当前值
 * @param  id 参数编号，取 SPEED_PARAM_xxx
 * @return 参数值，单位同 Speed_SetParam：死区 = TIM1 比较值，FF_GAIN = Q10，
 *         Kp/Ki/Kd = Q16，SYNC = 0/1，TRIM = mm/s；id 非法时返回 0
 */
int32_t Speed_GetParam(uint8_t id)
{
    switch (id)
    {
    case SPEED_PARAM_DEAD_ZONE: return g_speed_dead_zone;
    case SPEED_PARAM_FF_GAIN:   return g_speed_ff_gain_q10;
    case SPEED_PARAM_KP:        return g_speed_kp_q16;
    case SPEED_PARAM_KI:        return g_speed_ki_q16;
    case SPEED_PARAM_KD:        return g_speed_kd_q16;
    case SPEED_PARAM_SYNC:      return g_speed_sync_on;
    case SPEED_PARAM_TRIM:      return g_speed_trim;
    default:                    return 0;
    }
}

/**
 * @brief  闭环主运算：读四路实测速度 -> 航向修正 -> 逐轮增量式 PI -> 输出到电机
 * @param  无
 * @return 无
 * @note   固定 10 ms 调用一次（FR-12）；进入前须由 encoder 更新
 *         v_LF / v_LB / v_RF / v_RR（单位 cm/s，本函数内部换成 mm/s）
 */
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
    int16_t tgt_base[4];        /* 同步补偿后的四轮目标，单位 mm/s */
    int32_t v_left;
    int32_t v_right;
    int32_t v_adj;

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

    /* ---------- 3. 航向修正 ----------
     * 两级补偿，只在四轮目标完全一致（直行）时生效：
     *   固定偏置压机械偏差，动态同步压转动不一致。
     * 两者都只改变左右目标的分配，不改变总速度。 */
    for (i = 0; i < 4; i++)
    {
        tgt_base[i] = s_wheel_target[i];
    }

    /* 固定偏置：跑偏方向恒定时强行修正
     * 正值 = 左加右减 = 车往右修；负值相反。
     * 这是经验补偿，压的是机械偏差，不改总速度 */
    if (g_speed_trim != 0 &&
        s_wheel_target[WHEEL_LF] != 0 &&
        s_wheel_target[WHEEL_LF] == s_wheel_target[WHEEL_LB] &&
        s_wheel_target[WHEEL_LF] == s_wheel_target[WHEEL_RF] &&
        s_wheel_target[WHEEL_LF] == s_wheel_target[WHEEL_RR])
    {
        if (s_wheel_target[WHEEL_LF] > 0)
        {
            tgt_base[WHEEL_LF] += (int16_t)g_speed_trim;
            tgt_base[WHEEL_LB] += (int16_t)g_speed_trim;
            tgt_base[WHEEL_RF] -= (int16_t)g_speed_trim;
            tgt_base[WHEEL_RR] -= (int16_t)g_speed_trim;
        }
        else
        {
            tgt_base[WHEEL_LF] -= (int16_t)g_speed_trim;
            tgt_base[WHEEL_LB] -= (int16_t)g_speed_trim;
            tgt_base[WHEEL_RF] += (int16_t)g_speed_trim;
            tgt_base[WHEEL_RR] += (int16_t)g_speed_trim;
        }
    }

    if (g_speed_sync_on &&
        s_wheel_target[WHEEL_LF] != 0 &&
        s_wheel_target[WHEEL_LF] == s_wheel_target[WHEEL_LB] &&
        s_wheel_target[WHEEL_LF] == s_wheel_target[WHEEL_RF] &&
        s_wheel_target[WHEEL_LF] == s_wheel_target[WHEEL_RR])
    {
        v_left  = (s_pid[WHEEL_LF].speed + s_pid[WHEEL_LB].speed) / 2;
        v_right = (s_pid[WHEEL_RF].speed + s_pid[WHEEL_RR].speed) / 2;

        /* ×增益再 /2：Q8=256 时等于差值的一半 */
        v_adj = ((v_left - v_right) * SPEED_SYNC_GAIN_Q8) >> 9;

        if (v_adj >  SPEED_SYNC_MAX_ADJ) v_adj =  SPEED_SYNC_MAX_ADJ;
        if (v_adj < -SPEED_SYNC_MAX_ADJ) v_adj = -SPEED_SYNC_MAX_ADJ;

        if (s_wheel_target[WHEEL_LF] > 0)
        {
            tgt_base[WHEEL_LF] -= (int16_t)v_adj;
            tgt_base[WHEEL_LB] -= (int16_t)v_adj;
            tgt_base[WHEEL_RF] += (int16_t)v_adj;
            tgt_base[WHEEL_RR] += (int16_t)v_adj;
        }
        else
        {
            tgt_base[WHEEL_LF] += (int16_t)v_adj;
            tgt_base[WHEEL_LB] += (int16_t)v_adj;
            tgt_base[WHEEL_RF] -= (int16_t)v_adj;
            tgt_base[WHEEL_RR] -= (int16_t)v_adj;
        }

        for (i = 0; i < 4; i++)
        {
            if (tgt_base[i] >  SPEED_TARGET_MAX) tgt_base[i] =  SPEED_TARGET_MAX;
            if (tgt_base[i] < -SPEED_TARGET_MAX) tgt_base[i] = -SPEED_TARGET_MAX;
        }
    }

    /* ---------- 4. 逐轮增量式 PI ---------- */
    for (i = 0; i < 4; i++)
    {
        tgt = tgt_base[i];          /* 用补偿后的目标 */
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
            du = (g_speed_kp_q16 * (e - s_pid[i].err_prev)) >> 16;
            du += (g_speed_ki_q16 * e) >> 16;

            if (g_speed_kd_q16 != 0)
            {
                du += (g_speed_kd_q16 *
                       (e - 2 * s_pid[i].err_prev + s_pid[i].err_prev2)) >> 16;
            }
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

/**
 * @brief  读取当前直行目标速度
 * @param  无
 * @return 目标线速度，单位 mm/s，非负；转向时为单侧轮速幅值
 */
int16_t Speed_GetTarget(void)
{
    return s_target;
}

/**
 * @brief  读取某一路的实测线速度
 * @param  i 轮序，取 WHEEL_xx；越界按 0 处理
 * @return 实测线速度，单位 mm/s，带符号（物理前进为正）
 */
int32_t Speed_GetWheelSpeed(uint8_t i)
{
    if (i > 3) return 0;
    return s_pid[i].speed;
}

/**
 * @brief  读取某一路当前输出占空比
 * @param  i 轮序，取 WHEEL_xx；越界按 0 处理
 * @return 带符号占空比，单位是 TIM1 比较值，范围 -SPEED_DUTY_MAX ~ +SPEED_DUTY_MAX
 */
int16_t Speed_GetDuty(uint8_t i)
{
    if (i > 3) return 0;
    return (int16_t)s_pid[i].duty;
}

/**
 * @brief  读取当前故障码
 * @param  无
 * @return SPEED_FAULT_NONE(0) 正常；SPEED_FAULT_STALL(2) 堵转
 */
uint8_t Speed_GetFault(void)
{
    return s_fault;
}

/**
 * @brief  查询闭环是否处于使能状态
 * @param  无
 * @return 1 = 已使能；0 = 已断开输出
 */
uint8_t Speed_IsEnabled(void)
{
    return s_enable;
}

