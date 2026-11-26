
#include "stm32f10x.h"                  // Device header

/* 模块接口（对应你提供的 .c 文件） */
#include "Delay.h"
#include "Tick.h"                       // DWT 毫秒/周期计数、Tick_Every 定时
#include "LED.h"                        // 就绪 / 运行 / 故障 三灯
#include "motor.h"                      // TIM1 四路 PWM + 方向
#include "encoder.h"                    // 3 路硬件编码器 + 右后软件解码
#include "speed.h"                      // 四轮速度环（增量式 PI + 前馈）
#include "Key.h"                        // 单键：短按换挡、长按 3 秒停车
#include "Sonar.h"                      // 超声波
#include "Avoid.h"                      // 避障阻塞
#include "BlueTooth.h"                  // USART2 蓝牙遥控（PA2/PA3）

/*==============================================================================
 *  MiniCar 主函数
 *------------------------------------------------------------------------------
 *  硬件依据：《STM32F103C8T6 引脚分配表 V2.0》
 *
 *  时间基准：全部来自 Tick.c —— DWT->CYCCNT 轮询，**不开 SysTick 中断**。
 *            所以本文件没有 SysTick_Handler，也没有任何定时器中断参与调度。
 *
 *  各模块的节拍归属（很重要，别在主循环里重复驱动）：
 *      Tick_Init()          DWT 时基，必须最先调用
 *      Key_Task()           内部自带 5ms 节拍（Tick_Every）
 *      Sonar_Task()         内部自带 20ms 节拍，并且会自己发下一次触发
 *      Speed_Calc_20ms()    需要主循环每 20ms 调一次
 *      Speed_Tick()         需要主循环每 10ms 调一次（内部按 SPEED_TICK_MS 累计堵转）
 *      LED_Update(now)      非阻塞，每轮调用即可（靠相位算闪烁）
 *      Avoid_Task()         每轮调用，但必须排在 Sonar_Task() 之后
 *
 *  控制优先级（从高到低）：
 *      速度环故障 > 避障阻塞 > 蓝牙遥控 > 按键档位
 *
 *  显示类外设不做：PB8/PB9 已分给右前轮的 DIR_RF1/DIR_RF2，
 *  没有空闲引脚再挂 I2C/SPI 屏。
 *============================================================================*/

/*------------------------------ 可调参数 -----------------------------------*/

/* 按键档位速度在 Key.c 里定义：{0, 300, 500, 800} mm/s
   蓝牙遥控的三个档位复用同一组值，手感一致 */
#define BT_SPEED_LOW		300		/* 蓝牙前进/后退 低档 mm/s */
#define BT_SPEED_MED		500		/* 中档 */
#define BT_SPEED_HIGH		800		/* 高档 */
#define BT_TURN_MMPS		400		/* 蓝牙原地转向时单轮速度 mm/s */

/* 圆周率，只用来算测速系数 */
#define CAR_PI			3.14159265f

/* 主循环两个硬节拍，必须和 speed.c 里的假设一致 */
#define ENC_PERIOD_MS		20		/* 测速周期 */
#define SPEED_LOOP_MS		10		/* 速度环周期 */

/* 蓝牙动作编号直接用 BlueTooth.h 里的宏（ForwardHigh=32 等），
   不再在本文件重复定义，避免协议改了两边不一致。 */

/* 注意 BlueTooth.h 里的 HighSpeed=3599 / MediumSpeed=1800 / LowSpeed=900
   是"开环占空比"（0~3599），不是 mm/s，不要拿来当 Speed_SetTarget 的输入。
   本工程走的是闭环速度控制，所以下面的蓝牙档位用 mm/s。 */

/*------------------------------ 运行状态 -----------------------------------*/
/* 内部 4 个状态，但对外只有 3 种灯（引脚表只留了 RDY/RUN/ERR 三盏）：
   避障阻塞时车已停住、在等指令，用"就绪灯常亮"表示，不占用故障灯 */
typedef enum
{
	CAR_READY = 0,			/* 待机：档位 0，等按键 */
	CAR_RUN,				/* 运行：速度环工作 */
	CAR_BLOCKED,			/* 避障阻塞：已刹车，等指令解除 */
	CAR_FAULT				/* 故障：速度环报警（堵转等） */
} CarState_t;

static CarState_t s_state   = CAR_READY;

/* 蓝牙遥控：收到遥控指令后目标由蓝牙接管，直到收到"停止" */
static uint8_t s_bt_active  = 0;	/* 1=蓝牙接管控制 */
static int16_t s_bt_target  = 0;	/* 蓝牙目标速度 mm/s，负=后退 */
static int8_t  s_bt_turn    = 0;	/* +1 原地左转，-1 原地右转，0 直行 */
static uint8_t s_gear_prev  = 0;	/* 上一轮档位，用来判断用户是否动过按键 */

/*------------------------------ 内部函数 -----------------------------------*/

static void BlueTooth_Task(void);
static void Car_Enter(CarState_t next);
static void Car_Update(void);
static void Car_SetReverse(int16_t mmps);

/*============================================================================*/
/*                                   main                                     */
/*============================================================================*/

int main(void)
{
	/* 这两个是 Tick_Every 的"上次触发时刻"，必须活着整个程序生命周期，
	   绝不能放到循环体内 —— 每次重置成 0 会让 Tick_Every 永远判"该执行了" */
	uint32_t t_enc = 0;			/* 20ms 测速节拍 */
	uint32_t t_spd = 0;			/* 10ms 速度环节拍 */

	/*---------------------- 一、底层与外设初始化 ----------------------*/
	/* Tick 必须最先：Key_Task / Sonar_Task / LED_Update 全都靠它取时间 */
	Tick_Init();

	/* LED_Init 内部做两件关键事：
	     1) 关 LSE，释放 PC14/PC15（引脚表要求）
	     2) 关 JTAG（SWJ_CFG=010），释放 PB3 —— 否则故障灯永远是 JTDO，不会亮 */
	LED_Init();
	LED_AllOff();

	Motor_Init();					/* TIM1 CH1~CH4，20kHz，ARR=3599，内部已关 JTAG */

	TIM2_Encoder_Init();			/* 左前 PA0/PA1 */
	TIM3_Encoder_Init();			/* 左后 PA6/PA7 */
	TIM4_Encoder_Init();			/* 右前 PB6/PB7 */
	EXTI_RR_Encoder_Init();			/* 右后 PB12/PB13 软件四倍频 */

	Key_Init();						/* PA5 ADC 梯形键盘 */
	Sonar_Init();					/* PB5 TRIG / PB4 ECHO */
	/* ---- 测速系数必须由 main 给定（Encoder.c 里没有初值，默认 0）----
	 * encoder.c 只做 v = 脉冲增量 * g_speed_coeff，系数留空等上层填。
	 * 若漏了这一步：v_LF~v_RR 恒为 0 -> 速度环以为是静止 ->
	 * 积分一路涨到满占空比 -> 车全速冲出去，且完全不受控。
	 * 公式来自 speed.h：
	 *   g_speed_coeff = (pi * 轮径cm) / (每圈脉冲数 * 采样周期s)
	 *                 = (pi * 6.5) / (1320 * 0.02) = 0.7735 cm/s per count
	 * 单位是 cm/s，speed.c 内部再乘 SPEED_SRC_SCALE(=10) 转成 mm/s。
	 * 换轮子/换减速比时只改 Encoder.h 里的 PULSE_PER_REV、WHEEL_D_CM，
	 * 这里会自动跟着变。 */
	g_speed_coeff = (CAR_PI * WHEEL_D_CM) / (PULSE_PER_REV * SAMPLE_T);

	Speed_Init();					/* 四轮速度环 */
	/* 喂一个初始电池电压做前馈归一化。speed.h 的集成步骤要求这一句；
	   接好 ADC 后改成每 200ms 喂一次实测值即可。 */
	Speed_SetVbat(12000);
	BlueTooth_Init();				/* USART2，PA2 TX / PA3 RX，9600 8N1 */

	/*------------------------ 二、上电安全状态 ------------------------*/
	/* 先让电机彻底不动，再点亮"就绪"，之后才进主循环 */
	Car_Stop();
	Speed_Stop();
	Speed_Enable(0);
	LED_SetMode(LED_MODE_READY);
	s_state = CAR_READY;

	/*---------------------------- 三、主循环 --------------------------*/
	while (1)
	{
		uint32_t now;
		uint8_t  gear;

		now = Tick_GetMs();

		/*=================== 1. 指示灯：非阻塞刷新 ===================*/
		/* 每轮调用，内部按相位算闪烁，不等待、不占 CPU */
		LED_Update(now);

		/*=================== 2. 按键：内部自带 5ms ===================*/
		/* 短按换挡（0->1->2->3->0），长按 3 秒立刻归零停车。
		   档位一变说明用户主动操作过，顺手解除避障阻塞（Avoid.c 要求
		   阻塞必须由外部指令清除，不会自己解除）。 */
		Key_Task();
		gear = Key_GetGear();
		if (gear != s_gear_prev)
		{
			s_gear_prev = gear;
			Avoid_ClearBlock();
			Speed_ClearFault();		/* 用户动过按键 = 明确的重置动作 */
		}

		/*=========== 3. 测速 20ms -> 速度环 10ms（顺序不能反）===========*/
		/* Speed_Tick() 用的反馈是 encoder 算出的 v_LF~v_RR，
		   所以必须先更新反馈再算 PI。两个 if 同拍触发时上面的先执行，
		   正好满足这个顺序。 */
		if (Tick_Every(&t_enc, ENC_PERIOD_MS))
		{
			Speed_Calc_20ms();
		}
		if (Tick_Every(&t_spd, SPEED_LOOP_MS))
		{
			Speed_Tick();
		}

		/*=========== 4. 超声波 + 避障：顺序固定，不能调换 ===========*/
		/* Sonar_Task 自带 20ms 节拍，并在末尾自动发下一次触发；
		   Avoid_Task 读的必须是刚刷新完的距离，所以必须紧跟其后 */
		Sonar_Task();
		Avoid_Task();

		/*=================== 5. 蓝牙遥控 ===================*/
		BlueTooth_Task();

		/*=================== 6. 状态机 ===================*/
		Car_Update();
	}
}

/*============================================================================*/
/*                            蓝牙遥控任务                                    */
/*============================================================================*/

/**
  * 函    数：处理一帧蓝牙遥控指令
  * 参    数：无
  * 返 回 值：无
  * 注意事项：BlueTooth_GetRxFlag() 读到 1 时会自动清标志，一帧只处理一次；
  *           BlueTooth_action() 比较的是最近一次收到的 4 字节数据包。
  */
static void BlueTooth_Task(void)
{
	int act;

	if (!BlueTooth_GetRxFlag())
	{
		return;						/* 没收到新数据，直接返回，不耽误主循环 */
	}

	act = BlueTooth_action();

	if (act < 0)
	{
		return;
	}

	/* 障碍移开后，需要收到新的运动指令才允许恢复。 */
	if (act != Stop && Avoid_IsBlocked())
	{
		if (Sonar_GetStatus() != SONAR_ST_OK ||
			Sonar_GetMm() <= AVOID_OBSTACLE_THRESHOLD_MM)
		{
			return;
		}
		Avoid_ClearBlock();
	}

	switch (act)
	{
		case ForwardLow:
			s_bt_active = 1; s_bt_turn = 0; s_bt_target = BT_SPEED_LOW;
			break;
		case ForwardMedium:
			s_bt_active = 1; s_bt_turn = 0; s_bt_target = BT_SPEED_MED;
			break;
		case ForwardHigh:
			s_bt_active = 1; s_bt_turn = 0; s_bt_target = BT_SPEED_HIGH;
			break;

		case BackLow:
			s_bt_active = 1; s_bt_turn = 0; s_bt_target = -BT_SPEED_LOW;
			break;
		case BackMedium:
			s_bt_active = 1; s_bt_turn = 0; s_bt_target = -BT_SPEED_MED;
			break;
		case BackHigh:
			s_bt_active = 1; s_bt_turn = 0; s_bt_target = -BT_SPEED_HIGH;
			break;

		case TurnLeft:
			s_bt_active = 1; s_bt_turn = 1; s_bt_target = 0;
			break;
		case TurnRight:
			s_bt_active = 1; s_bt_turn = -1; s_bt_target = 0;
			break;

		case Stop:
			/* 收到停止：交还按键控制，并解除避障阻塞 */
			s_bt_active = 0;
			s_bt_turn   = 0;
			s_bt_target = 0;
			Avoid_ClearBlock();
			Speed_ClearFault();		/* 遥控端按停止 = 明确的重置动作 */
			break;

		default:
			break;					/* 未知动作，忽略 */
	}
}

/*============================================================================*/
/*                               状态机                                       */
/*============================================================================*/

/**
  * 函    数：进入指定状态（所有副作用集中在这里）
  * 参    数：next 目标状态
  * 返 回 值：无
  * 注意事项：灯、电机、速度环统一在这里配置，避免迁移时漏步骤。
  *           CAR_BLOCKED / CAR_FAULT 必须 Speed_Enable(0)：Avoid_Task 虽然调了
  *           Car_Stop()，但只要速度环还开着，下一拍 Speed_Tick() 就会把 PWM
  *           重新加上去，车会往前顶。
  */
static void Car_Enter(CarState_t next)
{
	if (next == s_state)
	{
		return;						/* 状态没变，不要反复复位电机和灯 */
	}
	s_state = next;

	switch (next)
	{
		case CAR_READY:					/* 待机：就绪灯常亮 */
			Speed_Enable(0);
			Speed_Stop();
			Car_Stop();
			LED_SetMode(LED_MODE_READY);
			break;

		case CAR_RUN:					/* 运行：运行灯 1Hz 闪 */
			Speed_Enable(1);
			LED_SetMode(LED_MODE_RUN);
			break;

		case CAR_BLOCKED:				/* 避障阻塞：已停，等指令 */
			Speed_Enable(0);
			Speed_Stop();
			Car_Stop();
			LED_SetMode(LED_MODE_READY);	/* 只有三盏灯，用就绪表示"停着等" */
			break;

		case CAR_FAULT:					/* 故障：故障灯 2Hz 闪 */
			Speed_Enable(0);
			Speed_Stop();
			Car_Stop();
			LED_SetMode(LED_MODE_FAULT);
			break;
	}
}

/**
  * 函    数：四轮后退
  * 参    数：mmps 目标速度，**必须为负**（speed.h 约定：负 = 后退）
  * 返 回 值：无
  * 注意事项：不能用 Speed_SetTarget()，它会把负值限幅成 +100（前进）。
  *           Speed_SetWheelTarget() 不做限幅，四个轮子直接给带符号目标。
  */
static void Car_SetReverse(int16_t mmps)
{
	int16_t w[4];

	w[0] = mmps;
	w[1] = mmps;
	w[2] = mmps;
	w[3] = mmps;
	Speed_SetWheelTarget(w);
}

/**
  * 函    数：状态机主判断（每轮主循环调用一次）
  * 参    数：无
  * 返 回 值：无
  * 注意事项：优先级 故障 > 阻塞 > 运行 > 待机。
  *           这是唯一决定状态的地方，别在别处直接改 s_state。
  */
static void Car_Update(void)
{
	int16_t target;

	/*================== 1. 速度环故障（最高优先级） ==================*/
	/* 堵转等由 speed.c 自己判定，主循环只负责读和展示。
	   故障只能靠 Speed_ClearFault() 清除，这里不自动恢复，防止带病运行。 */
	if (Speed_GetFault() != SPEED_FAULT_NONE)
	{
		Car_Enter(CAR_FAULT);
		return;
	}

	/*====================== 2. 避障阻塞 ======================*/
	/* Avoid_Task 在触发瞬间已经刹过车，这里只负责切换状态 */
	if (Avoid_IsBlocked())
	{
		Car_Enter(CAR_BLOCKED);
		return;
	}

	/*================ 3. 解析目标速度（蓝牙优先） ================*/
	if (s_bt_active)
	{
		target = s_bt_target;
	}
	else
	{
		target = Key_GetSpeed();		/* 按键档位：0 / 300 / 500 / 800 mm/s */
	}

	/*=================== 4. 目标为 0 -> 待机 ===================*/
	if (target == 0 && s_bt_turn == 0)
	{
		Car_Enter(CAR_READY);
		return;
	}

	/*========================= 5. 运行 =========================*/
	Car_Enter(CAR_RUN);

	/* 三类指令分别走不同接口，别混用：
	     Speed_SetTurn(dir, mmps)  原地转向：dir>0 左转，dir<0 右转
	     Speed_SetTarget(mmps)     只接受正的直行目标，内部限幅 100~800
	     Speed_SetWheelTarget(w)   四轮任意带符号目标，后退只能走这个

	   注意 Speed_SetTarget() 会被限幅：
	       if (mmps != 0 && mmps < SPEED_TARGET_MIN) mmps = SPEED_TARGET_MIN;
	   所以传 -300 会被改成 +100（前进），倒车必须用 WheelTarget。 */
	if (s_bt_turn > 0)
	{
		Speed_SetTurn(1, BT_TURN_MMPS);
	}
	else if (s_bt_turn < 0)
	{
		Speed_SetTurn(-1, BT_TURN_MMPS);
	}
	else if (target > 0)
	{
		Speed_SetTarget(target);
	}
	else
	{
		Car_SetReverse(target);		/* target 为负 */
	}
}
