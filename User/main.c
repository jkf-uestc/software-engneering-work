
#include "stm32f10x.h"                  // Device header

/* 模块接口 */
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
#include "MPU6050.h"			// 姿态控制

/*==============================================================================
 *  MiniCar 主函数
 *  引脚见《STM32F103C8T6 引脚分配表 V2.0》
 *
 *  时基：Tick.c，DWT 轮询，不开 SysTick 中断
 *  节拍：速度环 10ms、测速 20ms、键盘 5ms、超声 20ms（后两个模块内部自己限速）
 *  优先级：故障 > 避障阻塞 > 蓝牙 > 按键
 *============================================================================*/

/*------------------------------ 可调参数 -----------------------------------*/

/* 蓝牙档位，和 Key.c 里的档位速度一致 */
#define BT_SPEED_LOW		300		/* 蓝牙前进/后退 低档 mm/s */
#define BT_SPEED_MED		500		/* 中档 */
#define BT_SPEED_HIGH		800		/* 高档 */
#define BT_TURN_MMPS		200		/* 定量转向速度 mm/s：实测 400 滑行多、200 更准 */
#define BT_TURN_DEG		90.0f	/* 左右键按一下转的角度（本车 deg>0 = 顺时针） */

/* 圆周率，只用来算测速系数 */
#define CAR_PI			3.14159265f

/* 主循环两个硬节拍，必须和 speed.c 里的假设一致 */
#define ENC_PERIOD_MS		20		/* 测速周期 */
#define SPEED_LOOP_MS		10		/* 速度环周期 */

/* 动作编号用 BlueTooth.h 的宏。注意 HighSpeed 那组是占空比，不是 mm/s */

/*------------------------------ 运行状态 -----------------------------------*/
/* 内部 4 个状态，灯只有就绪/运行/故障三种，阻塞时用就绪灯表示 */
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
static void Task_Run(void);
static void Task_Led(void);
static void Task_Key(void);
static void Task_Enc(void);
static void Task_Speed(void);
static void Task_Turn(void);
static void Task_Sonar(void);
static void Task_Avoid(void);
static void Task_Bt(void);
static void Task_Car(void);
/*============================================================================*/
/*                                   main                                     */
/*============================================================================*/

float g_dbg_yaw = 0.0f;		/* 调试：yaw 抄到全局，程序跑着也能在 Watch 里看 */
int main(void)
{
	/*---------------------- 一、底层与外设初始化 ----------------------*/
	/* NVIC 分组要在所有 NVIC_Init 之前设好。
	   Encoder.c、Sonar.c 比 BlueTooth.c 先配中断，那时还是默认分组 0，
	   抢占优先级会写错，所以先设成分组 2。 */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	/* 时基，必须最先初始化 */
	Tick_Init();

	/* 内部会关 LSE 和 JTAG，PB3 才能当普通 IO */
	LED_Init();
	LED_AllOff();

	Motor_Init();					/* TIM1 CH1~CH4，20kHz，ARR=3599，内部已关 JTAG */

	TIM2_Encoder_Init();			/* 左前 PA0/PA1 */
	TIM3_Encoder_Init();			/* 左后 PA6/PA7 */
	TIM4_Encoder_Init();			/* 右前 PB6/PB7 */
	EXTI_RR_Encoder_Init();			/* 右后 PB12/PB13 软件四倍频 */

	Key_Init();						/* PA5 按键 */
	Sonar_Init();					/* PB5 TRIG / PB4 ECHO */
	/* 测速系数，Encoder.c 里没有初值，必须由这里给 */
	g_speed_coeff = (CAR_PI * WHEEL_D_CM) / (PULSE_PER_REV * SAMPLE_T);

	Speed_Init();					/* 四轮速度环 */
	/* 电池电压，前馈用；以后接 ADC 改成实测值 */
	Speed_SetVbat(12000);
	BlueTooth_Init();				/* USART2，PA2 TX / PA3 RX，9600 8N1 */

	/*------------------------ 二、上电安全状态 ------------------------*/
	/* 先让电机彻底不动，再点亮"就绪"，之后才进主循环 */
	Car_Stop();
	Speed_Stop();
	Speed_Enable(0);
	LED_SetMode(LED_MODE_READY);
	s_state = CAR_READY;
	
	MPU6050_Init();
	Delay_ms(500);				/*避免按键余震造成影响*/
	Heading_Init();
	Heading_Calibrate();
	
	Heading_TurnStart(90.0f,200);	/* 上电自检：定量转 90 度，非阻塞，由 Task_Turn 推进 */


	/*---------------------------- 三、主循环 --------------------------*/
	/* 裸机多任务：任务表统一调度，所有任务都不阻塞 */
	while (1)
	{
		Task_Run();
	}
}

/*============================================================================*/
/*                            蓝牙遥控任务                                    */
/*============================================================================*/

/**
  * 函    数：解析一帧蓝牙遥控指令
  * 参    数：无
  * 返 回 值：无
  */
static void BlueTooth_Task(void)
{
	int act;

	if (!BlueTooth_GetRxFlag())
	{
		return;						/* 没有新数据 */
	}

	/* 自定义角度帧：FF 02 XX 00 00 FE 顺时针 XX 度，FF 03 为逆时针 */
	if ((BlueTooth_RxPacket[0]==0x02 || BlueTooth_RxPacket[0]==0x03)
		&& BlueTooth_RxPacket[2]==0x00
		&& BlueTooth_RxPacket[3]==0x00
		&& BlueTooth_RxPacket[1]>=0x01
		&& BlueTooth_RxPacket[1]<=0xB4)
	{
		float d=(float)BlueTooth_RxPacket[1];

		/* 正在转的时候不接新角度 */
		if (Heading_TurnState()==TURN_RUN)
		{
			return;
		}

		s_bt_turn=0;
		Avoid_ClearBlock();
		if (BlueTooth_RxPacket[0]==0x02) {Heading_TurnStart(d,BT_TURN_MMPS);}	/* 正 = 顺时针 */
		else                            {Heading_TurnStart(-d,BT_TURN_MMPS);}	/* 负 = 逆时针 */
		return;
	}	act = BlueTooth_action();

	/* 转向中只处理 Stop，别的帧直接丢掉 */
	if (act != Stop && Heading_TurnState() == TURN_RUN)
	{
		return;
	}

	if (act < 0)
	{
		return;
	}

	/* 障碍移开后要重新收到运动指令才恢复 */
	if (act != Stop && Avoid_IsBlocked())
	{
		if (Sonar_GetStatus() != SONAR_ST_OK ||
			Sonar_GetMm() <= AVOID_OBSTACLE_THRESHOLD_MM)
		{
			/* 障碍还在，指令作废，同时清掉旧目标 */
			s_bt_active = 0;
			s_bt_turn   = 0;
			s_bt_target = 0;
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
			/* 转向会清掉之前的运动目标，转完停在原地 */
			s_bt_active = 0;
			s_bt_turn   = 0;
			s_bt_target = 0;
			Avoid_ClearBlock();
			Heading_TurnStart(-BT_TURN_DEG, BT_TURN_MMPS);	/* 左转 90 度（本车 deg<0 才是左） */
			break;
		case TurnRight:
			/* 转向是一次性动作，不能把之前的运动目标带过来。
			   原来写着"保留 s_bt_active/s_bt_target，转完接着执行上一条
			   运动指令"，实测后果：避障停车前点过前进，再点左转，转完
			   车会自己往前冲 —— 就是"自动执行刚才点的前进"。
			   现在转向即清目标，转完停在原地等新指令。 */
			s_bt_active = 0;
			s_bt_turn   = 0;
			s_bt_target = 0;
			Avoid_ClearBlock();
			Heading_TurnStart(BT_TURN_DEG, BT_TURN_MMPS);	/* 按一下 = 车往右转 90 度 */
			break;

		case Stop:
			Heading_TurnAbort();	/* 定量转向进行中也要能一按就停 */
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
  * 函    数：切换状态，灯和电机在这里统一设置
  * 参    数：next 目标状态
  * 返 回 值：无
  */
static void Car_Enter(CarState_t next)
{
	if (next == s_state)
	{
		return;						/* 状态没变就不重复设置 */
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
			LED_SetMode(LED_MODE_READY);	/* 用就绪灯表示停着等 */
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
  * 参    数：mmps 目标速度，取负值
  * 返 回 值：无
  * 说    明：Speed_SetTarget 会把负值限幅成 100，所以后退走 WheelTarget
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
  * 函    数：状态机判断，每轮调用一次
  * 参    数：无
  * 返 回 值：无
  */
static void Car_Update(void)
{
	int16_t target;

	/*================== 1. 速度环故障（最高优先级） ==================*/
	/* 故障由 speed.c 判定，这里只读 */
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

	/* 转向、前进、后退分别用不同接口 */
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

/*============================================================================*/
/*                      裸机多任务：任务表 + 统一调度                         */
/*============================================================================*/
/* 任务表：period=0 表示每轮都跑，大于 0 交给 Tick_Every 限速 */
typedef struct
{
	void (*fn)(void);
	uint32_t period_ms;
	uint32_t t_last;
} Task_t;

static Task_t s_tasks[] =
{
	{ Task_Led,     0,             0 },	/* 灯：非阻塞刷新 */
	{ Task_Key,     0,             0 },	/* 键：内部自带 5ms */
	{ Task_Enc,     ENC_PERIOD_MS, 0 },	/* 20ms：测速 + 航向积分 */
	{ Task_Speed,   SPEED_LOOP_MS, 0 },	/* 10ms：速度环 */
	{ Task_Turn,    10,            0 },	/* 10ms：定量转向状态机 */
	{ Task_Sonar,   0,             0 },	/* 超声：内部自带 20ms */
	{ Task_Avoid,   0,             0 },	/* 避障：必须紧跟超声 */
	{ Task_Bt,      0,             0 },	/* 蓝牙遥控 */
	{ Task_Car,     0,             0 },	/* 整车状态机 */
};

static void Task_Run(void)
{
	uint8_t i;
	for (i=0;i<sizeof(s_tasks)/sizeof(s_tasks[0]);i++)
	{
		if (s_tasks[i].period_ms==0)
		{
			s_tasks[i].fn();
		}
		else if (Tick_Every(&s_tasks[i].t_last,s_tasks[i].period_ms))
		{
			s_tasks[i].fn();
		}
	}
}

static void Task_Led(void)
{
	LED_Update(Tick_GetMs());
}

static void Task_Key(void)
{
	uint8_t gear;
	Key_Task();
	gear=Key_GetGear();
	if (gear!=s_gear_prev)
	{
		s_gear_prev=gear;

		/* 动过按键就把控制权从蓝牙收回，并清掉旧目标 */
		s_bt_active=0;
		s_bt_turn=0;
		s_bt_target=0;

		/* 转向中按键也能立刻停 */
		Heading_TurnAbort();
		Avoid_ClearBlock();
		Speed_ClearFault();
	}
}

static void Task_Enc(void)
{
	Speed_Calc_20ms();
	Heading_Update();	/* 必须排在测速之后：要用刚刷新的轮速 */
}

static void Task_Speed(void)
{
	Speed_Tick();
}

static void Task_Turn(void)
{
	if (Heading_TurnTask()==TURN_FAIL)
	{
		/* 转向超时：收回转向状态，交回状态机 */
		Heading_TurnAbort();
		Car_Enter(CAR_READY);
	}
}

/* 转向时超声和避障不跑：车在原地转，朝向一直在变 */
static void Task_Sonar(void)
{
	if (Heading_TurnState()==TURN_RUN) {return;}
	Sonar_Task();
}

static void Task_Avoid(void)
{
	if (Heading_TurnState()==TURN_RUN) {return;}
	Avoid_Task();
}

static void Task_Bt(void)
{
	/* 转向中也要把帧消费掉，不能留着 */
	BlueTooth_Task();
}

static void Task_Car(void)
{
	g_dbg_yaw = Heading_GetYaw();	/* 调试：每轮抄一份，Watch 里能看 */
	if (Heading_TurnState()==TURN_RUN) {return;}	/* 转向期间转向任务独占电机 */
	Car_Update();
}
