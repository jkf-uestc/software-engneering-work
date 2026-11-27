#ifndef __MPU6050_H
#define __MPU6050_H

#include "stm32f10x.h"
#include "Delay.h"
#include "Speed.h"
#include "Tick.h"
#include "Encoder.h"


//选用I2C2
#define MPU_SCL_PIN GPIO_Pin_10    
#define MPU_SDA_PIN GPIO_Pin_11

#define MPU_ADDR 0x68 

#define MPU_REG_PWR_MGMT_1 0x6B      
#define MPU_REG_GYRO_CONFIG 0x1B
#define MPU_REG_ACCEL_CONFIG 0x1C      
#define MPU_REG_CONFIG 0x1A

#define	MPU_ACCEL_XOUT_H 0x3B
#define	MPU_ACCEL_XOUT_L 0x3C
#define	MPU_ACCEL_YOUT_H 0x3D
#define	MPU_ACCEL_YOUT_L 0x3E
#define	MPU_ACCEL_ZOUT_H 0x3F
#define	MPU_ACCEL_ZOUT_L 0x40
#define	MPU_TEMP_OUT_H 0x41
#define	MPU_TEMP_OUT_L 0x42
#define	MPU_GYRO_XOUT_H 0x43
#define	MPU_GYRO_XOUT_L 0x44
#define	MPU_GYRO_YOUT_H 0x45
#define	MPU_GYRO_YOUT_L 0x46
#define	MPU_GYRO_ZOUT_H 0x47
#define	MPU_GYRO_ZOUT_L	0x48

#define HEAD_STILL 0.0f //静止速度判定
#define TRACK_mm 200.f // 轮距
#define EXAMINE_T 0.2f //检查矫正时间间隔
#define WEIGHT 0.2f //关于轮速差的参考权重
#define MAXT 10000 //10s没到位就停车；含电机加速爬升时间
#define BUFFER  15.0f //缓冲减速阈值
#define TOLERANT 1.5f //偏差不大于1.5°就认为到位，其实断电后还有滑行

/*==================== 卡尔曼滤波参数（双状态：航向角 a + 陀螺零偏 b）====================*/
/* 每步 W：过程噪声方差(deg^2)，陀螺积分本身的噪声，越小越信陀螺 */
#define KF_W          0.01f
/* Φ：观测噪声方差(deg^2)。观测 = 左右轮速差积分出来的里程计航向角 */
#define KF_PHI_NORM   0.05f      /* 直行、里程计可信 */
#define KF_PHI_SLIP   1.0f     /* 检测到打滑：放宽但仍参与修正 */
#define KF_PHI_TURN   5.0f    /* 原地转向中：基本不信里程计，只靠陀螺 */
/* 打滑判据：里程计角速度和陀螺角速度差超过这个值(deg/s) 就判打滑 */
#define KF_SLIP_DEG   6.0f
/* Φ 恢复时间常数(s)：打滑结束后可信度不是瞬间回来，按这个时间常数慢慢降 */
#define KF_PHI_TAU    0.3f
/* 观测有效性：两轮转速都低于这个值(mm/s)就认为车没在滚，里程计没有读数 */
#define KF_WHEEL_MOVE 60.0f
/* 打滑判据的上限：超过就认为里程计完全不可用 */
#define KF_TURN_LIMIT KF_PHI_TURN
/* 初始方差 */
#define KF_A0         0.02f     /* 航向角 deg^2 */
#define KF_B0         0.1f     /* 零偏 (deg/s)^2 */

typedef struct
{
	int16_t acc_x, acc_y, acc_z;        
	int16_t temp;                       
	int16_t gyro_x, gyro_y, gyro_z;     
} MPU6050_Raw_t;

void MPU6050_Init(void);                     
void MPU6050_ReadRaw(MPU6050_Raw_t *out);
void Heading_Init(void);
void Heading_Calibrate(void);
void Heading_Update(void);
float Heading_GetYaw(void);
#define TURN_IDLE 0 //待命
#define TURN_RUN  1 //正在转
#define TURN_OK   2 //到位
#define TURN_FAIL 3 //超时
void    Heading_TurnStart(float deg,int16_t mmps); //非阻塞：启动一次定量转向
void    Heading_TurnAbort(void);                   //立刻刹车并退出转向
uint8_t Heading_TurnTask(void);                    //放 10ms 任务里，返回上面 4 个状态
uint8_t Heading_TurnState(void);                   //只查状态，不改

#endif
