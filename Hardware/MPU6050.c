#include "MPU6050.h"

static float yaw; //状态 a：航向角(deg)
static float bias; //标定出的陀螺零偏(raw LSB)，只作记录
static float b_est; //状态 b：零偏估计(deg/s)，和 yaw 同向
static float kf_A,kf_B,kf_C; //方差 A(deg^2)、B((deg/s)^2)、协方差 C
static float phi_f; //当前生效的观测噪声方差 Φ（打滑后按时间常数恢复）
static uint32_t s_t_last; //上次积分时刻，算真实 dt 用
static uint8_t s_turn_state=TURN_IDLE; //转向状态机：0待命 1运行 2到位 3超时
static float s_turn_goal; //这一次要转到的瞬时目标
static int16_t s_turn_mmps;
static int8_t s_turn_dir0,s_turn_dir; //起跑方向 / 当前方向
static uint32_t s_turn_t0;

static void MPU_WriteReg(uint8_t reg, uint8_t val)
{
	while (I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY));
	I2C_GenerateSTART(I2C2, ENABLE);
	
	while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));
	I2C_Send7bitAddress(I2C2, MPU_ADDR << 1, I2C_Direction_Transmitter);
	
	while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	I2C_SendData(I2C2, reg);
	
	while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTING));
	I2C_SendData(I2C2, val);
	
	while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	I2C_GenerateSTOP(I2C2, ENABLE);
}

void MPU6050_ReadRaw(MPU6050_Raw_t *out)
{
	uint8_t buf[14];
	uint8_t i;

	while (I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY));
	I2C_GenerateSTART(I2C2, ENABLE);
	
	while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));
	I2C_Send7bitAddress(I2C2, MPU_ADDR << 1, I2C_Direction_Transmitter);
	
	while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	I2C_SendData(I2C2, MPU_ACCEL_XOUT_H);
	
	while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	I2C_GenerateSTART(I2C2, ENABLE);
	
	while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));
	I2C_Send7bitAddress(I2C2, MPU_ADDR << 1, I2C_Direction_Receiver);
	
	while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
	I2C_AcknowledgeConfig(I2C2, ENABLE);
	
	for(i=0;i<14;i++)
	{
		if (i==13)								
		{
			I2C_AcknowledgeConfig(I2C2, DISABLE);	
			I2C_GenerateSTOP(I2C2, ENABLE);			
		}
		while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED));
		buf[i] = I2C_ReceiveData(I2C2);
	}
	I2C_AcknowledgeConfig(I2C2, ENABLE);			

	out->acc_x=(int16_t)((buf[0]<<8) | buf[1]);
	out->acc_y=(int16_t)((buf[2]<<8) | buf[3]);
	out->acc_z=(int16_t)((buf[4]<<8) | buf[5]);
	out->temp=(int16_t)((buf[6]<<8) | buf[7]);
	out->gyro_x=(int16_t)((buf[8]<<8) | buf[9]);
	out->gyro_y=(int16_t)((buf[10]<<8) | buf[11]);
	out->gyro_z=(int16_t)((buf[12]<<8) | buf[13]);
}


void MPU6050_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitTypeDef gpio;
	gpio.GPIO_Mode=GPIO_Mode_AF_OD;
	gpio.GPIO_Pin=MPU_SCL_PIN | MPU_SDA_PIN;
	gpio.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&gpio);
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
	I2C_InitTypeDef i2c;
	i2c.I2C_Mode=I2C_Mode_I2C;
	i2c.I2C_ClockSpeed=400000;
	i2c.I2C_DutyCycle=I2C_DutyCycle_2;
	I2C_Init(I2C2,&i2c);
	
	I2C_Cmd(I2C2,ENABLE);
	
	MPU_WriteReg(MPU_REG_PWR_MGMT_1,0x80);//复位
	Delay_ms(100);//手册要求
	MPU_WriteReg(MPU_REG_PWR_MGMT_1,0x00);//退出睡眠
	MPU_WriteReg(MPU_REG_CONFIG,0x04);	
	MPU_WriteReg(MPU_REG_GYRO_CONFIG,0x08);//+-500°/s
	MPU_WriteReg(MPU_REG_ACCEL_CONFIG,0x00);//+-2g

}

void Heading_Init(void)
{
	yaw=0.0f;
	b_est=0.0f;
	kf_A=KF_A0;
	kf_B=KF_B0;
	kf_C=0.0f;
	phi_f=KF_PHI_NORM;
}

void Heading_Calibrate(void)
{
	MPU6050_Raw_t raw;
	int32_t sum=0;
	uint16_t i;

	for (i=0;i<50;i++)
	{
		MPU6050_ReadRaw(&raw);
		sum=sum+raw.gyro_z;
		Delay_ms(20);
	}
	bias=(float)sum/50.0f;
	b_est=-bias/65.536f; //零偏状态换成 deg/s，减号同 yaw 的符号约定
	kf_A=KF_A0;
	kf_B=KF_B0;
	kf_C=0.0f;
}

void Heading_Update(void)
{
	MPU6050_Raw_t raw;

	float vl,vr;
	float dt,u;
	float d_w,d_g,x,diff;
	float omega_w,omega_g,phi_var;
	float Ap,S,k,h_gain,cov_ba;
	uint32_t now;

	vl=(Speed_GetWheelSpeed(WHEEL_LF)+Speed_GetWheelSpeed(WHEEL_LB))*0.5f;
	vr=(Speed_GetWheelSpeed(WHEEL_RF)+Speed_GetWheelSpeed(WHEEL_RR))*0.5f;

	now=Tick_GetMs();
	dt=(now-s_t_last)*0.001f;
	s_t_last=now;
	if (dt>0.1f) {dt=0.1f;} //超过100ms只按100ms算：宁可多积一点，也不要凭空丢掉转角

	MPU6050_ReadRaw(&raw);
	u=-((float)raw.gyro_z)/65.536f; //陀螺角速度(deg/s)，减号=逆时针为正，与 yaw 同向

	/* 这一拍两个增量：轮速说转了多少度、陀螺说转了多少度。全系统只有 yaw 一条角度 */
	omega_w=((vr-vl)/TRACK_mm)*57.2958f;
	omega_g=u-b_est;
	d_w=omega_w*dt;
	d_g=omega_g*dt;

	/* ---------- 1. 预测：先把这一拍的陀螺增量走掉 ---------- */
	yaw=yaw+d_g;
	Ap=kf_A+dt*dt*kf_B-2.0f*dt*kf_C+KF_W;
	cov_ba=kf_C-dt*kf_B;

	/* ---------- 2. Φ 按打滑情况自己设置（变脏立刻承认，恢复按一阶过程） ---------- */
	diff=omega_w-omega_g;
	if (Heading_TurnState()==TURN_RUN)
	{
		phi_var=KF_PHI_TURN; //原地转向：轮子必然打滑，只信陀螺
	}
	else if (vl>-KF_WHEEL_MOVE&&vl<KF_WHEEL_MOVE&&vr>-KF_WHEEL_MOVE&&vr<KF_WHEEL_MOVE)
	{
		phi_var=KF_PHI_TURN; //轮子没在滚（滑行/停车）：里程计没有读数，不给观测
	}
	else if (diff>KF_SLIP_DEG||diff<-KF_SLIP_DEG)
	{
		phi_var=KF_PHI_SLIP;
	}
	else
	{
		phi_var=KF_PHI_NORM; //直行：轮速可信，拿它修漂
	}

	if (phi_var>=phi_f)
	{
		phi_f=phi_var; //这一拍轮速变脏，立刻收紧权重
	}
	else
	{
		phi_f=phi_f+(phi_var-phi_f)*(dt/KF_PHI_TAU); //可信度恢复是渐进的，不是一个开关
	}
	phi_var=phi_f;

	/* ---------- 3. 更新：观测残差就是两个增量之差 ---------- */
	if (phi_var<KF_TURN_LIMIT)
	{
		x=d_w-d_g;
		S=phi_var+Ap;

		k=Ap/S;
		h_gain=cov_ba/S;

		yaw=yaw+k*x;
		b_est=b_est+h_gain*x;

		kf_A=(phi_var*Ap)/S;
		kf_B=kf_B-(cov_ba*cov_ba)/S;
		kf_C=(phi_var/S)*cov_ba;
	}
	else
	{
		kf_A=Ap; //不可信段只做预测
		kf_C=cov_ba;
	}

	if (kf_B<0.0001f) {kf_B=0.0001f;}
	if (b_est>5.0f) {b_est=5.0f;}
	if (b_est<-5.0f) {b_est=-5.0f;}
}
float Heading_GetYaw(void)
{
	return yaw;
}

void Heading_TurnStart(float deg,int16_t mmps)
{
	/* 符号实测约定（2026-11-25 上电 90 度验证）：本车 deg>0 物理上是顺时针（右转），
	   deg<0 是逆时针（左转）。原因是差速方向与陀螺安装朝向配成了一对，能收敛但不等于直觉。
	   换模块朝向或换电机线后此约定会翻转，必须重测。 */
	s_turn_goal=yaw+deg; //把"要转多少度"落成这一次的瞬时目标
	s_turn_mmps=mmps;
	if (deg>0) {s_turn_dir0=1;} else {s_turn_dir0=-1;}
	s_turn_dir=s_turn_dir0;
	s_turn_t0=Tick_GetMs();
	Speed_SetTurn(s_turn_dir,s_turn_mmps);
	Speed_Enable(1);
	s_turn_state=TURN_RUN;
}

/* 每 10ms 调一次；不阻塞，不用自己测速，轮速和姿态由 20ms 任务喂好 */
uint8_t Heading_TurnTask(void)
{
	float err;

	if (s_turn_state!=TURN_RUN) {return s_turn_state;}

	if (Tick_GetMs()-s_turn_t0>=MAXT)
	{
		Speed_Stop();
		s_turn_state=TURN_FAIL;
		return s_turn_state;
	}

	err=s_turn_goal-yaw;
	if (err>-TOLERANT&&err<TOLERANT)
	{
		Speed_Stop();
		s_turn_state=TURN_OK;
		return s_turn_state;
	}

	if (err>0) {s_turn_dir=1;} else {s_turn_dir=-1;} //每一拍重算方向：冲过头就反向补
	if (s_turn_dir!=s_turn_dir0)
	{
		Speed_SetTurn(s_turn_dir,SPEED_TARGET_MIN); //反向回补只用最低速，防来回摆
	}
	else if (err>-BUFFER&&err<BUFFER)
	{
		Speed_SetTurn(s_turn_dir,s_turn_mmps/3);
	}
	else
	{
		Speed_SetTurn(s_turn_dir,s_turn_mmps);
	}
	return s_turn_state;
}

void Heading_TurnAbort(void)
{
	if (s_turn_state==TURN_RUN)
	{
		Speed_Stop();
		Speed_Enable(0);
	}
	s_turn_state=TURN_IDLE;
}

uint8_t Heading_TurnState(void)
{
	return s_turn_state;
}
