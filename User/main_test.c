#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "motor.h"


int main(void)
{
	OLED_Init(); 
	
	OLED_ShowChar(1, 1, 'A');
	OLED_ShowString(1,3,"HElloWorld!");
	OLED_ShowNum(2,1,12345,5);
	OLED_ShowSignedNum(2,7,-66,5);
	OLED_ShowHexNum(3,1,0xAA55,4);
	OLED_ShowBinNum(4,1,0xAA55,16);
	
    Motor_Init();
	//正向前进代码

    // M1
    M1_SetDir(MOTOR_FORWARD);
    M1_SetPWM(1800);

    // M2
    M2_SetDir(MOTOR_FORWARD);
    M2_SetPWM(1800);

    // M3
    M3_SetDir(MOTOR_BACKWARD);
    M3_SetPWM(1800);

    // M4
    M4_SetDir(MOTOR_BACKWARD);
    M4_SetPWM(1800);

    //调用转向函数 用的时候删注释就行
	// 模式1：一直直行
    //Car_GoStraight(1800);

    // 模式2：原地左转自旋
	//Car_TurnLeft(1200);

    // 模式3：原地右转自旋
    // Car_TurnRight(1200);

    // 模式4：全部刹车停止
    // Car_Stop();


	while(1)
	{

	}
	
}

