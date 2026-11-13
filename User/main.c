#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "BlueTooth.h"
#include "motor.h"
/*
int main(void)
{
	OLED_Init(); 
	
	OLED_ShowChar(1, 1, 'A');
	OLED_ShowString(1,3,"HElloWorld!");
	OLED_ShowNum(2,1,12345,5);
	OLED_ShowSignedNum(2,7,-66,5);
	OLED_ShowHexNum(3,1,0xAA55,4);
	OLED_ShowBinNum(4,1,0xAA55,16);

	while(1)
	{

	}
}
*/
int main(void)
{
	/*模块初始化*/
	OLED_Init();		//OLED初始化
	BlueTooth_Init();	//蓝牙初始化
	Motor_Init();       //电机初始化

	while (1)
	{
		if (BlueTooth_GetRxFlag() == 1)			//检查蓝牙接收数据的标志位，为1表示接收到数据包，然后标志位清零
		{
			int action = BlueTooth_action();    //获取蓝牙接收数据的动作
			BlueTooth_SendArray(BlueTooth_RxPacket, 4);		//蓝牙将收到的数据回传回去，用于测试
			if(action == ForwardHigh){
				Car_GoStraight(HighSpeed);       
				BlueTooth_SendString("High Speed:3600");
			}
			if(action == ForwardMedium){
				Car_GoStraight(MediumSpeed);       
				BlueTooth_SendString("Medium Speed:1800");
			}
			if(action == ForwardLow){
				Car_GoStraight(LowSpeed);       
				BlueTooth_SendString("Low Speed:900");
			}
			else if(action == TurnLeft){
				Car_TurnLeft(1800);
			}
			else if(action == TurnRight){
				Car_TurnRight(1800);
			}
			else if(action == Stop){
				Car_Stop();
			}

		}
	}
}
