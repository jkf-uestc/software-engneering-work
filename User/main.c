#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "BlueTooth.h"
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
	
	/*显示静态字符串*/
	OLED_ShowString(1, 1, "RxData:");
	
	uint8_t RxData_f;		//定义接收数据变量
	while (1)
	{
		if (BlueTooth_GetRxFlag() == 1)			//检查蓝牙接收数据的标志位
		{
			RxData_f = BlueTooth_GetFirstRxData();		//获取蓝牙接收的数据
			BlueTooth_SendByte(RxData_f);			//蓝牙将收到的数据回传回去，用于测试
			OLED_ShowHexNum(1, 8, RxData_f, 2);	//显示蓝牙接收的数据
		}
	}
}
