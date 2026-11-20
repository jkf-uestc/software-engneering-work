#ifndef __BlueTooth_H
#define __BlueTooth_H

#include <stdio.h>
#include <stdint.h>		/* uint8_t/uint16_t/uint32_t：头文件自带，不能依赖调用方的包含顺序 */

/*
总共设置9个按键，左转，右转，高速前进，中速前进，低速前进，高速后退，中速后退，低速后退，停止
每个按键对应一个数据包，数据包格式为：FF 01 02 03 04 FE
*/

extern uint8_t BlueTooth_TxPacket[];
extern volatile uint8_t BlueTooth_RxPacket[];
extern volatile uint8_t BlueTooth_RxFlag;
#define ForwardHigh 32
#define ForwardMedium 31
#define ForwardLow 30
#define BackHigh 52
#define BackMedium 51
#define BackLow 50
#define TurnLeft 1
#define TurnRight 2
#define Stop 4
#define HighSpeed 3599
#define MediumSpeed 1800
#define LowSpeed 900
void BlueTooth_Init(void);            //初始化蓝牙
void BlueTooth_SendByte(uint8_t Byte);              //发送一个字节
void BlueTooth_SendArray(uint8_t *Array, uint16_t Length);             //发送一个数组
void BlueTooth_SendString(char *String);              //发送一个字符串
void BlueTooth_SendNumber(uint32_t Number, uint8_t Length);          //发送一个数字
void BlueTooth_Printf(char *format, ...);                //自己封装的printf函数
uint8_t BlueTooth_GetFirstRxData(void);                //获得数据包第一个字节
uint8_t BlueTooth_CmpRxPacket(const volatile uint8_t *RxPacket, const uint8_t *ReferencePacket);          //比较接收数据包和参考数据包
void BlueTooth_SendPacket(void);            //发送数据包
uint8_t BlueTooth_GetRxFlag(void);            //获取串口接收数据包标志位
int BlueTooth_action(void);                //获取蓝牙操作的动作，返回值为动作的编号

#endif
