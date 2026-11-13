#ifndef __BlueTooth_H
#define __BlueTooth_H

#include <stdio.h>

extern uint8_t BlueTooth_TxPacket[];
extern uint8_t BlueTooth_RxPacket[];
extern uint8_t BlueTooth_RxFlag;
#define Forward 0
#define Stop 4
void BlueTooth_Init(void);
void BlueTooth_SendByte(uint8_t Byte);
void BlueTooth_SendArray(uint8_t *Array, uint16_t Length);
void BlueTooth_SendString(char *String);
void BlueTooth_SendNumber(uint32_t Number, uint8_t Length);
void BlueTooth_Printf(char *format, ...);
uint8_t BlueTooth_GetFirstRxData(void);
uint8_t BlueTooth_CmpRxPacket(uint8_t *BlueTooth_RxPacket, uint8_t *ReferencePacket);
uint8_t BlueTooth_CmpRxPacket(uint8_t *BlueTooth_RxPacket, uint8_t *ReferencePacket);
void BlueTooth_SendPacket(void);
uint8_t BlueTooth_GetRxFlag(void);
int BlueTooth_action(void);

#endif
