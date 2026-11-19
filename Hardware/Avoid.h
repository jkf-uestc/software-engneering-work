#ifndef __AVOID_H
#define __AVOID_H
#include "stm32f10x.h"

/**
 * @brief 避障触发距离阈值，单位毫米
 * 当超声波检测到前方距离 <=200mm(20cm)，触发小车刹车停止
 */
#define AVOID_OBSTACLE_THRESHOLD_MM  200U

/**
 * @brief 避障模块状态枚举
 * AVOID_ST_FREE：自由状态，无障碍物，可以响应前进、转弯等运动指令
 * AVOID_ST_BLOCKED：检测到障碍物，小车已经刹车，不会自动恢复，等待外部指令解除阻塞
 */
typedef enum
{
    AVOID_ST_FREE = 0,      // 正常，允许小车执行运动
    AVOID_ST_BLOCKED        // 前方检测障碍物，小车停止，等待后续指令
}Avoid_State_t;


void Avoid_Task(void);                  //【主循环调用】避障检测任务函数，放在while(1)轮询
Avoid_State_t Avoid_GetState(void);     // 获取当前避障模块状态
uint8_t Avoid_IsBlocked(void);          // 判断是否被障碍物阻挡，返回1代表阻挡，0代表正常
void Avoid_ClearBlock(void);            // 清除阻塞标记，外部收到继续指令时调用，恢复小车运动权限

#endif
