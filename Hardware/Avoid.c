#include "Avoid.h"
#include "Sonar.h"   //引入超声波驱动接口
#include "motor.h"   //引入电机驱动，使用Car_Stop()刹车函数

/**
 * @brief s_avoid_state 避障模块内部状态变量
 * 静态全局变量，保存避障状态；初始为自由状态，小车可以正常行驶
 * volatile不需要，只在主循环读写，不在中断修改
 */
static Avoid_State_t s_avoid_state = AVOID_ST_FREE;


/**
 * @brief Avoid_Task 避障任务函数
 * 注意：只能在main函数while(1)主循环调用，禁止放到任何中断内部！
 * 依赖：Sonar_Task()必须优先于本函数调用，保证超声波距离数据已经刷新完成
 *
 * 逻辑流程：
 * 1、读取超声波返回距离
 * 2、判断超声波测量结果是否有效（Sonar_IsValid=1代表不是超时/噪声）
 * 3、有效前提下，如果距离小于等于阈值200mm，触发刹车
 * 4、进入阻塞状态后不会自动退出；就算障碍物拿走，小车也不会自动跑，等待外部指令
 * 5、超声波无效时，不做刹车动作，维持原有状态，防止噪声误刹车
 */
void Avoid_Task(void)
{
    uint16_t dist_mm;

    //读取超声波最近一次测量得到的距离（单位mm，已经做三取中滤波）
    dist_mm = Sonar_GetMm();

    //只有超声波测量有效才做障碍物判断；超时、噪声的时候直接跳过判断
    if(Sonar_IsValid() == 1U)
    {
        // 判断距离是否小于等于20cm障碍物阈值
        if(dist_mm <= AVOID_OBSTACLE_THRESHOLD_MM)
        {
            // 如果当前还没有进入阻塞状态，则执行刹车，切换状态
            if(s_avoid_state != AVOID_ST_BLOCKED)
            {
                Car_Stop();               //调用电机驱动：四个电机全部刹车停止
                s_avoid_state = AVOID_ST_BLOCKED; //标记为障碍物阻挡状态
            }
        }
    }
    /*
    重点：这里没有写自动解除阻塞的代码！
    需求：检测障碍物停下之后，等待后续指令，不自动继续运行
    哪怕超声波后续读到距离变大（障碍物被拿走），s_avoid_state仍然保持BLOCKED
    必须外部调用 Avoid_ClearBlock() 才会切回AVOID_ST_FREE
    */
}

/**
 * @brief Avoid_GetState 获取避障模块状态
 * @retval 返回枚举AVOID_ST_FREE / AVOID_ST_BLOCKED
 */
Avoid_State_t Avoid_GetState(void)
{
    return s_avoid_state;
}

/**
 * @brief Avoid_IsBlocked 判断小车是否被障碍物阻挡
 * @retval 1：被障碍物挡住，禁止小车运动；0：无障碍物，可以执行运动指令
 */
uint8_t Avoid_IsBlocked(void)
{
    //三目运算符，状态等于阻塞就返回1，否则返回0
    return (s_avoid_state == AVOID_ST_BLOCKED) ? 1U : 0U;
}

/**
 * @brief Avoid_ClearBlock 清除障碍物阻塞标记
 * 使用场景：蓝牙/按键收到“继续行驶”指令的时候调用
 * 调用之后，Avoid_IsBlocked返回0，上层才可以调用Car_GoStraight等运动函数
 * 注意：本函数**不会让小车自动前进**，仅仅解除权限；前进动作需要上层自己调用电机函数
 */
void Avoid_ClearBlock(void)
{
    s_avoid_state = AVOID_ST_FREE;
}
