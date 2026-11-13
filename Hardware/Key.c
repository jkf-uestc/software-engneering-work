#include "Key.h"
#include "Tick.h"

/* ============================================================================
 * Key.c —— 单键循环档位
 *按一次升一档，到顶回零，长按强制归零。
 * ========================================================================== */

static uint8_t s_win_buf[KEY_WIN_SIZE]; // 环形窗口用于消抖
static uint8_t s_win_idx = 0;           // 当前写入位置
static uint8_t sum_high = 0u;  // 缓存当前窗口里1的总数，避免每次求和

//下标就是档位号
static const int16_t s_speed[KEY_GEAR_MAX + 1] ={0,300,500,800};

static uint8_t  s_gear      = 0;    //当前档位
static uint8_t  s_stable    = 0;    //消抖后电平，1表示按下
static uint8_t  s_long_done = 0;    /* 1 = 这次按压已按长按处理过，松手时不再加档 */
static uint32_t s_last_scan = 0;    //上次成功扫描按键的时间戳
static uint32_t s_press_ms  = 0;    //刚按下的时刻

void Key_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_InitTypeDef gpio;
    gpio.GPIO_Pin  = KEY_GPIO_PIN;
    gpio.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(KEY_GPIO_PORT, &gpio);

    s_gear      = 0;
    s_stable    = 0;
    s_long_done = 0;
    s_last_scan = 0;
    s_press_ms  = 0;

    s_win_idx = 0;
    sum_high = 0u;
    for(uint8_t i = 0; i < KEY_WIN_SIZE; i++)
    {
        s_win_buf[i] = 0u;
    }
}

void Key_Task(void)
{
    uint8_t  raw;
    uint32_t now;
    // 删掉：uint8_t sum_high = 0;

    //限制5ms
    if (!Tick_Every(&s_last_scan, KEY_SCAN_MS))
    {
        return;
    }

	raw=GPIO_ReadInputDataBit(KEY_GPIO_PORT, KEY_GPIO_PIN);
	
    //环形窗口更新
	uint8_t old_val = s_win_buf[s_win_idx];
	
	if (old_val == 1)sum_high--;
	
	if (raw == 1)sum_high++;
	
	s_win_buf[s_win_idx] = raw;
	
	s_win_idx = (s_win_idx + 1) % KEY_WIN_SIZE;

	//多数表决
    uint8_t new_state;
	
    if(sum_high >= KEY_THRESHOLD)new_state = 1;
    
    else if( (KEY_WIN_SIZE - sum_high) >= KEY_THRESHOLD )new_state = 0;

    else new_state = s_stable;
    
    if (new_state != s_stable) //注意是按键变化才进入这里，长按过程中是没有状态变化的
    {
        s_stable = new_state;
        now      = Tick_GetMs();

        if (s_stable)                       
        {
            s_press_ms = now;
            s_long_done = 0;
        }
        else //松手换挡                                
        {
            if (!s_long_done)               
            {
                Key_GearNext();
            }
            s_long_done = 0;
        }
    }


    //按住不放，到 3 秒立刻归零，不等松手——强制停止要快
    if (s_stable && !s_long_done &&(Tick_GetMs() - s_press_ms) >= KEY_LONG_PRESS_MS)
    {
        s_long_done = 1;
        Key_GearStop();                         
    }
}

uint8_t Key_GetGear(void)
{
    return s_gear;
}

int16_t Key_GetSpeed(void)
{
    return s_speed[s_gear];
}

void Key_GearNext(void)
{
    s_gear = (uint8_t)((s_gear + 1u) % (KEY_GEAR_MAX + 1u));
}

void Key_GearStop(void)
{
    s_gear = 0;
}
