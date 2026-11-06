#include "Tick.h"

#define CYCLES_PER_MS   72000u    /* 72MHz / 1000 */

static uint32_t s_last_cy = 0;    /* 上次读到的 DWT 计数 */
static uint32_t s_ms      = 0;    /* 累计毫秒 */
static uint32_t s_frac    = 0;    /* 不足 1ms 的余数（周期数） */

void Tick_Init(void)
{
    /* TRCENA 是调试外设总开关，不开它 CYCCNT 永远是 0 */
    TICK_DEMCR |= TICK_DEMCR_TRCENA;
    TICK_DWT_CYCCNT = 0;
    TICK_DWT_CTRL |= TICK_DWT_CTRL_CYCCNTENA;

    s_last_cy = TICK_DWT_CYCCNT;
    s_ms      = 0;
    s_frac    = 0;
}

uint32_t Tick_GetMs(void)
{
    uint32_t now = TICK_DWT_CYCCNT;
    uint32_t d   = now - s_last_cy;   /* 无符号相减，自动处理 32 位回绕 */

    s_last_cy = now;

    /* 把周期数折算成毫秒，余数留着下次累计，避免每次都截断导致偏慢 */
    s_frac += d;
    while (s_frac >= CYCLES_PER_MS)
    {
        s_frac -= CYCLES_PER_MS;
        s_ms++;
    }
    return s_ms;
}

uint32_t Tick_GetCycles(void)
{
    return TICK_DWT_CYCCNT;
}

void Tick_DelayUs(uint32_t us)
{
    uint32_t start = TICK_DWT_CYCCNT;
    uint32_t need  = us * 72u;
    while ((TICK_DWT_CYCCNT - start) < need)
    {
        /* 忙等 */
    }
}

uint8_t Tick_Every(uint32_t *last_ms, uint32_t period_ms)
{
    uint32_t now = Tick_GetMs();

    /* 无符号相减，DWT 32 位回绕也照样对（59.6 秒一轮） */
    if ((uint32_t)(now - *last_ms) < period_ms)
    {
        return 0;
    }

    *last_ms = now;
    return 1;
}
