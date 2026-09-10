#include "delay.h"

/*
 * DWT cycle-counter delay.
 * This deliberately does not reconfigure SysTick: SysTick is the shared
 * 1 ms system timebase used by motor UART timeouts and safety watchdogs.
 */
static uint32_t g_cycles_per_us = 0;

void Delay_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    g_cycles_per_us = SystemCoreClock / 1000000u;
}

void Delay_us(uint32_t us)
{
    uint32_t start;
    uint32_t cycles;

    if (g_cycles_per_us == 0) Delay_Init();
    while (us > 0) {
        uint32_t chunk = (us > 1000000u) ? 1000000u : us;
        cycles = chunk * g_cycles_per_us;
        start = DWT->CYCCNT;
        while ((uint32_t)(DWT->CYCCNT - start) < cycles) { }
        us -= chunk;
    }
}

void Delay_ms(uint32_t ms)
{
    while (ms--) Delay_us(1000u);
}
