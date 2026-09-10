#include "estop.h"
#include "stm32f10x.h"

#define ESTOP_PIN          GPIO_Pin_0
#define ESTOP_DEBOUNCE_MS  20u

static volatile u8 g_estop_latched = 0;
static volatile u8 g_low_ms = 0;

static u8 Estop_RawPressed(void)
{
    return (GPIO_ReadInputDataBit(GPIOA, ESTOP_PIN) == Bit_RESET) ? 1 : 0;
}

void Estop_Init(void)
{
    GPIO_InitTypeDef gpio;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    gpio.GPIO_Pin = ESTOP_PIN;
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    gpio.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOA, &gpio);

    g_low_ms = 0;
    /* A held button at boot must prevent motor enable immediately. */
    g_estop_latched = Estop_RawPressed();
}

void Estop_Tick1ms(void)
{
    if (g_estop_latched) return;

    if (Estop_RawPressed()) {
        if (g_low_ms < ESTOP_DEBOUNCE_MS) g_low_ms++;
        if (g_low_ms >= ESTOP_DEBOUNCE_MS) g_estop_latched = 1;
    } else {
        g_low_ms = 0;
    }
}

u8 Estop_IsLatched(void)
{
    return g_estop_latched;
}
