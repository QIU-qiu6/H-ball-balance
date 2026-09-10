/**
 * 中断服务函数
 * USART2_IRQHandler — K230 逐字节接收 (RXNE 中断)
 * SysTick_Handler — 在 main.c 中定义
 */
#include "stm32f10x.h"
#include "k230_proto.h"
#include "uart1_ttl.h"
#include "uart3_ttl.h"

void NMI_Handler(void)           {}
void HardFault_Handler(void)     { while(1); }
void MemManage_Handler(void)     { while(1); }
void BusFault_Handler(void)      { while(1); }
void UsageFault_Handler(void)    { while(1); }
void SVC_Handler(void)           {}
void DebugMon_Handler(void)      {}
void PendSV_Handler(void)        {}

/* USART2 — K230 接收 */
void USART2_IRQHandler(void)
{
    K230_RX_IRQ();
}

void USART1_IRQHandler(void)
{
    UART1_RX_IRQ();
}

/* USART3 — 保留 (目前用轮询, 与 USART1 一致) */
void USART3_IRQHandler(void)
{
    UART3_RX_IRQ();
}
