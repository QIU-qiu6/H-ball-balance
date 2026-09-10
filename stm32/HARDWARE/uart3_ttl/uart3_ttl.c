#include "uart3_ttl.h"

extern volatile uint32_t g_sys_tick;

#define UART3_RX_BUF_SIZE 32u
static volatile u8 uart3_rx_buf[UART3_RX_BUF_SIZE];
static volatile u8 uart3_rx_head = 0;
static volatile u8 uart3_rx_tail = 0;

static u8 UART3_PopByte(u8 *byte)
{
    u8 tail = uart3_rx_tail;
    if (tail == uart3_rx_head) return 0;
    *byte = uart3_rx_buf[tail];
    uart3_rx_tail = (u8)((tail + 1u) % UART3_RX_BUF_SIZE);
    return 1;
}

void UART3_TTL_Init(uint32_t baudrate)
{
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef usart;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    gpio.GPIO_Pin = GPIO_Pin_10;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpio);
    gpio.GPIO_Pin = GPIO_Pin_11;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &gpio);

    usart.USART_BaudRate = baudrate;
    usart.USART_WordLength = USART_WordLength_8b;
    usart.USART_StopBits = USART_StopBits_1;
    usart.USART_Parity = USART_Parity_No;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART3, &usart);
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART3, ENABLE);
}

void UART3_RX_IRQ(void)
{
    u32 sr = USART3->SR;
    if (sr & (USART_SR_RXNE | USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE)) {
        u8 data = (u8)(USART3->DR & 0xFF);
        if (sr & USART_SR_RXNE) {
            u8 next = (u8)((uart3_rx_head + 1u) % UART3_RX_BUF_SIZE);
            if (next != uart3_rx_tail) {
                uart3_rx_buf[uart3_rx_head] = data;
                uart3_rx_head = next;
            }
        }
    }
}

void UART3_SendByte(u8 data)
{
    USART_SendData(USART3, data);
    while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
}

void UART3_SendBytes(u8 *buf, u16 len)
{
    u16 i;
    for (i = 0; i < len; i++) UART3_SendByte(buf[i]);
}

void UART3_SendString(const char *str)
{
    while (*str) UART3_SendByte((u8)*str++);
}

u8 UART3_RecvByte(u8 *byte, u32 timeout_ms)
{
    u32 start = g_sys_tick;
    while ((u32)(g_sys_tick - start) < timeout_ms) {
        if (UART3_PopByte(byte)) return 1;
    }
    return 0;
}

u16 UART3_RecvBytes(u8 *buf, u16 max_len, u32 timeout_ms)
{
    u16 count = 0;
    u32 start = g_sys_tick;
    u32 last_byte = start;

    while ((u32)(g_sys_tick - start) < timeout_ms && count < max_len) {
        if (UART3_PopByte(&buf[count])) {
            count++;
            last_byte = g_sys_tick;
        } else if (count > 0 && (u32)(g_sys_tick - last_byte) >= 2u) {
            break;
        }
    }
    return count;
}

void UART3_Flush(void)
{
    volatile u32 dummy;
    uart3_rx_tail = uart3_rx_head;
    dummy = USART3->SR;
    dummy = USART3->DR;
    (void)dummy;
}
