#include "uart1_ttl.h"

extern volatile uint32_t g_sys_tick;

#define UART1_RX_BUF_SIZE 32u
#define UART1_TX_BUF_SIZE 1792u
static volatile u8 uart1_rx_buf[UART1_RX_BUF_SIZE];
static volatile u8 uart1_rx_head = 0;
static volatile u8 uart1_rx_tail = 0;
static u8 uart1_tx_buf[UART1_TX_BUF_SIZE];
static volatile u16 uart1_tx_len = 0u;
static volatile u16 uart1_tx_pos = 0u;
static volatile u8 uart1_tx_busy = 0u;

static u8 UART1_AppendChar(u16 *pos, char ch)
{
    if (*pos >= UART1_TX_BUF_SIZE) return 0u;
    uart1_tx_buf[(*pos)++] = (u8)ch;
    return 1u;
}

static u8 UART1_AppendText(u16 *pos, const char *text)
{
    while (*text) {
        if (!UART1_AppendChar(pos, *text++)) return 0u;
    }
    return 1u;
}

static u8 UART1_AppendFloat3(u16 *pos, float value)
{
    char digits[12];
    u8 digit_count = 0u;
    u32 scaled;
    u32 integer;
    u32 fraction;

    if (value != value) return UART1_AppendText(pos, "nan");
    if (value < 0.0f) {
        if (!UART1_AppendChar(pos, '-')) return 0u;
        value = -value;
    }
    if (value > 4000000.0f) value = 4000000.0f;
    scaled = (u32)(value * 1000.0f + 0.5f);
    integer = scaled / 1000u;
    fraction = scaled % 1000u;

    do {
        digits[digit_count++] = (char)('0' + integer % 10u);
        integer /= 10u;
    } while (integer != 0u && digit_count < sizeof(digits));
    while (digit_count > 0u) {
        if (!UART1_AppendChar(pos, digits[--digit_count])) return 0u;
    }
    if (!UART1_AppendChar(pos, '.')) return 0u;
    if (!UART1_AppendChar(pos, (char)('0' + fraction / 100u))) return 0u;
    if (!UART1_AppendChar(pos, (char)('0' + (fraction / 10u) % 10u))) return 0u;
    return UART1_AppendChar(pos, (char)('0' + fraction % 10u));
}

static u8 UART1_PopByte(u8 *byte)
{
    u8 tail = uart1_rx_tail;
    if (tail == uart1_rx_head) return 0;
    *byte = uart1_rx_buf[tail];
    uart1_rx_tail = (u8)((tail + 1u) % UART1_RX_BUF_SIZE);
    return 1;
}

void UART1_TTL_Init(uint32_t baudrate)
{
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef usart;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1, ENABLE);
    gpio.GPIO_Pin = GPIO_Pin_9;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);
    gpio.GPIO_Pin = GPIO_Pin_10;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &gpio);

    usart.USART_BaudRate = baudrate;
    usart.USART_WordLength = USART_WordLength_8b;
    usart.USART_StopBits = USART_StopBits_1;
    usart.USART_Parity = USART_Parity_No;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &usart);
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART1, ENABLE);
}

void UART1_RX_IRQ(void)
{
    u32 sr = USART1->SR;
    if (sr & (USART_SR_RXNE | USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE)) {
        u8 data = (u8)(USART1->DR & 0xFF);
        if (sr & USART_SR_RXNE) {
            u8 next = (u8)((uart1_rx_head + 1u) % UART1_RX_BUF_SIZE);
            if (next != uart1_rx_tail) {
                uart1_rx_buf[uart1_rx_head] = data;
                uart1_rx_head = next;
            }
        }
    }
    if ((USART1->CR1 & USART_CR1_TXEIE) && (USART1->SR & USART_SR_TXE)) {
        if (uart1_tx_pos < uart1_tx_len) {
            USART1->DR = uart1_tx_buf[uart1_tx_pos++];
        } else {
            USART1->CR1 &= (u16)~USART_CR1_TXEIE;
            uart1_tx_busy = 0u;
        }
    }
}

void UART1_SendByte(u8 data)
{
    USART_SendData(USART1, data);
    while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
}

void UART1_SendBytes(u8 *buf, u16 len)
{
    u16 i;
    for (i = 0; i < len; i++) UART1_SendByte(buf[i]);
}

void UART1_SendString(const char *str)
{
    while (*str) UART1_SendByte((u8)*str++);
}

u8 UART1_SendVOFAFrame(const float *values, u8 count)
{
    u8 i;
    u8 b;
    u16 len;

    if (values == 0 || count == 0u || uart1_tx_busy) return 0u;
    len = (u16)count * 4u + 4u;
    if (len > UART1_TX_BUF_SIZE) return 0u;
    for (i = 0u; i < count; i++) {
        const u8 *p = (const u8 *)&values[i];
        for (b = 0u; b < 4u; b++) {
            uart1_tx_buf[(u16)i * 4u + b] = p[b];
        }
    }
    uart1_tx_buf[len - 4u] = 0x00u;
    uart1_tx_buf[len - 3u] = 0x00u;
    uart1_tx_buf[len - 2u] = 0x80u;
    uart1_tx_buf[len - 1u] = 0x7Fu;
    uart1_tx_pos = 0u;
    uart1_tx_len = len;
    uart1_tx_busy = 1u;
    USART1->CR1 |= USART_CR1_TXEIE;
    return 1u;
}

u8 UART1_SendAsciiFrame(const char *const *names,
                        const float *values, u8 count)
{
    u8 i;
    u16 len = 0u;

    if (names == 0 || values == 0 || count == 0u || uart1_tx_busy)
        return 0u;
    for (i = 0u; i < count; i++) {
        if (i != 0u && !UART1_AppendChar(&len, ',')) return 0u;
        if (!UART1_AppendText(&len, names[i])) return 0u;
        if (!UART1_AppendChar(&len, '=')) return 0u;
        if (!UART1_AppendFloat3(&len, values[i])) return 0u;
    }
    if (!UART1_AppendChar(&len, '\r')) return 0u;
    if (!UART1_AppendChar(&len, '\n')) return 0u;

    uart1_tx_pos = 0u;
    uart1_tx_len = len;
    uart1_tx_busy = 1u;
    USART1->CR1 |= USART_CR1_TXEIE;
    return 1u;
}

u8 UART1_RecvByte(u8 *byte, u32 timeout_ms)
{
    u32 start = g_sys_tick;
    while ((u32)(g_sys_tick - start) < timeout_ms) {
        if (UART1_PopByte(byte)) return 1;
    }
    return 0;
}

u16 UART1_RecvBytes(u8 *buf, u16 max_len, u32 timeout_ms)
{
    u16 count = 0;
    u32 start = g_sys_tick;
    u32 last_byte = start;

    while ((u32)(g_sys_tick - start) < timeout_ms && count < max_len) {
        if (UART1_PopByte(&buf[count])) {
            count++;
            last_byte = g_sys_tick;
        } else if (count > 0 && (u32)(g_sys_tick - last_byte) >= 2u) {
            break;
        }
    }
    return count;
}

void UART1_Flush(void)
{
    volatile u32 dummy;
    uart1_rx_tail = uart1_rx_head;
    dummy = USART1->SR;
    dummy = USART1->DR;
    (void)dummy;
}
