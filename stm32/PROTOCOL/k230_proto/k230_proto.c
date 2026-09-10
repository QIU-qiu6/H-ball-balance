/**
 * K230 通信协议实现
 *
 * 使用 USART2 逐字节中断接收, 检测 '\n' 作为帧结束.
 * 比 DMA+IDLE 方案简单可靠, 115200bps 下中断开销 <0.1% CPU.
 */
#include "k230_proto.h"
#include "stm32f10x.h"
#include <string.h>

/* ---- 行缓冲 ---- */
static u8  g_line_buf[K230_FRAME_SIZE];
static u8  g_line_idx = 0;
static u8  g_bin_buf[K230_BINARY_FRAME_SIZE];
static u8  g_bin_idx = 0;
static u8  g_bin_active = 0;

static volatile u8  g_new_frame = 0;
static K230_Frame    g_frame;
static volatile u16 g_silence_ms = K230_TIMEOUT_MS;
static volatile u8  g_seen_valid = 0;
static volatile u8  g_no_target_frames = 0;

/* ================================================================
 * 初始化
 * ================================================================ */
void K230_Init(uint32_t baudrate)
{
    GPIO_InitTypeDef  G;
    USART_InitTypeDef U;

    /* ---- 时钟 ---- */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    /* PA2 transmission is deliberately disabled.  PA3 remains USART2_RX
     * because the zero-point controller still needs MaixCAM coordinates. */
    G.GPIO_Pin   = GPIO_Pin_2;
    G.GPIO_Mode  = GPIO_Mode_AIN;
    G.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOA, &G);

    G.GPIO_Pin  = GPIO_Pin_3;
    G.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &G);

    /* ---- USART2 ---- */
    U.USART_BaudRate            = baudrate;
    U.USART_WordLength          = USART_WordLength_8b;
    U.USART_StopBits            = USART_StopBits_1;
    U.USART_Parity              = USART_Parity_No;
    U.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    U.USART_Mode                = USART_Mode_Rx;
    USART_Init(USART2, &U);

    /* 使能 RXNE 中断 (每收到1字节触发) */
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART2, ENABLE);

    /* 初始化帧 */
    g_line_idx       = 0;
    g_bin_idx        = 0;
    g_bin_active     = 0;
    g_frame.has_target = 0;
    g_frame.fresh_target = 0;
    g_frame.cx = g_frame.cy = 0;
    g_frame.w = g_frame.h = 0;
    g_frame.interval_ms = K230_TIMEOUT_MS;
    g_frame.velocity_milli_px_s = 0;
    g_frame.velocity_valid = 0;
    g_new_frame = 0;
    g_silence_ms = K230_TIMEOUT_MS;
    g_seen_valid = 0;
    g_no_target_frames = 0;
}

/* ================================================================
 * K230_SendString — 阻塞发送
 * ================================================================ */
void K230_SendString(const char *str)
{
    /* RX-only build: never drive PA2. */
    (void)str;
}

void K230_SendVOFAFrame(const float *values, u8 count)
{
    /* RX-only build: telemetry transmission is disabled. */
    (void)values;
    (void)count;
}

/* ================================================================
 * 帧解析: "X:320,Y:240,W:85,H:60" → K230_Frame
 * ================================================================ */
static u8 ParseU16(u8 **cursor, u8 *end, u16 *value)
{
    u32 v = 0;
    u8 *p = *cursor;
    if (p >= end || *p < '0' || *p > '9') return 0;
    while (p < end && *p >= '0' && *p <= '9') {
        v = v * 10u + (u32)(*p - '0');
        if (v > 65535u) return 0;
        p++;
    }
    *cursor = p;
    *value = (u16)v;
    return 1;
}

static u8 Match(u8 **cursor, u8 *end, const char *token)
{
    u8 *p = *cursor;
    while (*token) {
        if (p >= end || *p++ != (u8)*token++) return 0;
    }
    *cursor = p;
    return 1;
}

static u8 ParseLine(u8 *line, u8 len)
{
    if (len >= 8 && line[0] == 'X' && line[1] == ':') {
        u16 cx=0, cy=0, w=0, h=0;
        u8 *p = line;
        u8 *end = line + len;
        if (!Match(&p, end, "X:") || !ParseU16(&p, end, &cx) ||
            !Match(&p, end, ",Y:") || !ParseU16(&p, end, &cy) ||
            !Match(&p, end, ",W:") || !ParseU16(&p, end, &w) ||
            !Match(&p, end, ",H:") || !ParseU16(&p, end, &h) || p != end) return 0;
        if (cx >= K230_IMAGE_WIDTH || cy >= K230_IMAGE_HEIGHT ||
            w == 0 || h == 0 || w > K230_IMAGE_WIDTH || h > K230_IMAGE_HEIGHT) {
            return 0;
        }
        g_frame.cx = cx;
        g_frame.cy = cy;
        g_frame.w  = w;
        g_frame.h  = h;
        g_frame.velocity_milli_px_s = 0;
        g_frame.velocity_valid = 0;
        g_frame.has_target = 1;
        g_frame.fresh_target = 1;
        g_no_target_frames = 0;
        return 1;
    } else if (len == 9 && memcmp(line, "NO_TARGET", 9) == 0) {
        g_frame.fresh_target = 0;
        if (g_no_target_frames < K230_LOST_CONFIRM_FRAMES) {
            g_no_target_frames++;
        }
        if (g_no_target_frames >= K230_LOST_CONFIRM_FRAMES) {
            g_frame.has_target = 0;
        }
        return 1;
    }
    return 0;
}

static s32 ParseBinaryS32(const u8 *p)
{
    u32 value = ((u32)p[0] << 24) | ((u32)p[1] << 16) |
                ((u32)p[2] << 8) | (u32)p[3];
    return (s32)value;
}

static u8 ParseBinaryFrame(void)
{
    u16 cx;

    if (g_bin_buf[0] != 0xFEu || g_bin_buf[7] != 0xFDu) return 0;
    cx = (u16)(((u16)g_bin_buf[1] << 8) | g_bin_buf[2]);
    if (cx == 0xFFFFu) {
        g_frame.fresh_target = 0;
        g_frame.velocity_valid = 0;
        if (g_no_target_frames < K230_LOST_CONFIRM_FRAMES) {
            g_no_target_frames++;
        }
        if (g_no_target_frames >= K230_LOST_CONFIRM_FRAMES) {
            g_frame.has_target = 0;
        }
        return 1;
    }
    if (cx >= K230_IMAGE_WIDTH) return 0;

    g_frame.cx = cx;
    g_frame.cy = 340u;
    /* The compact binary protocol carries X and Vx only.  The fixed positive
     * box dimensions satisfy the existing frame validity contract. */
    g_frame.w = 20u;
    g_frame.h = 20u;
    g_frame.velocity_milli_px_s = ParseBinaryS32(&g_bin_buf[3]);
    g_frame.velocity_valid = 1u;
    g_frame.has_target = 1u;
    g_frame.fresh_target = 1u;
    g_no_target_frames = 0u;
    return 1;
}

/* ================================================================
 * RXNE 中断处理 — 在 USART2_IRQHandler 中调用
 * ================================================================ */
void K230_RX_IRQ(void)
{
    u8 ch;
    u32 sr = USART2->SR;

    if (sr & USART_SR_RXNE) {
        ch = (u8)(USART2->DR & 0xFF);

        if (!g_bin_active && ch == 0xFEu) {
            g_bin_active = 1u;
            g_bin_idx = 0u;
            g_bin_buf[g_bin_idx++] = ch;
            return;
        }
        if (g_bin_active) {
            if (g_bin_idx < K230_BINARY_FRAME_SIZE) {
                g_bin_buf[g_bin_idx++] = ch;
            } else {
                g_bin_active = 0u;
                g_bin_idx = 0u;
            }
            if (g_bin_idx >= K230_BINARY_FRAME_SIZE) {
                if (ParseBinaryFrame()) {
                    g_frame.interval_ms = g_silence_ms;
                    if (g_frame.interval_ms == 0u) {
                        g_frame.interval_ms = 1u;
                    }
                    g_silence_ms = 0u;
                    g_seen_valid = 1u;
                    g_new_frame = 1u;
                }
                g_bin_active = 0u;
                g_bin_idx = 0u;
            }
            return;
        }

        if (ch == '\n') {
            /* 完整一行 */
            if (g_line_idx > 0) {
                g_line_buf[g_line_idx] = '\0';
                if (ParseLine(g_line_buf, g_line_idx)) {
                    g_frame.interval_ms = g_silence_ms;
                    if (g_frame.interval_ms == 0u) {
                        g_frame.interval_ms = 1u;
                    }
                    g_silence_ms = 0;
                    g_seen_valid = 1;
                    g_new_frame = 1;
                }
                g_line_idx = 0;
            }
        } else if (ch != '\r' && g_line_idx < K230_FRAME_SIZE - 1) {
            g_line_buf[g_line_idx++] = ch;
        }
    }
}

/* ================================================================
 * 上层查询接口
 * ================================================================ */
u8 K230_GetNewFrame(K230_Frame *f)
{
    u8 has_frame;

    /* USART2 IRQ is the sole writer; mask it across copy+clear. */
    NVIC_DisableIRQ(USART2_IRQn);
    has_frame = g_new_frame;
    if (has_frame) {
        *f = g_frame;
        g_new_frame = 0;
    }
    NVIC_EnableIRQ(USART2_IRQn);
    return has_frame;
}

void K230_Tick1ms(void)
{
    if (g_silence_ms < 0xFFFFu) {
        g_silence_ms++;
    }
}

u8 K230_LinkAlive(void)
{
    return (g_seen_valid && g_silence_ms < K230_TIMEOUT_MS) ? 1 : 0;
}
