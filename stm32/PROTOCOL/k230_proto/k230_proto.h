#ifndef __K230_PROTO_H__
#define __K230_PROTO_H__
#include "sys.h"

/* ================================================================
 * K230 摄像头通信协议
 *
 * 硬件: USART2 (PA2=TX, PA3=RX), RXNE逐字节中断
 * 波特率: 115200-8N1
 *
 * K230 → STM32 (文本帧, 行结束符 \n):
 *   "X:<cx>,Y:<cy>,W:<w>,H:<h>\n"  检测到色板
 *   "NO_TARGET\n"                    未检测到
 *   "K230_READY\n"                   启动就绪
 *
 * STM32 → K230:
 *   "STM32_READY\n"                  初始化完成
 *   "TRACKING\n" / "LOST\n" / "STOP\n"
 * ================================================================ */

#define K230_BAUDRATE      115200
#define K230_FRAME_SIZE    32          /* 单帧最大字节 */
#define K230_IMAGE_WIDTH   640
#define K230_IMAGE_HEIGHT  480
#define K230_LOST_CONFIRM_FRAMES 3u
#define K230_TIMEOUT_MS    300         /* 超过此时间无有效帧则停机 */

/* Fast binary frame: FE + X(u16 BE) + Vx(i32 BE, *1000) + FD. */
#define K230_BINARY_FRAME_SIZE 8u

typedef struct {
    u8  fresh_target;
    u8  has_target;                    /* 1=有效目标, 0=丢失 */
    u16 cx, cy;                        /* 色板中心像素坐标 (0~640, 0~480) */
    u16 w, h;                          /* 色板宽高 */
    u16 interval_ms;                   /* 与上一有效视觉帧的间隔 */
    s32 velocity_milli_px_s;
    u8  velocity_valid;
} K230_Frame;

/* 初始化 USART2 + DMA + IDLE中断 */
void K230_Init(uint32_t baudrate);

/* 原子获取最新帧: 1=已复制新帧, 0=无新帧 */
u8 K230_GetNewFrame(K230_Frame *f);

/* 由1ms SysTick调用；用于摄像头失联保护 */
void K230_Tick1ms(void);
u8 K230_LinkAlive(void);

/* 发送字符串到 K230 (阻塞) */
void K230_SendString(const char *str);

/* VOFA+ JustFloat monitor frame on the existing USART2 TX (PA2). */
void K230_SendVOFAFrame(const float *values, u8 count);

/* USART2 RXNE 中断处理 (在 stm32f10x_it.c 中调用) */
void K230_RX_IRQ(void);

#endif
