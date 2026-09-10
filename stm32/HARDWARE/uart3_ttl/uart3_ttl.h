#ifndef __UART3_TTL_H__
#define __UART3_TTL_H__
#include "sys.h"

/* Correct X42S TTL wiring: PB10 TX -> R/A/H, PB11 RX <- T/B/L. */

/* PB10=TX, PB11=RX, 直连电机 R/A/H 和 T/B/L */
void UART3_TTL_Init(uint32_t baudrate);
void UART3_SendByte(u8 data);
void UART3_SendBytes(u8 *buf, u16 len);
void UART3_SendString(const char *str);
u8   UART3_RecvByte(u8 *byte, u32 timeout_ms);
u16  UART3_RecvBytes(u8 *buf, u16 max_len, u32 timeout_ms);
void UART3_Flush(void);
void UART3_RX_IRQ(void);

#endif
