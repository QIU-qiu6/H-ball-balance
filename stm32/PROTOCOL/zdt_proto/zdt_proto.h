#ifndef __ZDT_PROTO_H__
#define __ZDT_PROTO_H__
#include "sys.h"

/* ================================================================
 * ZDT X42S 闭环步进电机 — 串口 TTL 协议。
 * X 与 Emm 固件的速度/位置命令格式不同；启动时通过 0x1A
 * 读取选项状态，自动识别固件类型，不写入任何电机参数。
 *
 * 帧格式: [Addr(1B)] [Cmd(1B)] [Data(NB)] [0x6B]
 *
 * 多电机支持: 每个电机绑定一个 ZDT_Port (不同UART通道)
 * ================================================================ */

/* ---- 端口抽象 (多电机核心) ---- */
typedef struct {
    void (*send)(u8 *buf, u16 len);           /* 发送函数 */
    void (*flush)(void);                      /* 清空接收缓冲 */
    u16  (*recv)(u8 *buf, u16 max, u32 to);   /* 接收函数 (直接寄存器访问) */
    u8   addr;                                /* 电机地址 (默认0x01) */
    u8   last_rx[12];
    u8   last_rx_len;
    u8   last_error;
    u8   firmware;                            /* ZDT_FW_* */
    u8   options;                             /* 0x1A 返回的状态字 */
} ZDT_Port;

/* ---- 通信参数 ---- */
#define ZDT_BAUDRATE       115200
#define ZDT_CHECKSUM_6B    0x6B

/* ---- 功能码 ---- */
#define ZDT_CMD_ENABLE     0xF3
#define ZDT_CMD_SPEED      0xF6
#define ZDT_CMD_POS_TRAP   0xFD
#define ZDT_CMD_STOP       0xFE
#define ZDT_CMD_READ_POS   0x36
#define ZDT_CMD_READ_SPEED 0x35
#define ZDT_CMD_READ_OPTIONS 0x1A
#define ZDT_CMD_HOME_STATUS 0x3B
#define ZDT_CMD_HOME        0x9A
#define ZDT_CMD_MULTI_SYNC 0xFF

/* ---- 返回码 ---- */
#define ZDT_RET_OK    0x02
#define ZDT_RET_BUSY  0xE2
#define ZDT_RET_ERROR 0xEE

/* ---- 方向 ---- */
#define ZDT_DIR_CW    0x00
#define ZDT_DIR_CCW   0x01

/* ---- 标志 ---- */
#define ZDT_ENABLE     0x01
#define ZDT_DISABLE    0x00
#define ZDT_SYNC_OFF   0x00
#define ZDT_SYNC_ON    0x01

/* Emm 单圈绝对零点回零模式：回到电机 EEPROM 已保存的零位，
 * 不会把当前角度误写成零位。 */
#define ZDT_HOME_SINGLE_TURN_NEAREST 0x00u

/* 0x1A bit1: 0=X, 1=Emm. */
#define ZDT_FW_UNKNOWN 0u
#define ZDT_FW_X       1u
#define ZDT_FW_EMM     2u

/* EMM F6 speed-mode acceleration. The vendor protocol defines 0 as a
 * direct start; normal moves use a nonzero value to remain smoother. */
#define ZDT_EMM_SPEED_ACCEL_DEFAULT 160u   /* 1..255, 越大加速越快 */
#define ZDT_EMM_REVERSE_ACCEL         0u   /* 0=direct start */
#define ZDT_X_SPEED_ACCEL_DEFAULT    120u  /* RPM/s */

/* ---- API (所有函数第一个参数为 ZDT_Port*) ---- */
u8  ZDT_Enable  (ZDT_Port *p, u8 en);
u8  ZDT_GoPos   (ZDT_Port *p, u8 dir, u16 speed, u32 pulses, u8 absolute);
u8  ZDT_GoSpeed (ZDT_Port *p, u8 dir, u16 speed);
u8  ZDT_GoSpeedAccel(ZDT_Port *p, u8 dir, u16 speed, u16 accel);
u8  ZDT_Stop    (ZDT_Port *p);
u8  ZDT_ReadPos (ZDT_Port *p, int32_t *pos);
u8  ZDT_ReadSpeed(ZDT_Port *p, int16_t *spd);
u8  ZDT_ReadOptions(ZDT_Port *p, u8 *options);
u8  ZDT_StartSingleTurnHome(ZDT_Port *p);
u8  ZDT_ReadHomeStatus(ZDT_Port *p, u8 *status);
u8  ZDT_Sync    (ZDT_Port *p);

/* 发送原始命令帧 (用于诊断) */
u8  ZDT_SendRaw (ZDT_Port *p, u8 cmd, u8 *data, u8 len);

#endif
