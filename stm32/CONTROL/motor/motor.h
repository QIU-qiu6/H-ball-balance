#ifndef __MOTOR_H__
#define __MOTOR_H__
#include "sys.h"
#include "zdt_proto.h"

/* ================================================================
 * 电机抽象层
 *
 * 封装 ZDT_Port + 电机状态.
 * 每个电机独立绑定一个 UART 通道.
 * ================================================================ */

#define MOTOR_COMMAND_PULSES_PER_REV  3200  /* 位置命令脉冲/圈（16细分） */
#define MOTOR_POSITION_COUNTS_PER_REV 65536 /* 0x36实时位置计数/圈 */

#define MOTOR_HOME_TIMEOUT_MS          5000u

typedef struct {
    ZDT_Port *port;                   /* ZDT 协议端口 (UART通道) */
    int32_t  encoder_raw;             /* 原始编码器值 */
    int32_t  encoder_zero;            /* 零点偏移 */
    float    angle_deg;               /* 当前角度 (°) */
    int16_t  speed_rpm;               /* 当前转速 (RPM) */
    u8       online;                  /* 在线标志 */
    u8       enabled;                 /* 使能标志 */
    u8       busy;                    /* 忙碌标志 */
} Motor_State;

/* 初始化 */
void Motor_Init(Motor_State *m, ZDT_Port *port);

/* 使能/失能 */
u8 Motor_Enable(Motor_State *m);
u8 Motor_Disable(Motor_State *m);

/* 紧急停止 */
u8 Motor_EmergencyStop(Motor_State *m);

/* 速度控制 (RPM, 带方向) */
u8 Motor_SetSpeed(Motor_State *m, int16_t speed_rpm);

/* 相对位置移动 (角度制) */
u8 Motor_MoveRelative(Motor_State *m, float delta_deg, u16 speed_rpm);

/* 绝对位置移动 */
u8 Motor_MoveAbsolute(Motor_State *m, float angle_deg, u16 speed_rpm);

/* 读反馈 */
u8 Motor_ReadPosition(Motor_State *m);
u8 Motor_ReadSpeed(Motor_State *m);

/* 只读检测 X / Emm 固件，同时确认处于闭环模式。 */
u8 Motor_DetectFirmware(Motor_State *m);

/* Execute the stored Emm single-turn absolute-home sequence. */
u8 Motor_HomeSingleTurn(Motor_State *m, u32 timeout_ms);

/* 当前位置设为零点 */
u8 Motor_SetZero(Motor_State *m);

/* 在线检测 (读一次位置, 更新online标志) */
u8 Motor_Ping(Motor_State *m);

/* 编码器 ↔ 角度转换 */
float   Motor_EncoderToAngle(const Motor_State *m, int32_t encoder);
int32_t Motor_AngleToEncoder(float angle_deg);

#endif
