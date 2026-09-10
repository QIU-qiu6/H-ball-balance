/**
 * 电机抽象层实现
 */
#include "motor.h"
#include "delay.h"

/* ---- 编码器 ↔ 角度 ---- */
float Motor_EncoderToAngle(const Motor_State *m, int32_t encoder)
{
    /* 0x36 的单位随固件而变：Emm 为 65536 counts/圈，
       X 为 0.1 度。 */
    if (m->port->firmware == ZDT_FW_X) {
        return (float)encoder / 10.0f;
    }
    return (float)encoder / (float)MOTOR_POSITION_COUNTS_PER_REV * 360.0f;
}

int32_t Motor_AngleToEncoder(float angle_deg)
{
    return (int32_t)(angle_deg / 360.0f * (float)MOTOR_COMMAND_PULSES_PER_REV);
}

/* ---- 初始化 ---- */
void Motor_Init(Motor_State *m, ZDT_Port *port)
{
    m->port          = port;
    m->encoder_raw   = 0;
    m->encoder_zero  = 0;
    m->angle_deg     = 0.0f;
    m->speed_rpm     = 0;
    m->online        = 0;
    m->enabled       = 0;
    m->busy          = 0;
}

u8 Motor_DetectFirmware(Motor_State *m)
{
    u8 options;
    u8 ret = ZDT_ReadOptions(m->port, &options);

    if (ret != 0) {
        m->online = 0;
        return ret;
    }
    /* bit2=1 表示闭环；平衡程序拒绝开环模式。 */
    if ((options & 0x04u) == 0u) {
        m->online = 0;
        return 0xFC;
    }
    m->online = 1;
    return 0;
}

/* ---- 使能 ---- */
u8 Motor_HomeSingleTurn(Motor_State *m, u32 timeout_ms)
{
    u32 elapsed = 0u;
    u8 ret;
    u8 status = 0u;

    if (m->port->firmware != ZDT_FW_EMM) return 0xFC;

    ret = ZDT_StartSingleTurnHome(m->port);
    if (ret != 0) return ret;

    /* The drive needs a short settling interval after enable/home command.
     * Completion is still confirmed by 0x3B rather than this delay. */
    Delay_ms(300);
    while (elapsed < timeout_ms) {
        ret = ZDT_ReadHomeStatus(m->port, &status);
        if (ret == ZDT_RET_BUSY) {
            Delay_ms(50);
            elapsed += 50u;
            continue;
        }
        if (ret != 0) return ret;
        if ((status & 0x03u) != 0x03u) return 0xF9;
        if (status & 0x08u) return 0xF8;
        if ((status & 0x04u) == 0u) {
            m->speed_rpm = 0;
            return 0;
        }
        Delay_ms(50);
        elapsed += 50u;
    }
    return 0xF7;
}

u8 Motor_Enable(Motor_State *m)
{
    u8 ret = ZDT_Enable(m->port, ZDT_ENABLE);
    if (ret == 0) {
        m->enabled = 1;
        m->online  = 1;
    }
    return ret;
}

u8 Motor_Disable(Motor_State *m)
{
    u8 ret = ZDT_Enable(m->port, ZDT_DISABLE);
    if (ret == 0) m->enabled = 0;
    return ret;
}

/* ---- 急停 ---- */
u8 Motor_EmergencyStop(Motor_State *m)
{
    u8 data[] = {0x98, 0x00};
    /* Emergency path sends immediately and never waits for a reply. */
    ZDT_SendRaw(m->port, ZDT_CMD_STOP, data, 2);
    m->speed_rpm = 0;
    m->enabled = 0;
    return 0;
}

/* ---- 速度控制 ---- */
u8 Motor_SetSpeed(Motor_State *m, int16_t speed_rpm)
{
    u8 dir;
    u8 ret;
    u16 abs_speed;
    u16 accel;

    /* 闭环每帧可能得到相同的整数 RPM，避免重复阻塞发送相同帧。 */
    if (speed_rpm == m->speed_rpm) return 0;

    if (speed_rpm == 0) {
        ret = ZDT_Stop(m->port);
        if (ret == 0) m->speed_rpm = 0;
        return ret;
    }

    dir       = (speed_rpm > 0) ? ZDT_DIR_CCW : ZDT_DIR_CW;
    abs_speed = (u16)((speed_rpm > 0) ? speed_rpm : -speed_rpm);

    /* 沿用已实测能转的角度工程策略：当前 F6 帧失败时，用另一种
     * X/Emm F6 参数格式重试一次，成功后记住该协议。 */
    /* This target has verified Emm firmware. Never fall back to an X frame
     * during control, because a transient UART failure must not change the
     * electrical command format. */
    if (m->port->firmware != ZDT_FW_EMM) return 0xFC;
    accel = ZDT_EMM_SPEED_ACCEL_DEFAULT;
    if ((speed_rpm > 0 && m->speed_rpm < 0) ||
        (speed_rpm < 0 && m->speed_rpm > 0)) {
        accel = ZDT_EMM_REVERSE_ACCEL;
    }
    ret = ZDT_GoSpeedAccel(m->port, dir, abs_speed, accel);
    if (ret != 0) return ret;

    m->speed_rpm = speed_rpm;
    return 0;
}

/* ---- 相对位置移动 ---- */
u8 Motor_MoveRelative(Motor_State *m, float delta_deg, u16 speed_rpm)
{
    int32_t pulses;
    u8 dir;

    /* 本工程的球平衡只使用速度模式。X 固件的位置帧格式不同，
       禁止沿用旧 Emm 位置帧，避免误动作。 */
    if (m->port->firmware != ZDT_FW_EMM) return 0xFC;
    pulses = Motor_AngleToEncoder(delta_deg);
    dir    = (pulses >= 0) ? ZDT_DIR_CCW : ZDT_DIR_CW;
    if (pulses < 0) pulses = -pulses;

    return ZDT_GoPos(m->port, dir, speed_rpm, (u32)pulses, 0);
}

/* ---- 绝对位置移动 ---- */
u8 Motor_MoveAbsolute(Motor_State *m, float angle_deg, u16 speed_rpm)
{
    int32_t pulses;
    u8 dir;

    if (m->port->firmware != ZDT_FW_EMM) return 0xFC;
    pulses = Motor_AngleToEncoder(angle_deg);
    dir    = (pulses >= 0) ? ZDT_DIR_CCW : ZDT_DIR_CW;
    if (pulses < 0) pulses = -pulses;

    return ZDT_GoPos(m->port, dir, speed_rpm, (u32)pulses, 1);
}

/* ---- 反馈 ---- */
u8 Motor_ReadPosition(Motor_State *m)
{
    int32_t pos;
    u8 ret = ZDT_ReadPos(m->port, &pos);
    if (ret == 0) {
        m->encoder_raw = pos;
        m->angle_deg   = Motor_EncoderToAngle(m, pos - m->encoder_zero);
        m->online      = 1;
    } else {
        m->online = 0;
    }
    return ret;
}

u8 Motor_ReadSpeed(Motor_State *m)
{
    int16_t spd;
    u8 ret = ZDT_ReadSpeed(m->port, &spd);
    if (ret == 0) m->speed_rpm = spd;
    return ret;
}

/* ---- 设零点 ---- */
u8 Motor_SetZero(Motor_State *m)
{
    int32_t pos;
    u8 ret = ZDT_ReadPos(m->port, &pos);
    if (ret == 0) {
        m->encoder_zero = pos;
        m->angle_deg    = 0.0f;
    }
    return ret;
}

/* ---- 在线检测 ---- */
u8 Motor_Ping(Motor_State *m)
{
    int32_t pos;
    u8 ret = ZDT_ReadPos(m->port, &pos);
    m->online = (ret == 0) ? 1 : 0;
    return ret;
}
