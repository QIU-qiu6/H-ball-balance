/* 单轴滚球平衡控制器，文件名保持 gimbal.c 以复用原 Keil 工程。 */
#include "gimbal.h"
#include <math.h>

static float Ball_Abs(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float Ball_Clamp(float value, float low, float high)
{
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

float BallBalance_PixelToMm(float axis_px)
{
    float delta_px = (axis_px - BALL_AXIS_CENTER_PX) *
                     BALL_IMAGE_TO_BEAM_SIGN;
    if (delta_px >= 0.0f) {
        return delta_px / BALL_PX_PER_MM_MOTOR_SIDE;
    }
    return delta_px / BALL_PX_PER_MM_FAR_SIDE;
}

float BallBalance_MmToPixel(float position_mm)
{
    float delta_px;
    if (position_mm >= 0.0f) {
        delta_px = position_mm * BALL_PX_PER_MM_MOTOR_SIDE;
    } else {
        delta_px = position_mm * BALL_PX_PER_MM_FAR_SIDE;
    }
    return BALL_AXIS_CENTER_PX + delta_px / BALL_IMAGE_TO_BEAM_SIGN;
}

/* 输出速度的正负号和编码器角度正方向并不总是一致。
 * 只在朝向极限继续运动时削弱该方向，离开极限的方向始终允许。 */
static float Ball_ApplyMotorLimit(const BallBalance_System *b, float speed)
{
    float angle = b->motor->angle_deg;
    float angular_direction;
    float remain;
    float scale;

    if (speed == 0.0f) return 0.0f;
    angular_direction = (speed > 0.0f) ? BALL_SPEED_TO_MOTOR_ANGLE_SIGN :
                                         -BALL_SPEED_TO_MOTOR_ANGLE_SIGN;

    if (angular_direction > 0.0f) {
        remain = BALL_MOTOR_MAX_DEG - angle;
        if (remain <= 0.0f) return 0.0f;
        if (remain < BALL_MOTOR_SOFT_ZONE_DEG) {
            scale = remain / BALL_MOTOR_SOFT_ZONE_DEG;
            return speed * Ball_Clamp(scale, 0.0f, 1.0f);
        }
    } else {
        remain = angle - BALL_MOTOR_MIN_DEG;
        if (remain <= 0.0f) return 0.0f;
        if (remain < BALL_MOTOR_SOFT_ZONE_DEG) {
            scale = remain / BALL_MOTOR_SOFT_ZONE_DEG;
            return speed * Ball_Clamp(scale, 0.0f, 1.0f);
        }
    }
    return speed;
}

static int16_t Ball_ToRpm(float speed)
{
    float magnitude;

    speed = Ball_Clamp(speed, -BALL_MOTOR_MAX_RPM, BALL_MOTOR_MAX_RPM);
    if (Ball_Abs(speed) < BALL_MOTOR_COMMAND_THRESHOLD_RPM) return 0;

    magnitude = Ball_Abs(speed);
    if (magnitude < BALL_MOTOR_MIN_RPM) {
        speed = (speed > 0.0f) ? BALL_MOTOR_MIN_RPM : -BALL_MOTOR_MIN_RPM;
    }
    return (speed > 0.0f) ? (int16_t)(speed + 0.5f) :
                             (int16_t)(speed - 0.5f);
}

static void Ball_StopAndReset(BallBalance_System *b)
{
    if (b->tracking || b->command_rpm != 0.0f) {
        Motor_SetSpeed(b->motor, 0);
    }
    PID_Reset(&b->pid);
    b->stiction_integral_mm_s = 0.0f;
    b->stall_elapsed_s = 0.0f;
    b->stall_boost_deg = 0.0f;
    b->dip_escape_active = 0u;
    b->desired_angle_deg = b->track_angle_deg;
    b->raw_desired_angle_deg = b->track_angle_deg;
    b->output_step_limit_deg = 0.0f;
    b->effective_kd = BALL_PID_KD;
    b->inertia_compensation_deg = 0.0f;
    b->active_drive_limit_deg = BALL_DRIVE_TILT_NEAR_DEG;
    b->active_brake_limit_deg = BALL_BRAKE_TILT_NEAR_DEG;
    b->command_rpm = 0.0f;
    b->tracking = 0;
}

static u8 Ball_UnstickStep(BallBalance_System *b, float dt_s)
{
    int16_t rpm;
    float phase;
    u8 ret;

    if (!b->unstick_active) return 0u;
    b->unstick_elapsed_s += Ball_Clamp(dt_s, 0.005f, 0.050f);
    if (b->unstick_elapsed_s >= BALL_UNSTICK_TOTAL_S) {
        b->unstick_active = 0u;
        b->command_rpm = 0.0f;
        b->tracking = 0u;
        ret = Motor_SetSpeed(b->motor, 0);
        b->motor_error = ret;
        PID_Reset(&b->pid);
        b->stiction_integral_mm_s = 0.0f;
        return 1u;
    }

    phase = b->unstick_elapsed_s / BALL_UNSTICK_PULSE_S;
    rpm = (((u8)phase & 1u) == 0u) ? BALL_UNSTICK_RPM :
                                      -BALL_UNSTICK_RPM;
    ret = Motor_SetSpeed(b->motor, rpm);
    b->motor_error = ret;
    if (ret != 0u) {
        b->unstick_active = 0u;
        b->command_rpm = 0.0f;
        b->tracking = 0u;
        return 1u;
    }
    b->command_rpm = (float)rpm;
    b->tracking = (ret == 0u) ? 1u : 0u;
    return 1u;
}

void BallBalance_Init(BallBalance_System *b, Motor_State *motor)
{
    b->motor = motor;
    b->target.has_target = 0;
    b->target.cx = (u16)BALL_AXIS_CENTER_PX;
    b->target.cy = 240u;
    b->target.w = 0;
    b->target.h = 0;
    b->ball_mm = 0.0f;
    b->filtered_axis_px = BALL_AXIS_CENTER_PX;
    b->target_mm = BALL_TARGET_MM;
    b->error_mm = 0.0f;
    b->last_ball_mm = 0.0f;
    b->ball_velocity_mm_s = 0.0f;
    b->stiction_integral_mm_s = 0.0f;
    b->stall_elapsed_s = 0.0f;
    b->stall_boost_deg = 0.0f;
    b->dip_escape_active = 0u;
    b->kalman_x_px = BALL_AXIS_CENTER_PX;
    b->kalman_v_px_s = 0.0f;
    b->kalman_p00 = 25.0f;
    b->kalman_p01 = 0.0f;
    b->kalman_p10 = 0.0f;
    b->kalman_p11 = 2500.0f;
    b->desired_angle_deg = 0.0f;
    b->raw_desired_angle_deg = 0.0f;
    b->output_step_limit_deg = 0.0f;
    b->effective_kd = BALL_PID_KD;
    b->inertia_compensation_deg = 0.0f;
    b->active_drive_limit_deg = BALL_DRIVE_TILT_NEAR_DEG;
    b->active_brake_limit_deg = BALL_BRAKE_TILT_NEAR_DEG;
    b->position_term_deg = 0.0f;
    b->velocity_term_deg = 0.0f;
    b->stiction_term_deg = 0.0f;
    b->friction_term_deg = 0.0f;
    b->profile_speed_mm_s = 0.0f;
    b->pre_limit_desired_angle_deg = 0.0f;
    b->tilt_limit_deg = BALL_DRIVE_TILT_NEAR_DEG;
    b->angle_error_deg = 0.0f;
    b->inner_p_rpm = 0.0f;
    b->inner_d_rpm = 0.0f;
    b->inner_speed_raw_rpm = 0.0f;
    b->profile_braking = 0u;
    b->friction_ff_active = 0u;
    b->terminal_return_active = 0u;
    b->early_return_guard = 0u;
    b->outer_output_saturated = 0u;
    b->inner_output_saturated = 0u;
    b->crossing_brake_active = 0u;
    b->track_angle_deg = 0.0f;
    b->track_rate_dps = 0.0f;
    b->command_rpm = 0.0f;
    b->tracking = 0;
    b->filter_ready = 0;
    b->velocity_ready = 0;
    b->range_fault = 0;
    b->control_enabled = 0;
    b->angle_command_valid = 0;
    b->unstick_active = 0;
    b->unstick_elapsed_s = 0.0f;
    b->frame_counter = 0;
    b->motor_error = 0;

    PID_Init(&b->pid, BALL_PID_KP, BALL_PID_KI, BALL_PID_KD,
             BALL_STICTION_I_LIMIT_MM_S,
             BALL_TARGET_TILT_MAX_DEG,
             BALL_DEADBAND_MM,
             BALL_REF_OUTER_D_ALPHA);
}

u8 BallBalance_Enable(BallBalance_System *b)
{
    return Motor_Enable(b->motor);
}

void BallBalance_Disable(BallBalance_System *b)
{
    Ball_StopAndReset(b);
    Motor_Disable(b->motor);
}

void BallBalance_EmergencyStop(BallBalance_System *b)
{
    Motor_EmergencyStop(b->motor);
    PID_Reset(&b->pid);
    b->stiction_integral_mm_s = 0.0f;
    b->stall_elapsed_s = 0.0f;
    b->stall_boost_deg = 0.0f;
    b->dip_escape_active = 0u;
    b->unstick_active = 0u;
    b->unstick_elapsed_s = 0.0f;
    b->command_rpm = 0.0f;
    b->tracking = 0;
}

void BallBalance_SetZero(BallBalance_System *b)
{
    Motor_SetZero(b->motor);
    b->track_angle_deg = 0.0f;
    b->track_rate_dps = 0.0f;
}

void BallBalance_SetControlEnabled(BallBalance_System *b, u8 enabled)
{
    enabled = enabled ? 1u : 0u;
    if (b->control_enabled != enabled) {
        PID_Reset(&b->pid);
        b->stiction_integral_mm_s = 0.0f;
        b->stall_elapsed_s = 0.0f;
        b->stall_boost_deg = 0.0f;
        b->dip_escape_active = 0u;
    }
    b->control_enabled = enabled;
    b->angle_command_valid = 0u;
    if (enabled) {
        b->unstick_active = 1u;
        b->unstick_elapsed_s = 0.0f;
    } else {
        b->unstick_active = 0u;
        b->unstick_elapsed_s = 0.0f;
    }
    if (!enabled) {
        Ball_StopAndReset(b);
    }
}

void BallBalance_SetTargetMm(BallBalance_System *b, float target_mm)
{
    target_mm = Ball_Clamp(target_mm, BALL_VALID_MIN_MM, BALL_VALID_MAX_MM);
    if (Ball_Abs(target_mm - b->target_mm) > 0.01f) {
        PID_Reset(&b->pid);
        b->stiction_integral_mm_s = 0.0f;
        b->stall_elapsed_s = 0.0f;
        b->stall_boost_deg = 0.0f;
        b->dip_escape_active = 0u;
    }
    b->target_mm = target_mm;
}

void BallBalance_SetTargetPx(BallBalance_System *b, float target_px)
{
    float target_mm = BallBalance_PixelToMm(target_px);
    BallBalance_SetTargetMm(b, target_mm);
}

float BallBalance_GetAxisPx(const BallBalance_System *b)
{
    return b->filtered_axis_px;
}

void BallBalance_InnerUpdate(BallBalance_System *b)
{
    float angle_error;
    float speed;
    int16_t rpm;
    u8 motor_ret;

    if (!b->control_enabled || b->unstick_active ||
        !b->angle_command_valid) return;
    if (!K230_LinkAlive() || !b->target.has_target) {
        Motor_SetSpeed(b->motor, 0);
        b->command_rpm = 0.0f;
        b->tracking = 0u;
        return;
    }

    /* No rail-mounted angle sensor is used.  The Emm encoder position is the
     * inner-loop angle feedback; the motor query is refreshed in main.c. */
    b->track_angle_deg = b->motor->angle_deg;
    /* Speed-mode command is used as a short-term rate estimate between the
     * relatively slow absolute-position queries. */
    b->track_rate_dps = BALL_SPEED_TO_MOTOR_ANGLE_SIGN *
                        (float)b->motor->speed_rpm * 6.0f;

    angle_error = b->desired_angle_deg - b->track_angle_deg;
    b->angle_error_deg = angle_error;
    b->inner_p_rpm = angle_error * BALL_ANGLE_KP_RPM_PER_DEG;
    b->inner_d_rpm = -b->track_rate_dps * BALL_ANGLE_KD_RPM_PER_DPS;
    if (Ball_Abs(angle_error) < BALL_ANGLE_DEADBAND_DEG) {
        speed = 0.0f;
    } else {
        speed = (b->inner_p_rpm + b->inner_d_rpm) /
                BALL_SPEED_TO_MOTOR_ANGLE_SIGN;
    }
    speed *= BALL_MOTOR_COMMAND_SIGN;
    b->inner_speed_raw_rpm = speed;
    speed = Ball_ApplyMotorLimit(b, speed);
    b->inner_output_saturated =
        (Ball_Abs(speed - b->inner_speed_raw_rpm) > 0.01f ||
         Ball_Abs(b->inner_speed_raw_rpm) > BALL_MOTOR_MAX_RPM) ? 1u : 0u;
    rpm = Ball_ToRpm(speed);

    motor_ret = Motor_SetSpeed(b->motor, rpm);
    b->motor_error = motor_ret;
    if (motor_ret != 0u) {
        Ball_StopAndReset(b);
        b->motor_error = motor_ret;
        return;
    }
    b->command_rpm = (float)rpm;
    b->tracking = (rpm != 0) ? 1u : 0u;
}

static void Ball_KalmanReset(BallBalance_System *b, float position_px)
{
    b->kalman_x_px = position_px;
    b->kalman_v_px_s = 0.0f;
    b->kalman_p00 = 25.0f;
    b->kalman_p01 = 0.0f;
    b->kalman_p10 = 0.0f;
    b->kalman_p11 = 2500.0f;
    b->filter_ready = 1u;
}

static void Ball_KalmanUpdate(BallBalance_System *b, float position_px,
                              float dt_s, u8 velocity_valid,
                              s32 velocity_milli_px_s)
{
    float dt2;
    float q;
    float q00, q01, q11;
    float x, v, p00, p01, p10, p11;
    float old_p00, old_p01, old_p10, old_p11;
    float innovation, innovation_cov, k0, k1;
    float measured_velocity;

    if (!b->filter_ready) {
        Ball_KalmanReset(b, position_px);
        return;
    }

    dt_s = Ball_Clamp(dt_s, 0.005f, 0.150f);
    dt2 = dt_s * dt_s;
    q = BALL_KALMAN_ACCEL_NOISE_PX_S2 *
        BALL_KALMAN_ACCEL_NOISE_PX_S2;
    q00 = 0.25f * dt2 * dt2 * q;
    q01 = 0.5f * dt2 * dt_s * q;
    q11 = dt2 * q;

    x = b->kalman_x_px + b->kalman_v_px_s * dt_s;
    v = b->kalman_v_px_s;
    p00 = b->kalman_p00 + dt_s * (b->kalman_p10 + b->kalman_p01) +
          dt2 * b->kalman_p11 + q00;
    p01 = b->kalman_p01 + dt_s * b->kalman_p11 + q01;
    p10 = b->kalman_p10 + dt_s * b->kalman_p11 + q01;
    p11 = b->kalman_p11 + q11;

    /* Position measurement update. */
    innovation = position_px - x;
    innovation_cov = p00 + BALL_KALMAN_POS_R_PX2;
    if (innovation_cov > 0.001f) {
        old_p00 = p00;
        old_p01 = p01;
        old_p10 = p10;
        old_p11 = p11;
        k0 = p00 / innovation_cov;
        k1 = p10 / innovation_cov;
        x += k0 * innovation;
        v += k1 * innovation;
        p00 = (1.0f - k0) * old_p00;
        p01 = (1.0f - k0) * old_p01;
        p10 = old_p10 - k1 * old_p00;
        p11 = old_p11 - k1 * old_p01;
    }

    /* Optional camera velocity measurement from the compact binary frame. */
    if (velocity_valid) {
        measured_velocity = (float)velocity_milli_px_s * 0.001f;
        innovation = measured_velocity - v;
        innovation_cov = p11 + BALL_KALMAN_VEL_R_PX2_S2;
        if (innovation_cov > 0.001f) {
            k0 = p01 / innovation_cov;
            k1 = p11 / innovation_cov;
            x += k0 * innovation;
            v += k1 * innovation;
            p00 -= k0 * p10;
            p01 -= k0 * p11;
            p10 -= k1 * p10;
            p11 -= k1 * p11;
        }
    }

    b->kalman_x_px = Ball_Clamp(x, 0.0f, (float)K230_IMAGE_WIDTH - 1.0f);
    b->kalman_v_px_s = Ball_Clamp(v, -1200.0f, 1200.0f);
    b->kalman_p00 = Ball_Clamp(p00, 0.001f, 100000.0f);
    b->kalman_p01 = p01;
    b->kalman_p10 = p10;
    b->kalman_p11 = Ball_Clamp(p11, 0.001f, 1000000.0f);
}

void BallBalance_Update(BallBalance_System *b, float dt_s)
{
    K230_Frame f;
    float axis_px;
    float frame_dt_s;
    float tilt_limit;
    float drive_tilt_limit;
    float brake_tilt_limit;
    float correction_sign;
    float profile_speed_abs;
    float speed_abs;
    float stiction_i_term;
    float decay;
    float min_tilt_deg;
    float previous_desired_angle_deg;
    float output_distance_ratio;
    float output_slew_rate_deg_s;
    float output_delta_deg;
    float output_limit_ratio;
    float inertia_distance_ratio;
    float position_error_for_pid;
    u8 profile_braking;
    u8 early_return_guard;
    u8 crossing_brake;

    early_return_guard = 0u;
    b->profile_braking = 0u;
    b->friction_ff_active = 0u;
    b->terminal_return_active = 0u;
    b->early_return_guard = 0u;
    b->outer_output_saturated = 0u;
    b->crossing_brake_active = 0u;
    b->friction_term_deg = 0.0f;
    previous_desired_angle_deg = b->desired_angle_deg;

    if (!K230_LinkAlive()) {
        b->target.has_target = 0;
        b->velocity_ready = 0;
        b->ball_velocity_mm_s = 0.0f;
        Ball_StopAndReset(b);
        return;
    }
    if (!K230_GetNewFrame(&f)) return;

    b->target = f;
    if (!f.has_target) {
        b->velocity_ready = 0;
        b->ball_velocity_mm_s = 0.0f;
        Ball_StopAndReset(b);
        return;
    }

    if (!f.fresh_target) {
        /* Keep control continuous across a very short detector dropout. */
        frame_dt_s = 0.02f;
        b->ball_velocity_mm_s = 0.0f;
        b->velocity_ready = 0;
        goto control_step;
    }

    /* Count only real coordinate frames for the K1 capture path. */
    b->frame_counter++;

#if BALL_AXIS_USE_X
    axis_px = (float)f.cx;
#else
    axis_px = (float)f.cy;
#endif
    /* 简单一阶滤波，避免检测框单帧抖动直接驱动电机。 */
    /* 必须按摄像头真实帧间隔估算速度，而不是按 20ms 主循环周期猜测。 */
    frame_dt_s = (float)f.interval_ms * 0.001f;
    frame_dt_s = Ball_Clamp(frame_dt_s, 0.005f, 0.150f);
    Ball_KalmanUpdate(b, axis_px, frame_dt_s,
                      f.velocity_valid, f.velocity_milli_px_s);
    b->filtered_axis_px = b->kalman_x_px;
    b->ball_mm = BallBalance_PixelToMm(b->filtered_axis_px);
    if (b->filtered_axis_px >= BALL_AXIS_CENTER_PX) {
        b->ball_velocity_mm_s = b->kalman_v_px_s /
                               BALL_PX_PER_MM_MOTOR_SIDE;
    } else {
        b->ball_velocity_mm_s = b->kalman_v_px_s /
                               BALL_PX_PER_MM_FAR_SIDE;
    }
    b->ball_velocity_mm_s = Ball_Clamp(b->ball_velocity_mm_s,
                                       -BALL_VELOCITY_MAX_MM_S,
                                        BALL_VELOCITY_MAX_MM_S);
    if (b->ball_mm < BALL_VALID_MIN_MM || b->ball_mm > BALL_VALID_MAX_MM) {
        b->range_fault = 1;
        Ball_StopAndReset(b);
        return;
    }
    b->range_fault = 0;

    /* 菜单和坐标记录状态只更新视觉位置，不允许PID驱动电机。 */
control_step:
    if (!b->control_enabled) {
        Ball_StopAndReset(b);
        return;
    }

    output_limit_ratio =
        (Ball_Abs(b->target_mm - b->ball_mm) -
         BALL_OUTPUT_LIMIT_NEAR_ERROR_MM) /
        (BALL_OUTPUT_LIMIT_FAR_ERROR_MM -
         BALL_OUTPUT_LIMIT_NEAR_ERROR_MM);
    output_limit_ratio = Ball_Clamp(output_limit_ratio, 0.0f, 1.0f);
    drive_tilt_limit = BALL_DRIVE_TILT_NEAR_DEG +
        output_limit_ratio *
        (BALL_DRIVE_TILT_MAX_DEG - BALL_DRIVE_TILT_NEAR_DEG);
    brake_tilt_limit = BALL_BRAKE_TILT_NEAR_DEG +
        output_limit_ratio *
        (BALL_BRAKE_TILT_MAX_DEG - BALL_BRAKE_TILT_NEAR_DEG);
    if (b->ball_mm * BALL_LEFT_SIDE_POSITION_SIGN > 0.0f) {
        drive_tilt_limit *= BALL_LEFT_SIDE_DRIVE_SCALE;
        brake_tilt_limit *= BALL_LEFT_SIDE_BRAKE_SCALE;
        drive_tilt_limit = Ball_Clamp(drive_tilt_limit, 0.0f,
                                      BALL_ASYMMETRIC_OUTPUT_MAX_DEG);
        brake_tilt_limit = Ball_Clamp(brake_tilt_limit, 0.0f,
                                      BALL_ASYMMETRIC_OUTPUT_MAX_DEG);
    }
    b->active_drive_limit_deg = drive_tilt_limit;
    b->active_brake_limit_deg = brake_tilt_limit;

    /* Do not let the position PID fight the anti-stiction pulse.  Once this
     * short open-loop phase ends, the normal angle and position loops take
     * over on the next frame. */
    if (Ball_UnstickStep(b, frame_dt_s)) return;

    b->error_mm = b->target_mm - b->ball_mm;

    /* The saved K1 coordinate is always the endpoint.  This speed envelope
     * turns the return into accelerate-then-decelerate motion: close to the
     * endpoint, the permitted approach speed naturally falls to zero. */
    profile_speed_abs = sqrtf(2.0f * BALL_PROFILE_BRAKE_DECEL_MM_S2 *
                              Ball_Abs(b->error_mm));
    profile_speed_abs = Ball_Clamp(profile_speed_abs, 0.0f,
                                   BALL_PROFILE_MAX_SPEED_MM_S);
    speed_abs = Ball_Abs(b->ball_velocity_mm_s);
    crossing_brake =
        (b->error_mm * b->ball_velocity_mm_s < 0.0f &&
         speed_abs >= BALL_TERMINAL_SPEED_MM_S) ? 1u : 0u;
    b->crossing_brake_active = crossing_brake;
    profile_braking =
        (b->error_mm * b->ball_velocity_mm_s > 0.0f &&
         speed_abs > profile_speed_abs + BALL_PROFILE_BRAKE_MARGIN_MM_S) ?
        1u : 0u;
    b->profile_speed_mm_s = profile_speed_abs;
    b->profile_braking = profile_braking;

    /* Gated static-friction compensation.  Accumulate only while the ball
     * is clearly away from the target and nearly stopped; while moving or
     * braking, bleed the state to prevent integral windup and overshoot. */
    if (Ball_Abs(b->error_mm) <= BALL_STICTION_ERROR_GATE_MM ||
        (b->stiction_integral_mm_s != 0.0f &&
         b->error_mm * b->stiction_integral_mm_s < 0.0f)) {
        b->stiction_integral_mm_s = 0.0f;
    } else if (!profile_braking &&
               Ball_Abs(b->ball_velocity_mm_s) <
                   BALL_STICTION_SPEED_GATE_MM_S) {
        b->stiction_integral_mm_s += b->error_mm * frame_dt_s;
        b->stiction_integral_mm_s = Ball_Clamp(
            b->stiction_integral_mm_s,
            -BALL_STICTION_I_LIMIT_MM_S,
             BALL_STICTION_I_LIMIT_MM_S);
    } else {
        decay = 1.0f - BALL_STICTION_I_DECAY_PER_S * frame_dt_s;
        decay = Ball_Clamp(decay, 0.80f, 1.0f);
        b->stiction_integral_mm_s *= decay;
    }
    /* Use the requested small Ki as an additional, gated static-friction
     * integral.  The state is still conditionally accumulated, clamped and
     * decayed above, so it cannot wind up during fast motion or braking. */
    stiction_i_term = (BALL_STICTION_KI_DEG_PER_MM_S + b->pid.ki) *
                      b->stiction_integral_mm_s;
    stiction_i_term = Ball_Clamp(stiction_i_term, -1.0f, 1.0f);

    /* A fixed tilt can form another equilibrium inside a mechanical dip.
     * Detect a ball that is genuinely stopped outside the target region and
     * ramp only the extra breakaway tilt.  As soon as the ball moves, remove
     * this boost quickly so the normal velocity damping can brake it. */
    if (!profile_braking &&
        Ball_Abs(b->error_mm) >= BALL_STALL_DETECT_ERROR_MM &&
        speed_abs <= BALL_STALL_DETECT_SPEED_MM_S) {
        b->stall_elapsed_s += frame_dt_s;
        if (b->stall_elapsed_s >= BALL_STALL_BOOST_DELAY_S) {
            b->stall_boost_deg += BALL_STALL_BOOST_RAMP_DEG_S * frame_dt_s;
            b->stall_boost_deg = Ball_Clamp(b->stall_boost_deg, 0.0f,
                                            BALL_STALL_BOOST_MAX_DEG);
        }
    } else {
        b->stall_elapsed_s = 0.0f;
        b->stall_boost_deg -= BALL_STALL_BOOST_DECAY_DEG_S * frame_dt_s;
        if (b->stall_boost_deg < 0.0f) b->stall_boost_deg = 0.0f;
    }

    /* Location-specific escape for the measured depression near B=+27.6.
     * State 1 is a short breakaway tilt.  Once motion toward the target is
     * confirmed, state 2 hands control back to PD while capping any further
     * drive tilt; reverse braking remains available. */
    if (!b->dip_escape_active &&
        Ball_Abs(b->target_mm) <= BALL_DIP_TARGET_WINDOW_MM &&
        b->ball_mm >= BALL_DIP_ENTER_MIN_MM &&
        b->ball_mm <= BALL_DIP_ENTER_MAX_MM &&
        speed_abs <= BALL_DIP_ESCAPE_SPEED_MM_S &&
        b->stall_elapsed_s >= BALL_STALL_BOOST_DELAY_S) {
        b->dip_escape_active = 1u;
    }
    if (b->dip_escape_active == 1u &&
        b->error_mm * b->ball_velocity_mm_s > 0.0f &&
        speed_abs >= BALL_DIP_HANDOFF_SPEED_MM_S) {
        b->dip_escape_active = 2u;
    }
    if (b->dip_escape_active &&
        (Ball_Abs(b->target_mm) > BALL_DIP_TARGET_WINDOW_MM ||
         b->ball_mm <= BALL_DIP_EXIT_MM ||
         (b->error_mm * b->ball_velocity_mm_s < 0.0f &&
          speed_abs >= BALL_DIP_HANDOFF_SPEED_MM_S))) {
        b->dip_escape_active = 0u;
    }

    /* 小积分只在球已经较慢、且靠近目标时工作，用来克服轨道零偏
     * 和静摩擦。积分输出单独限幅，不能把轨道越积越斜。 */
    /* 外环 PID：位置项拉向目标，速度项提前反向制动，积分项消除
     * 最终静止位置与中心之间的小偏差。 */
    /* Distance-adaptive D term: weaker far away for useful acceleration,
     * stronger near target to remove inertia before the crossing. */
    inertia_distance_ratio =
        (Ball_Abs(b->error_mm) - BALL_OUTPUT_SLEW_NEAR_ERROR_MM) /
        (BALL_OUTPUT_SLEW_FAR_ERROR_MM -
         BALL_OUTPUT_SLEW_NEAR_ERROR_MM);
    inertia_distance_ratio = Ball_Clamp(inertia_distance_ratio, 0.0f, 1.0f);
    b->effective_kd = BALL_INERTIA_KD_NEAR +
        inertia_distance_ratio *
        (BALL_INERTIA_KD_FAR - BALL_INERTIA_KD_NEAR);
    b->inertia_compensation_deg = b->effective_kd *
                                   b->ball_velocity_mm_s;

    /* Inside the accepted +/-1 cm dynamic zone, keep velocity damping but
     * weaken the position term.  The controller then balances dynamically
     * instead of fighting for an exact, motionless zero coordinate. */
    position_error_for_pid = b->error_mm;
    if (Ball_Abs(position_error_for_pid) < BALL_DYNAMIC_ZONE_MM) {
        position_error_for_pid *= BALL_DYNAMIC_ZONE_POSITION_SCALE +
            (1.0f - BALL_DYNAMIC_ZONE_POSITION_SCALE) *
            Ball_Abs(position_error_for_pid) / BALL_DYNAMIC_ZONE_MM;
    }

    b->position_term_deg = BALL_POSITION_TO_TILT_SIGN *
                           (b->pid.kp * position_error_for_pid);
    b->stiction_term_deg = BALL_POSITION_TO_TILT_SIGN * stiction_i_term;
    b->velocity_term_deg = BALL_POSITION_TO_TILT_SIGN *
                           (-b->inertia_compensation_deg);

    /* Pure outer-loop PD: position creates the restoring tilt and the
     * adaptive velocity term supplies direct anticipatory braking. */
    b->desired_angle_deg = b->position_term_deg +
                           b->stiction_term_deg +
                           b->velocity_term_deg;

    /* Coulomb-friction feedforward.  Apply only while starting or moving
     * toward the target and only outside the accepted dynamic zone.  During
     * braking, natural friction is useful and must not be cancelled. */
    if (!profile_braking &&
        Ball_Abs(b->error_mm) > BALL_DYNAMIC_ZONE_MM &&
        b->error_mm * b->ball_velocity_mm_s >= 0.0f) {
        correction_sign = (b->error_mm > 0.0f) ?
                          BALL_POSITION_TO_TILT_SIGN :
                         -BALL_POSITION_TO_TILT_SIGN;
        b->desired_angle_deg += correction_sign *
                                BALL_FRICTION_FF_TILT_DEG;
        b->friction_term_deg = correction_sign * BALL_FRICTION_FF_TILT_DEG;
        b->friction_ff_active = 1u;
    }

    if (profile_braking) {
        /* Outside the braking envelope: use a larger opposite slope now,
         * rather than waiting until the ball has already crossed target. */
        b->desired_angle_deg = (b->ball_velocity_mm_s > 0.0f) ?
            Ball_Clamp(BALL_PROFILE_BRAKE_BASE_DEG +
                       (speed_abs - profile_speed_abs) *
                       BALL_PROFILE_BRAKE_GAIN_DEG_PER_MM_S,
                       BALL_PROFILE_BRAKE_BASE_DEG,
                       brake_tilt_limit) :
           -Ball_Clamp(BALL_PROFILE_BRAKE_BASE_DEG +
                       (speed_abs - profile_speed_abs) *
                       BALL_PROFILE_BRAKE_GAIN_DEG_PER_MM_S,
                       BALL_PROFILE_BRAKE_BASE_DEG,
                       brake_tilt_limit);
    } else if (!profile_braking &&
               Ball_Abs(b->error_mm) < BALL_TERMINAL_WINDOW_MM &&
               b->error_mm * b->ball_velocity_mm_s < 0.0f &&
               speed_abs >= BALL_TERMINAL_SPEED_MM_S) {
        /* If it has crossed the saved target, pull it back with a smaller
         * slope.  The rail then gives the remaining energy to gravity and
         * friction instead of launching another large overshoot. */
        correction_sign = (b->error_mm > 0.0f) ?
                          BALL_POSITION_TO_TILT_SIGN :
                         -BALL_POSITION_TO_TILT_SIGN;
        b->desired_angle_deg = correction_sign * Ball_Clamp(
            BALL_TERMINAL_RETURN_TILT_DEG +
            speed_abs * BALL_PROFILE_RETURN_GAIN_DEG_PER_MM_S,
            BALL_TERMINAL_RETURN_TILT_DEG, drive_tilt_limit);
        b->terminal_return_active = 1u;
    }

    if (b->dip_escape_active == 1u) {
        correction_sign = (b->error_mm > 0.0f) ?
                          BALL_POSITION_TO_TILT_SIGN :
                         -BALL_POSITION_TO_TILT_SIGN;
        b->desired_angle_deg = correction_sign * BALL_DIP_ESCAPE_TILT_DEG;
    } else if (b->dip_escape_active == 2u) {
        correction_sign = (b->error_mm > 0.0f) ?
                          BALL_POSITION_TO_TILT_SIGN :
                         -BALL_POSITION_TO_TILT_SIGN;
        if (b->desired_angle_deg * correction_sign >
            BALL_DIP_CARRY_TILT_DEG) {
            b->desired_angle_deg = correction_sign *
                                   BALL_DIP_CARRY_TILT_DEG;
        }
    }

    /* Do not let braking reverse the ball before it has entered the
     * required target region.  If an approaching ball has already slowed
     * almost to zero outside +/-8 mm, replace a remaining reverse-brake
     * command with a small forward creep.  Normal PD braking resumes as
     * soon as the ball enters the target region or its speed rises again. */
    if (Ball_Abs(b->error_mm) > BALL_NO_EARLY_RETURN_ERROR_MM &&
        b->error_mm * b->ball_velocity_mm_s >= 0.0f &&
        speed_abs <= BALL_NO_EARLY_RETURN_SPEED_MM_S) {
        correction_sign = (b->error_mm > 0.0f) ?
                          BALL_POSITION_TO_TILT_SIGN :
                         -BALL_POSITION_TO_TILT_SIGN;
        if (b->desired_angle_deg * correction_sign <= 0.0f) {
            b->desired_angle_deg = correction_sign *
                                   BALL_NO_EARLY_RETURN_TILT_DEG;
            early_return_guard = 1u;
            b->early_return_guard = 1u;
        }
    }

    /* 球几乎停住但仍偏离中心时，保证至少有一个很小的回中倾角，
     * 避免比例输出太小而克服不了钢珠/轨道静摩擦。 */
    if (Ball_Abs(b->error_mm) >= BALL_DEADBAND_MM &&
        (Ball_Abs(b->ball_velocity_mm_s) < BALL_SETTLE_SPEED_MM_S ||
         (Ball_Abs(b->error_mm) > BALL_HOLD_ZONE_MM &&
          Ball_Abs(b->ball_velocity_mm_s) < BALL_HOLD_SPEED_GATE_MM_S &&
          b->error_mm * b->ball_velocity_mm_s <= 0.0f))) {
        correction_sign = (b->error_mm > 0.0f) ?
                          BALL_POSITION_TO_TILT_SIGN :
                         -BALL_POSITION_TO_TILT_SIGN;
        if (Ball_Abs(b->desired_angle_deg) <
            BALL_CENTER_CREEP_TILT_DEG) {
            b->desired_angle_deg = correction_sign *
                                   BALL_CENTER_CREEP_TILT_DEG;
        }

        /* Outside the hold zone, do not accept a small fixed tilt as a
         * final solution.  Increase the low-speed escape tilt with distance
         * so a ball in a shallow mechanical dip keeps being pulled toward
         * the saved target until it enters the zone. */
        if (Ball_Abs(b->error_mm) > BALL_HOLD_ZONE_MM &&
            !early_return_guard) {
            min_tilt_deg = BALL_CENTER_CREEP_TILT_DEG +
                (Ball_Abs(b->error_mm) - BALL_HOLD_ZONE_MM) *
                BALL_STALL_TILT_GAIN_DEG_PER_MM;
            min_tilt_deg = Ball_Clamp(min_tilt_deg,
                                      BALL_CENTER_CREEP_TILT_DEG,
                                      BALL_STALL_TILT_MAX_DEG);
            min_tilt_deg += b->stall_boost_deg;
            min_tilt_deg = Ball_Clamp(min_tilt_deg,
                                      BALL_CENTER_CREEP_TILT_DEG,
                                      BALL_STALL_DYNAMIC_MAX_DEG);
            if (Ball_Abs(b->desired_angle_deg) < min_tilt_deg) {
                b->desired_angle_deg = correction_sign * min_tilt_deg;
            }
        }
    } else if (Ball_Abs(b->error_mm) < BALL_DEADBAND_MM &&
               Ball_Abs(b->ball_velocity_mm_s) <
                   BALL_SETTLE_SPEED_MM_S &&
               Ball_Abs(b->desired_angle_deg) < 0.12f) {
        b->desired_angle_deg = 0.0f;
    }

    /* If the ball has been stationary away from the target long enough,
     * apply the measured stall boost as a real additional breakaway tilt.
     * Previously the boost only raised a minimum-angle threshold; when the
     * normal PD output was already larger than that threshold, no extra
     * torque was generated and the ball could remain stuck indefinitely. */
    if (!profile_braking && !crossing_brake &&
        Ball_Abs(b->error_mm) >= BALL_STALL_DETECT_ERROR_MM &&
        speed_abs <= BALL_STALL_DETECT_SPEED_MM_S &&
        b->stall_boost_deg > 0.0f) {
        correction_sign = (b->error_mm > 0.0f) ?
                          BALL_POSITION_TO_TILT_SIGN :
                         -BALL_POSITION_TO_TILT_SIGN;
        b->desired_angle_deg += correction_sign * b->stall_boost_deg;
    }

    /* 朝目标运动时允许更大的反向制动倾角；其余时间仍限制为较小
     * 的驱动倾角，所以提高刹车能力不会重新加快起步。 */
    if ((b->error_mm * b->ball_velocity_mm_s > 0.0f &&
         Ball_Abs(b->ball_velocity_mm_s) >= BALL_SETTLE_SPEED_MM_S) ||
        crossing_brake) {
        tilt_limit = brake_tilt_limit;
    } else {
        tilt_limit = drive_tilt_limit;
    }
    if (b->dip_escape_active == 1u &&
        tilt_limit < BALL_DIP_ESCAPE_TILT_DEG) {
        tilt_limit = BALL_DIP_ESCAPE_TILT_DEG;
    }
    b->pre_limit_desired_angle_deg = b->desired_angle_deg;
    b->outer_output_saturated =
        (Ball_Abs(b->desired_angle_deg) > tilt_limit) ? 1u : 0u;
    b->tilt_limit_deg = tilt_limit;
    b->desired_angle_deg = Ball_Clamp(b->desired_angle_deg,
                                      -tilt_limit, tilt_limit);

    /* Distance-adaptive PID output step.  Far from target the requested
     * angle can move at nearly the motor's physical rate; close to target
     * each change is much smaller, suppressing sign-flip chatter. */
    b->raw_desired_angle_deg = b->desired_angle_deg;
    output_distance_ratio =
        (Ball_Abs(b->error_mm) - BALL_OUTPUT_SLEW_NEAR_ERROR_MM) /
        (BALL_OUTPUT_SLEW_FAR_ERROR_MM -
         BALL_OUTPUT_SLEW_NEAR_ERROR_MM);
    output_distance_ratio = Ball_Clamp(output_distance_ratio, 0.0f, 1.0f);
    output_slew_rate_deg_s = BALL_OUTPUT_SLEW_NEAR_DEG_S +
        output_distance_ratio *
        (BALL_OUTPUT_SLEW_FAR_DEG_S - BALL_OUTPUT_SLEW_NEAR_DEG_S);

    /* Braking and escape are allowed to react faster than ordinary drive.
     * Reducing an existing tilt also removes energy, so do not delay it. */
    if (profile_braking || early_return_guard ||
        b->dip_escape_active == 1u ||
        (b->raw_desired_angle_deg * previous_desired_angle_deg >= 0.0f &&
         Ball_Abs(b->raw_desired_angle_deg) <
         Ball_Abs(previous_desired_angle_deg))) {
        output_slew_rate_deg_s *= BALL_OUTPUT_SLEW_FAST_MULT;
    }
    if (crossing_brake) {
        output_slew_rate_deg_s *= BALL_OUTPUT_SLEW_CROSS_MULT;
    }
    b->output_step_limit_deg = Ball_Clamp(
        output_slew_rate_deg_s * frame_dt_s,
        BALL_OUTPUT_STEP_MIN_DEG, BALL_OUTPUT_STEP_MAX_DEG);
    if (crossing_brake &&
        b->output_step_limit_deg < BALL_CROSS_BRAKE_MIN_STEP_DEG) {
        b->output_step_limit_deg = BALL_CROSS_BRAKE_MIN_STEP_DEG;
    }
    if (profile_braking &&
        b->output_step_limit_deg < BALL_PROFILE_BRAKE_MIN_STEP_DEG) {
        b->output_step_limit_deg = BALL_PROFILE_BRAKE_MIN_STEP_DEG;
    }
    output_delta_deg = b->raw_desired_angle_deg -
                       previous_desired_angle_deg;
    output_delta_deg = Ball_Clamp(output_delta_deg,
                                  -b->output_step_limit_deg,
                                   b->output_step_limit_deg);
    b->desired_angle_deg = previous_desired_angle_deg + output_delta_deg;
    b->desired_angle_deg = Ball_Clamp(b->desired_angle_deg,
                                      -tilt_limit, tilt_limit);
    b->angle_command_valid = 1u;
#if 0

    (void)dt_s;  /* 位置微分使用视觉帧的真实间隔。 */

    /* 内环：目标倾角 -> 电机低速命令。编码器角度正方向和速度
     * 命令正方向由 BALL_SPEED_TO_MOTOR_ANGLE_SIGN 明确换算。 */
    angle_error = b->desired_angle_deg - b->track_angle_deg;
    if (Ball_Abs(angle_error) < BALL_ANGLE_DEADBAND_DEG) {
        speed = 0.0f;
    } else {
        /* Gyro rate damps rail motion before it overshoots the requested
         * angle.  It is zero while the encoder fallback is active. */
        speed = (angle_error * BALL_ANGLE_KP_RPM_PER_DEG -
                 b->track_rate_dps * BALL_ANGLE_KD_RPM_PER_DPS) /
                BALL_SPEED_TO_MOTOR_ANGLE_SIGN;
    }
    speed *= BALL_MOTOR_COMMAND_SIGN;
    speed = Ball_ApplyMotorLimit(b, speed);
    rpm = Ball_ToRpm(speed);

    motor_ret = Motor_SetSpeed(b->motor, rpm);
    b->motor_error = motor_ret;
    if (motor_ret != 0) {
        Ball_StopAndReset(b);
        /* 停止帧不能覆盖导致本次控制失败的原始错误码。 */
        b->motor_error = motor_ret;
        return;
    }
    b->command_rpm = (float)rpm;
    b->tracking = (rpm != 0) ? 1u : 0u;
#endif
}

void BallBalance_GetStatus(const BallBalance_System *b, float *ball_mm,
                           float *target_mm, float *error_mm,
                           float *command_rpm)
{
    *ball_mm = b->ball_mm;
    *target_mm = b->target_mm;
    *error_mm = b->error_mm;
    *command_rpm = b->command_rpm;
}
