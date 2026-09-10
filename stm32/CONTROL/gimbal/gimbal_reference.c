/* Clean two-loop controller based on the supplied reference project.
 * Outer loop: camera ball position -> desired rail angle.
 * Inner loop: desired rail angle -> Emm motor speed command.
 * The previous experimental gimbal.c is intentionally retained on disk;
 * this file is the one selected by the Keil project. */
#include "gimbal.h"
#include <math.h>
#include <string.h>

static float Ball_Abs(float x)
{
    return (x < 0.0f) ? -x : x;
}

static float Ball_Clamp(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

float BallBalance_PixelToMm(float axis_px)
{
    float delta_px = (axis_px - BALL_AXIS_CENTER_PX) *
                     BALL_IMAGE_TO_BEAM_SIGN;
    if (delta_px >= 0.0f) return delta_px / BALL_PX_PER_MM_MOTOR_SIDE;
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

static int16_t Ball_ToRpm(float rpm)
{
    float magnitude;

    rpm = Ball_Clamp(rpm, -BALL_REF_INNER_RPM_LIMIT,
                     BALL_REF_INNER_RPM_LIMIT);
    if (Ball_Abs(rpm) < BALL_MOTOR_COMMAND_THRESHOLD_RPM) return 0;
    magnitude = Ball_Abs(rpm);
    if (magnitude < BALL_MOTOR_MIN_RPM) {
        rpm = (rpm > 0.0f) ? BALL_MOTOR_MIN_RPM : -BALL_MOTOR_MIN_RPM;
    }
    return (rpm > 0.0f) ? (int16_t)(rpm + 0.5f) :
                           (int16_t)(rpm - 0.5f);
}

static float Ball_ApplyMotorLimit(const BallBalance_System *b, float rpm)
{
    float angular_direction;
    float remaining;

    if (rpm == 0.0f) return 0.0f;
    angular_direction = (rpm > 0.0f) ? BALL_SPEED_TO_MOTOR_ANGLE_SIGN :
                                        -BALL_SPEED_TO_MOTOR_ANGLE_SIGN;
    if (angular_direction > 0.0f) {
        remaining = BALL_MOTOR_MAX_DEG - b->motor->angle_deg;
    } else {
        remaining = b->motor->angle_deg - BALL_MOTOR_MIN_DEG;
    }
    if (remaining <= 0.0f) return 0.0f;
    if (remaining < BALL_MOTOR_SOFT_ZONE_DEG) {
        rpm *= remaining / BALL_MOTOR_SOFT_ZONE_DEG;
    }
    return rpm;
}

static void Ball_StopAndReset(BallBalance_System *b)
{
    if (b->command_rpm != 0.0f || b->tracking) {
        (void)Motor_SetSpeed(b->motor, 0);
    }
    PID_Reset(&b->pid);
    PID_Reset(&b->angle_pid);
    b->desired_angle_deg = b->motor->angle_deg;
    b->raw_desired_angle_deg = b->desired_angle_deg;
    b->command_rpm = 0.0f;
    b->tracking = 0u;
    b->angle_command_valid = 0u;
    b->unstick_active = 0u;
    b->stall_elapsed_s = 0.0f;
    b->stall_boost_deg = 0.0f;
    b->dip_escape_active = 0u;
    b->dither_active = 0u;
    b->dither_sign = 1u;
    b->dither_elapsed_s = 0.0f;
}

void BallBalance_Init(BallBalance_System *b, Motor_State *motor)
{
    memset(b, 0, sizeof(*b));
    b->motor = motor;
    b->filtered_axis_px = BALL_AXIS_CENTER_PX;
    b->target_mm = BALL_TARGET_MM;
    b->target.cx = (u16)BALL_AXIS_CENTER_PX;
    b->target.cy = 240u;
    b->active_drive_limit_deg = BALL_REF_OUTER_TILT_LIMIT_DEG;
    b->active_brake_limit_deg = BALL_REF_OUTER_TILT_LIMIT_DEG;
    b->tilt_limit_deg = BALL_REF_OUTER_TILT_LIMIT_DEG;

    PID_Init(&b->pid,
             BALL_REF_OUTER_KP, BALL_REF_OUTER_KI, BALL_REF_OUTER_KD,
             BALL_REF_OUTER_I_LIMIT, BALL_REF_OUTER_TILT_LIMIT_DEG,
             0.0f, BALL_REF_OUTER_D_ALPHA);
    PID_Init(&b->angle_pid,
             BALL_REF_INNER_KP_RPM_PER_DEG,
             BALL_REF_INNER_KI_RPM_PER_DEG_S,
             BALL_REF_INNER_KD_RPM_PER_DPS,
             BALL_REF_INNER_I_LIMIT, BALL_REF_INNER_RPM_LIMIT,
             BALL_REF_INNER_DEADBAND_DEG, BALL_REF_INNER_D_ALPHA);
}

u8 BallBalance_Enable(BallBalance_System *b)
{
    return Motor_Enable(b->motor);
}

void BallBalance_Disable(BallBalance_System *b)
{
    Ball_StopAndReset(b);
    (void)Motor_Disable(b->motor);
}

void BallBalance_EmergencyStop(BallBalance_System *b)
{
    (void)Motor_EmergencyStop(b->motor);
    PID_Reset(&b->pid);
    PID_Reset(&b->angle_pid);
    b->command_rpm = 0.0f;
    b->tracking = 0u;
    b->control_enabled = 0u;
    b->angle_command_valid = 0u;
}

void BallBalance_SetZero(BallBalance_System *b)
{
    (void)Motor_SetZero(b->motor);
    b->track_angle_deg = 0.0f;
    b->track_rate_dps = 0.0f;
}

void BallBalance_SetControlEnabled(BallBalance_System *b, u8 enabled)
{
    enabled = enabled ? 1u : 0u;
    if (b->control_enabled != enabled) {
        PID_Reset(&b->pid);
        PID_Reset(&b->angle_pid);
        b->desired_angle_deg = b->motor->angle_deg;
        b->raw_desired_angle_deg = b->desired_angle_deg;
    }
    b->control_enabled = enabled;
    b->angle_command_valid = 0u;
    b->unstick_active = 0u;
    b->stall_elapsed_s = 0.0f;
    b->stall_boost_deg = 0.0f;
    b->dip_escape_active = 0u;
    b->dither_active = 0u;
    b->dither_sign = 1u;
    b->dither_elapsed_s = 0.0f;
    if (!enabled) Ball_StopAndReset(b);
}

void BallBalance_SetTargetMm(BallBalance_System *b, float target_mm)
{
    b->target_mm = Ball_Clamp(target_mm, BALL_VALID_MIN_MM,
                              BALL_VALID_MAX_MM);
    PID_Reset(&b->pid);
}

void BallBalance_SetTargetPx(BallBalance_System *b, float target_px)
{
    BallBalance_SetTargetMm(b, BallBalance_PixelToMm(target_px));
}

float BallBalance_GetAxisPx(const BallBalance_System *b)
{
    return b->filtered_axis_px;
}

void BallBalance_Update(BallBalance_System *b, float unused_dt_s)
{
    K230_Frame frame;
    float axis_px;
    float frame_dt_s;
    float raw_velocity_mm_s;
    float predicted_mm;
    float desired_raw_deg;
    float correction_deg;
    float max_delta_deg;
    float abs_error_mm;
    float restoring_sign;
    float minimum_tilt_deg;

    (void)unused_dt_s;
    if (!K230_LinkAlive()) {
        b->target.has_target = 0u;
        Ball_StopAndReset(b);
        return;
    }
    if (!K230_GetNewFrame(&frame)) return;
    b->target = frame;

    if (!frame.has_target) {
        Ball_StopAndReset(b);
        return;
    }
    if (!frame.fresh_target) return;

    b->frame_counter++;
#if BALL_AXIS_USE_X
    axis_px = (float)frame.cx;
#else
    axis_px = (float)frame.cy;
#endif
    frame_dt_s = Ball_Clamp((float)frame.interval_ms * 0.001f,
                            0.010f, 0.120f);

    if (!b->filter_ready) {
        b->filtered_axis_px = axis_px;
        b->ball_mm = BallBalance_PixelToMm(axis_px);
        b->last_ball_mm = b->ball_mm;
        b->ball_velocity_mm_s = 0.0f;
        b->filter_ready = 1u;
    } else {
        b->filtered_axis_px += BALL_REF_POS_LPF_ALPHA *
                               (axis_px - b->filtered_axis_px);
        b->ball_mm = BallBalance_PixelToMm(b->filtered_axis_px);
        raw_velocity_mm_s = (b->ball_mm - b->last_ball_mm) / frame_dt_s;
        b->ball_velocity_mm_s += BALL_REF_VEL_LPF_ALPHA *
                                 (raw_velocity_mm_s -
                                  b->ball_velocity_mm_s);
        b->ball_velocity_mm_s = Ball_Clamp(b->ball_velocity_mm_s,
                                            -BALL_VELOCITY_MAX_MM_S,
                                             BALL_VELOCITY_MAX_MM_S);
        b->last_ball_mm = b->ball_mm;
    }

    if (b->ball_mm < BALL_VALID_MIN_MM || b->ball_mm > BALL_VALID_MAX_MM) {
        b->range_fault = 1u;
        Ball_StopAndReset(b);
        return;
    }
    b->range_fault = 0u;
    if (!b->control_enabled) return;

    /* The runtime target is deliberately fixed to the calibrated physical
     * zero.  Keeping this assignment here prevents any stale menu state from
     * silently changing the closed-loop objective. */
    b->target_mm = BALL_TARGET_MM;

    /* Predict only a short distance ahead.  The former 120 ms prediction and
     * continuous square-wave dither both caused premature reversal. */
    predicted_mm = b->ball_mm + b->ball_velocity_mm_s *
                   BALL_REF_PREDICT_HORIZON_S;
    predicted_mm = Ball_Clamp(predicted_mm,
                              b->ball_mm - BALL_REF_PREDICT_MAX_MM,
                              b->ball_mm + BALL_REF_PREDICT_MAX_MM);
    b->error_mm = b->target_mm - b->ball_mm;
    abs_error_mm = Ball_Abs(b->error_mm);
    restoring_sign = (b->error_mm >= 0.0f) ?
                     BALL_POSITION_TO_TILT_SIGN :
                    -BALL_POSITION_TO_TILT_SIGN;
    minimum_tilt_deg = 0.0f;
    b->dither_active = 0u;
    b->dither_elapsed_s = 0.0f;
    b->stall_boost_deg = 0.0f;
    b->stall_elapsed_s = 0.0f;
    b->dip_escape_active = 0u;

    /* Once the ball is both close and slow, clear stored integral energy.
     * The physical level command is still the calibrated +7 degree bias. */
    if (abs_error_mm <= BALL_REF_ZERO_HOLD_BAND_MM &&
        Ball_Abs(b->ball_velocity_mm_s) <= BALL_REF_ZERO_HOLD_SPEED_MM_S) {
        PID_Reset(&b->pid);
        correction_deg = 0.0f;
    } else {
        correction_deg = BALL_POSITION_TO_TILT_SIGN *
            PID_Update(&b->pid, b->target_mm - predicted_mm, frame_dt_s);
        correction_deg = Ball_Clamp(correction_deg,
                                    -BALL_REF_OUTER_TILT_LIMIT_DEG,
                                     BALL_REF_OUTER_TILT_LIMIT_DEG);

        /* A fixed one-direction minimum correction may overcome static
         * friction, but unlike the old dither it never reverses periodically. */
        if (abs_error_mm > BALL_REF_ZERO_HOLD_BAND_MM &&
            Ball_Abs(b->ball_velocity_mm_s) <=
                BALL_REF_STICTION_SPEED_MM_S) {
            minimum_tilt_deg = BALL_REF_STICTION_MIN_TILT_DEG;
        }

        if (minimum_tilt_deg > 0.0f &&
            Ball_Abs(correction_deg) < minimum_tilt_deg) {
            correction_deg = restoring_sign * minimum_tilt_deg;
        }
        b->stall_boost_deg = minimum_tilt_deg;
    }

    /* PID output is only a correction around the fixed chassis-motion bias.
     * At zero error the rail remains at the final +7 degree command. */
    desired_raw_deg = BALL_REF_ZERO_TRACK_BIAS_DEG + correction_deg;
    desired_raw_deg = Ball_Clamp(
        desired_raw_deg,
        BALL_REF_ZERO_TRACK_BIAS_DEG - BALL_REF_OUTER_TILT_LIMIT_DEG,
        BALL_REF_ZERO_TRACK_BIAS_DEG + BALL_REF_OUTER_TILT_LIMIT_DEG);

    /* A slew limit converts noisy camera updates into smooth rail commands. */
    max_delta_deg = BALL_REF_OUTER_SLEW_DEG_S * frame_dt_s;
    b->raw_desired_angle_deg = desired_raw_deg;
    b->desired_angle_deg += Ball_Clamp(desired_raw_deg - b->desired_angle_deg,
                                       -max_delta_deg, max_delta_deg);
    /* The absolute rail command is the fixed +7 degree chassis-motion bias
     * plus/minus the limited PID correction window. */
    b->desired_angle_deg = Ball_Clamp(b->desired_angle_deg,
        BALL_REF_ZERO_TRACK_BIAS_DEG - BALL_REF_OUTER_TILT_LIMIT_DEG,
        BALL_REF_ZERO_TRACK_BIAS_DEG + BALL_REF_OUTER_TILT_LIMIT_DEG);
    b->angle_command_valid = 1u;

    /* Keep legacy telemetry fields meaningful for VOFA/OLED. */
    b->position_term_deg = BALL_POSITION_TO_TILT_SIGN *
                           b->pid.kp * (b->target_mm - predicted_mm);
    b->velocity_term_deg = BALL_POSITION_TO_TILT_SIGN *
                           b->pid.kd * b->pid.derivative;
    b->stiction_integral_mm_s = b->pid.integral;
    b->stiction_term_deg = BALL_POSITION_TO_TILT_SIGN *
                           b->pid.ki * b->pid.integral;
    b->effective_kd = b->pid.kd;
    b->active_drive_limit_deg = BALL_REF_OUTER_TILT_LIMIT_DEG;
    b->active_brake_limit_deg = BALL_REF_OUTER_TILT_LIMIT_DEG;
    b->tilt_limit_deg = BALL_REF_OUTER_TILT_LIMIT_DEG;
    b->output_step_limit_deg = max_delta_deg;
    b->outer_output_saturated =
        (Ball_Abs(correction_deg) >= BALL_REF_OUTER_TILT_LIMIT_DEG) ? 1u : 0u;
}

void BallBalance_InnerUpdate(BallBalance_System *b)
{
    float speed_rpm;
    int16_t command_rpm;
    u8 ret;

    if (!b->control_enabled || !b->angle_command_valid) return;
    if (!K230_LinkAlive() || !b->target.has_target) {
        Ball_StopAndReset(b);
        return;
    }

    b->track_angle_deg = b->motor->angle_deg;
    b->track_rate_dps = BALL_SPEED_TO_MOTOR_ANGLE_SIGN *
                        (float)b->motor->speed_rpm * 6.0f;
    b->angle_error_deg = b->desired_angle_deg - b->track_angle_deg;
    speed_rpm = PID_Update(&b->angle_pid, b->angle_error_deg, 0.005f);
    b->inner_p_rpm = b->angle_pid.kp * b->angle_error_deg;
    b->inner_d_rpm = b->angle_pid.kd * b->angle_pid.derivative;
    b->inner_speed_raw_rpm = speed_rpm;

    speed_rpm /= BALL_SPEED_TO_MOTOR_ANGLE_SIGN;
    speed_rpm *= BALL_MOTOR_COMMAND_SIGN;
    speed_rpm = Ball_ApplyMotorLimit(b, speed_rpm);
    b->inner_output_saturated =
        (Ball_Abs(speed_rpm) >= BALL_REF_INNER_RPM_LIMIT) ? 1u : 0u;
    command_rpm = Ball_ToRpm(speed_rpm);
    ret = Motor_SetSpeed(b->motor, command_rpm);
    b->motor_error = ret;
    if (ret != 0u) {
        Ball_StopAndReset(b);
        b->motor_error = ret;
        return;
    }
    b->command_rpm = (float)command_rpm;
    b->tracking = (command_rpm != 0) ? 1u : 0u;
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
