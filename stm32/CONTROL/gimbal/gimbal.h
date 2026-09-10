#ifndef __GIMBAL_H__
#define __GIMBAL_H__

/*
 * 此文件保留原工程路径，以免修改 Keil 工程；内容已改为单轴滚球平衡。
 * K230 每帧发送: X:<cx>,Y:<cy>,W:<w>,H:<h>\r\n
 */
#include "sys.h"
#include "motor.h"
#include "pid.h"
#include "k230_proto.h"

/* ====== 2026-07-31 MaixCAM 实测标定 ======
 * 轨道有效长度 240mm，中心到两端各 120mm。
 * 远离电机端 X=53，中点 X=336，靠近电机端 X=606。
 * 广角镜头存在透视差，因此中心两侧分别换算，不再用单一比例。
 */
#define BALL_AXIS_USE_X                 1u
#define BALL_AXIS_CENTER_PX             336.0f
#define BALL_FAR_END_PX                  53.0f
#define BALL_MOTOR_END_PX               606.0f
#define BALL_HALF_LENGTH_MM             120.0f
#define BALL_PX_PER_MM_FAR_SIDE           2.358333f
#define BALL_PX_PER_MM_MOTOR_SIDE         2.250000f
#define BALL_IMAGE_TO_BEAM_SIGN         1.0f
#define BALL_TARGET_MM                  0.0f

/* 640x480 画面中的三点标定值。联调时按屏幕实测坐标修改这三项。 */
#define BALL_POS_ZERO_PX                336.0f
#define BALL_POS_PLUS_5CM_PX            448.5f
#define BALL_POS_MINUS_5CM_PX           218.1f

/* 实测两端正好约为 ±120 mm。额外保留 3 mm 检测框抖动余量；
 * 超出 ±123 mm 才认为坐标异常并停止。 */
#define BALL_VALID_MIN_MM             (-123.0f)
#define BALL_VALID_MAX_MM               123.0f

/* 慢速双环控制：位置环输出“目标轨道倾角”，不再直接输出电机转速。
 * 50 mm 误差对应约 2.5°，最大只允许 3°，到点后目标倾角自动回到 0°。 */
#define BALL_PID_KP                     0.100f
#define BALL_PID_KI                     0.004f
#define BALL_PID_KD                     0.110f
#define BALL_STICTION_KI_DEG_PER_MM_S   0.002f
#define BALL_STICTION_I_LIMIT_MM_S       40.0f
#define BALL_STICTION_SPEED_GATE_MM_S    6.0f
#define BALL_STICTION_ERROR_GATE_MM     10.0f
#define BALL_STICTION_I_DECAY_PER_S      10.0f
#define BALL_DEADBAND_MM                 2.0f
#define BALL_SETTLE_SPEED_MM_S          15.0f
#define BALL_DRIVE_TILT_NEAR_DEG        3.0f
#define BALL_DRIVE_TILT_MAX_DEG         10.0f
#define BALL_BRAKE_TILT_NEAR_DEG        7.0f
#define BALL_BRAKE_TILT_MAX_DEG         12.0f
#define BALL_OUTPUT_LIMIT_NEAR_ERROR_MM 10.0f
#define BALL_OUTPUT_LIMIT_FAR_ERROR_MM  60.0f
#define BALL_LEFT_SIDE_POSITION_SIGN   (-1.0f) /* B<0 is image/track left */
#define BALL_LEFT_SIDE_DRIVE_SCALE       1.00f
#define BALL_LEFT_SIDE_BRAKE_SCALE       1.00f
#define BALL_ASYMMETRIC_OUTPUT_MAX_DEG  20.0f
#define BALL_TARGET_TILT_MAX_DEG        BALL_ASYMMETRIC_OUTPUT_MAX_DEG
#define BALL_POSITION_TO_TILT_SIGN       1.0f
#define BALL_DYNAMIC_ZONE_MM             10.0f
#define BALL_DYNAMIC_ZONE_POSITION_SCALE  0.25f
#define BALL_FRICTION_FF_TILT_DEG          0.8f
#define BALL_CENTER_CREEP_TILT_DEG         1.2f
#define BALL_HOLD_ZONE_MM                 10.0f
#define BALL_HOLD_SPEED_GATE_MM_S         25.0f
#define BALL_NO_EARLY_RETURN_ERROR_MM     10.0f
#define BALL_NO_EARLY_RETURN_SPEED_MM_S   15.0f
#define BALL_NO_EARLY_RETURN_TILT_DEG      1.2f

/* Distance-adaptive slew limit for the outer-loop angle command.  The
 * actual per-frame step is rate * measured camera frame interval. */
#define BALL_OUTPUT_SLEW_NEAR_ERROR_MM     5.0f
#define BALL_OUTPUT_SLEW_FAR_ERROR_MM     60.0f
#define BALL_OUTPUT_SLEW_NEAR_DEG_S        6.0f
#define BALL_OUTPUT_SLEW_FAR_DEG_S        40.0f
#define BALL_OUTPUT_SLEW_FAST_MULT          3.0f
#define BALL_OUTPUT_SLEW_CROSS_MULT         8.0f
#define BALL_CROSS_BRAKE_MIN_STEP_DEG       1.2f
#define BALL_PROFILE_BRAKE_MIN_STEP_DEG     1.0f
#define BALL_OUTPUT_STEP_MIN_DEG            0.15f
#define BALL_OUTPUT_STEP_MAX_DEG            4.0f

/* Distance-adaptive inertia cancellation.  Far from target the ball is
 * allowed to build useful speed; close to target stronger velocity damping
 * removes kinetic energy.  The transition uses the same 5..60 mm range as
 * the adaptive output slew limiter. */
#define BALL_INERTIA_KD_NEAR                 0.240f
#define BALL_INERTIA_KD_FAR                  0.130f
#define BALL_STALL_TILT_GAIN_DEG_PER_MM  0.150f
#define BALL_STALL_TILT_MAX_DEG          5.0f
#define BALL_STALL_DETECT_ERROR_MM       10.0f
#define BALL_STALL_DETECT_SPEED_MM_S      3.0f
#define BALL_STALL_BOOST_DELAY_S          0.35f
#define BALL_STALL_BOOST_RAMP_DEG_S       2.0f
#define BALL_STALL_BOOST_DECAY_DEG_S      8.0f
#define BALL_STALL_BOOST_MAX_DEG          2.0f
#define BALL_STALL_DYNAMIC_MAX_DEG        7.0f

/* Measured one-sided rail depression: the ball can remain stationary near
 * B=+27.6 mm even with about +7 degrees of commanded/actual rail angle.
 * For a centre target only, latch a stronger escape tilt after the normal
 * stall delay and keep it until the ball has cleared the depression. */
#define BALL_DIP_TARGET_WINDOW_MM          3.0f
#define BALL_DIP_ENTER_MIN_MM             18.0f
#define BALL_DIP_ENTER_MAX_MM             38.0f
#define BALL_DIP_EXIT_MM                  12.0f
#define BALL_DIP_ESCAPE_SPEED_MM_S         8.0f
#define BALL_DIP_HANDOFF_SPEED_MM_S         5.0f
#define BALL_DIP_ESCAPE_TILT_DEG            9.0f
#define BALL_DIP_CARRY_TILT_DEG             2.0f

/* Terminal energy damping: a strong reverse brake while approaching the
 * target, followed by a smaller return tilt after it crosses the target. */
#define BALL_TERMINAL_WINDOW_MM         20.0f
#define BALL_TERMINAL_SPEED_MM_S         15.0f
#define BALL_TERMINAL_BRAKE_TILT_DEG     10.0f
#define BALL_TERMINAL_RETURN_TILT_DEG     1.0f

/* Position-velocity coupled return profile.  sqrt(2*a*distance) is the
 * highest ball speed that can still be stopped at the recorded target. */
#define BALL_PROFILE_BRAKE_DECEL_MM_S2    90.0f
#define BALL_PROFILE_MAX_SPEED_MM_S      140.0f
#define BALL_PROFILE_BRAKE_MARGIN_MM_S     6.0f
#define BALL_PROFILE_BRAKE_BASE_DEG         7.0f
#define BALL_PROFILE_BRAKE_GAIN_DEG_PER_MM_S 0.16f
#define BALL_PROFILE_RETURN_GAIN_DEG_PER_MM_S 0.040f

/* K2 starts with a short open-loop anti-stiction dither.  One RPM is only
 * 0.1 degree per 16.7 ms, so the pulse remains small while Emm can overcome
 * static friction with its immediate reverse acceleration. */
#define BALL_UNSTICK_PULSE_S             0.080f
#define BALL_UNSTICK_TOTAL_S             0.240f
#define BALL_UNSTICK_RPM                 1

/* 视觉速度估算：先对位置做已有低通，再对速度继续低通并限幅，
 * 防止 YOLO 检测框的一两个像素抖动产生突然反向。 */
#define BALL_VELOCITY_FILTER_ALPHA      0.35f
#define BALL_VELOCITY_MAX_MM_S          400.0f

/* Two-state constant-velocity Kalman filter for camera position/velocity. */
#define BALL_KALMAN_POS_R_PX2           16.0f
#define BALL_KALMAN_VEL_R_PX2_S2        900.0f
#define BALL_KALMAN_ACCEL_NOISE_PX_S2   500.0f

/* 角度内环：轨道每相差 1°，只命令约 0.5 RPM；最高 2 RPM。 */
#define BALL_ANGLE_KP_RPM_PER_DEG       1.00f
#define BALL_ANGLE_KD_RPM_PER_DPS        0.08f
#define BALL_ANGLE_DEADBAND_DEG         0.10f
#define BALL_MOTOR_COMMAND_THRESHOLD_RPM 0.15f
#define BALL_MOTOR_MAX_RPM              6.0f
#define BALL_MOTOR_MIN_RPM              1.0f

/* 若钢珠偏右时电机动作使其继续向右滚，把此值改为 -1.0f。 */
#define BALL_MOTOR_COMMAND_SIGN         1.0f

/* 电机相对“上电时人工调平位置”的软件安全窗口。
 * 未安装物理限位前，先保守使用 ±30° 电机轴角度。 */
#define BALL_MOTOR_MIN_DEG             (-20.0f)
#define BALL_MOTOR_MAX_DEG               20.0f
#define BALL_MOTOR_SOFT_ZONE_DEG          3.0f
/* 已由原云台实测：正速度通常令编码器角度减小；反了仅修改该项。 */
#define BALL_SPEED_TO_MOTOR_ANGLE_SIGN (-1.0f)

/* Reference-project PID architecture adapted to this 24 cm rail and Emm
 * velocity-mode actuator. Units: mm, degree, second and RPM. */
#define BALL_REF_POS_LPF_ALPHA             0.60f
#define BALL_REF_VEL_LPF_ALPHA             0.30f
#define BALL_REF_PREDICT_HORIZON_S         0.040f
#define BALL_REF_PREDICT_MAX_MM            6.0f
/* Zero-point-only outer loop.  A short prediction and moderate D term remove
 * energy before the ball crosses zero; the small I term only removes a steady
 * rail bias.  The normal angle remains deliberately much smaller than the
 * mechanical safety window. */
#define BALL_REF_OUTER_KP                  0.060f
#define BALL_REF_OUTER_KI                  0.004f
#define BALL_REF_OUTER_KD                  0.080f
#define BALL_REF_OUTER_I_LIMIT           100.0f
#define BALL_REF_OUTER_TILT_LIMIT_DEG      3.5f
#define BALL_REF_OUTER_SLEW_DEG_S         45.0f
#define BALL_REF_OUTER_D_ALPHA             0.30f
/* Final fixed chassis-motion compensation.  The chassis travels from the
 * motor end toward the far end, so keep the motor end raised continuously;
 * this gravity component keeps the ball seated in the small mechanical pit.
 * The measured A/G value near the required level was about +6.9 degrees. */
#define BALL_CAR_MOTION_TILT_BIAS_DEG       7.0f
#define BALL_REF_ZERO_TRACK_BIAS_DEG        BALL_CAR_MOTION_TILT_BIAS_DEG

/* Do not shake the rail continuously.  Within this window the rail returns
 * to level once the ball is slow.  Outside it, a one-direction minimum tilt
 * is allowed only when the measured ball is nearly stationary. */
#define BALL_REF_ZERO_HOLD_BAND_MM          5.0f
#define BALL_REF_ZERO_HOLD_SPEED_MM_S      10.0f
#define BALL_REF_STICTION_SPEED_MM_S        6.0f
#define BALL_REF_STICTION_MIN_TILT_DEG      1.2f
/* Reference inner output is deg/s.  Emm accepts RPM, so divide gains and
 * limit by 6 (one RPM = 6 deg/s). */
#define BALL_REF_INNER_KP_RPM_PER_DEG      2.000f
#define BALL_REF_INNER_KI_RPM_PER_DEG_S    0.000f
#define BALL_REF_INNER_KD_RPM_PER_DPS      0.008333f
#define BALL_REF_INNER_I_LIMIT             8.0f
#define BALL_REF_INNER_RPM_LIMIT           5.0f
#define BALL_REF_INNER_DEADBAND_DEG        0.0f
#define BALL_REF_INNER_D_ALPHA             0.25f

typedef struct {
    Motor_State *motor;
    PID_State    pid;
    PID_State    angle_pid;
    K230_Frame   target;
    float        ball_mm;
    float        filtered_axis_px;
    float        target_mm;
    float        error_mm;
    float        last_ball_mm;
    float        ball_velocity_mm_s;
    float        stiction_integral_mm_s;
    float        stall_elapsed_s;
    float        stall_boost_deg;
    u8           dip_escape_active;
    float        kalman_x_px;
    float        kalman_v_px_s;
    float        kalman_p00;
    float        kalman_p01;
    float        kalman_p10;
    float        kalman_p11;
    float        desired_angle_deg;
    float        raw_desired_angle_deg;
    float        output_step_limit_deg;
    float        effective_kd;
    float        inertia_compensation_deg;
    float        active_drive_limit_deg;
    float        active_brake_limit_deg;
    float        position_term_deg;
    float        velocity_term_deg;
    float        stiction_term_deg;
    float        friction_term_deg;
    float        profile_speed_mm_s;
    float        pre_limit_desired_angle_deg;
    float        tilt_limit_deg;
    float        angle_error_deg;
    float        inner_p_rpm;
    float        inner_d_rpm;
    float        inner_speed_raw_rpm;
    u8           profile_braking;
    u8           friction_ff_active;
    u8           terminal_return_active;
    u8           early_return_guard;
    u8           outer_output_saturated;
    u8           inner_output_saturated;
    u8           crossing_brake_active;
    float        track_angle_deg;
    float        track_rate_dps;
    float        command_rpm;
    u8           tracking;
    u8           filter_ready;
    u8           velocity_ready;
    u8           range_fault;
    u8           control_enabled;
    u8           angle_command_valid;
    u8           unstick_active;
    float        unstick_elapsed_s;
    u8           dither_active;
    u8           dither_sign;
    float        dither_elapsed_s;
    u32          frame_counter;
    u8           motor_error;       /* 最近一次速度命令结果，0=正常 */
} BallBalance_System;

void BallBalance_Init(BallBalance_System *b, Motor_State *motor);
u8   BallBalance_Enable(BallBalance_System *b);
void BallBalance_Disable(BallBalance_System *b);
void BallBalance_EmergencyStop(BallBalance_System *b);
void BallBalance_SetZero(BallBalance_System *b);
void BallBalance_SetControlEnabled(BallBalance_System *b, u8 enabled);
void BallBalance_SetTargetMm(BallBalance_System *b, float target_mm);
void BallBalance_SetTargetPx(BallBalance_System *b, float target_px);
float BallBalance_GetAxisPx(const BallBalance_System *b);
float BallBalance_PixelToMm(float axis_px);
float BallBalance_MmToPixel(float position_mm);
void BallBalance_Update(BallBalance_System *b, float dt_s);
void BallBalance_InnerUpdate(BallBalance_System *b);
void BallBalance_GetStatus(const BallBalance_System *b, float *ball_mm,
                           float *target_mm, float *error_mm,
                           float *command_rpm);

#endif
