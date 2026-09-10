#include "pid.h"
#include <math.h>

static float PID_Clamp(float value, float min_value, float max_value)
{
    if (value > max_value) return max_value;
    if (value < min_value) return min_value;
    return value;
}

void PID_Init(PID_State *pid, float kp, float ki, float kd,
              float integral_max, float out_max, float deadband,
              float derivative_alpha)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->out_max = out_max;
    pid->integral_max = integral_max;
    pid->deadband = deadband;
    pid->derivative_alpha = PID_Clamp(derivative_alpha, 0.0f, 1.0f);
    PID_Reset(pid);
}

void PID_Reset(PID_State *pid)
{
    pid->integral = 0.0f;
    pid->last_error = 0.0f;
    pid->derivative = 0.0f;
    pid->initialized = 0u;
}

void PID_SetTuning(PID_State *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

float PID_Update(PID_State *pid, float error, float dt_s)
{
    float effective_error;
    float raw_derivative;
    float next_integral;
    float p_out, i_out, d_out, unsaturated_output;
    u8 allow_integral;

    if (dt_s <= 0.0f) return 0.0f;
    effective_error = (fabs(error) < pid->deadband) ? 0.0f : error;

    if (pid->initialized == 0u) {
        raw_derivative = 0.0f;
        pid->initialized = 1u;
    } else {
        raw_derivative = (effective_error - pid->last_error) / dt_s;
    }
    pid->derivative += pid->derivative_alpha *
                       (raw_derivative - pid->derivative);

    next_integral = PID_Clamp(pid->integral + effective_error * dt_s,
                              -pid->integral_max, pid->integral_max);
    p_out = pid->kp * effective_error;
    i_out = pid->ki * next_integral;
    d_out = pid->kd * pid->derivative;
    unsaturated_output = p_out + i_out + d_out;

    /* Conditional integration prevents windup at either output limit. */
    allow_integral = (fabs(unsaturated_output) <= pid->out_max) ||
                     ((unsaturated_output > pid->out_max) &&
                      (effective_error < 0.0f)) ||
                     ((unsaturated_output < -pid->out_max) &&
                      (effective_error > 0.0f));
    if (allow_integral) pid->integral = next_integral;

    pid->last_error = effective_error;
    i_out = pid->ki * pid->integral;
    return PID_Clamp(p_out + i_out + d_out,
                     -pid->out_max, pid->out_max);
}
