#ifndef __PID_H__
#define __PID_H__
#include "sys.h"

/* PID with derivative low-pass filtering and conditional-integration
 * anti-windup. All values use physical units supplied by the caller. */
typedef struct {
    float kp, ki, kd;
    float integral;
    float last_error;
    float out_max;
    float integral_max;
    float deadband;
    float derivative;
    float derivative_alpha;
    u8 initialized;
} PID_State;

void PID_Init(PID_State *pid, float kp, float ki, float kd,
              float integral_max, float out_max, float deadband,
              float derivative_alpha);
float PID_Update(PID_State *pid, float error, float dt_s);
void PID_Reset(PID_State *pid);
void PID_SetTuning(PID_State *pid, float kp, float ki, float kd);

#endif
