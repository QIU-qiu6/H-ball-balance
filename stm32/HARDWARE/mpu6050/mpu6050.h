#ifndef __MPU6050_H__
#define __MPU6050_H__

#include "sys.h"

typedef struct {
    u8    online;
    u8    addr;
    float roll_deg;
    float pitch_deg;
    float roll_rate_dps;
    float pitch_rate_dps;
    float gyro_x_bias;
    float gyro_y_bias;
    float gyro_z_bias;
    float roll_zero_deg;
    float pitch_zero_deg;
} MPU6050_State;

/* PB8=SCL, PB9=SDA. Both lines must be pulled up to 3.3 V. */
u8 MPU6050_Init(MPU6050_State *m);
u8 MPU6050_SetZero(MPU6050_State *m, u16 samples);
u8 MPU6050_Update(MPU6050_State *m, float dt_s);

#endif
