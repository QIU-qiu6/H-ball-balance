#include "mpu6050.h"
#include "delay.h"
#include <math.h>
#include <string.h>

#define MPU_PORT              GPIOB
#define MPU_SCL_PIN           GPIO_Pin_8
#define MPU_SDA_PIN           GPIO_Pin_9
#define MPU_I2C_DELAY_US      3u

#define MPU_REG_SMPLRT_DIV    0x19u
#define MPU_REG_CONFIG        0x1Au
#define MPU_REG_GYRO_CONFIG   0x1Bu
#define MPU_REG_ACCEL_CONFIG  0x1Cu
#define MPU_REG_ACCEL_XOUT_H  0x3Bu
#define MPU_REG_PWR_MGMT_1    0x6Bu
#define MPU_REG_WHO_AM_I      0x75u

#define MPU_ADDR_LOW          0x68u
#define MPU_ADDR_HIGH         0x69u
#define DEG_PER_RAD           57.2957795f
#define COMPLEMENTARY_ALPHA   0.985f

static void I2C_Delay(void)
{
    Delay_us(MPU_I2C_DELAY_US);
}

static void SCL(u8 high)
{
    if (high) GPIO_SetBits(MPU_PORT, MPU_SCL_PIN);
    else GPIO_ResetBits(MPU_PORT, MPU_SCL_PIN);
}

static void SDA(u8 high)
{
    if (high) GPIO_SetBits(MPU_PORT, MPU_SDA_PIN);
    else GPIO_ResetBits(MPU_PORT, MPU_SDA_PIN);
}

static u8 SDA_Read(void)
{
    return GPIO_ReadInputDataBit(MPU_PORT, MPU_SDA_PIN) == Bit_SET ? 1u : 0u;
}

static void I2C_Start(void)
{
    SDA(1); SCL(1); I2C_Delay();
    SDA(0); I2C_Delay();
    SCL(0); I2C_Delay();
}

static void I2C_Stop(void)
{
    SDA(0); I2C_Delay();
    SCL(1); I2C_Delay();
    SDA(1); I2C_Delay();
}

static u8 I2C_WriteByte(u8 value)
{
    u8 bit;
    for (bit = 0; bit < 8u; bit++) {
        SDA((value & 0x80u) ? 1u : 0u);
        I2C_Delay(); SCL(1); I2C_Delay(); SCL(0); I2C_Delay();
        value <<= 1;
    }
    SDA(1); I2C_Delay(); SCL(1); I2C_Delay();
    bit = SDA_Read() ? 0u : 1u;
    SCL(0); I2C_Delay();
    return bit;
}

static u8 I2C_ReadByte(u8 ack)
{
    u8 bit, value = 0u;
    SDA(1);
    for (bit = 0; bit < 8u; bit++) {
        I2C_Delay(); SCL(1); I2C_Delay();
        value = (u8)((value << 1) | SDA_Read());
        SCL(0);
    }
    SDA(ack ? 0u : 1u); I2C_Delay();
    SCL(1); I2C_Delay(); SCL(0); I2C_Delay(); SDA(1);
    return value;
}

static u8 MPU_WriteReg(MPU6050_State *m, u8 reg, u8 value)
{
    u8 ok;
    I2C_Start();
    ok = I2C_WriteByte((u8)(m->addr << 1));
    if (ok) ok = I2C_WriteByte(reg);
    if (ok) ok = I2C_WriteByte(value);
    I2C_Stop();
    return ok;
}

static u8 MPU_ReadRegs(MPU6050_State *m, u8 reg, u8 *data, u8 len)
{
    u8 i, ok;
    I2C_Start();
    ok = I2C_WriteByte((u8)(m->addr << 1));
    if (ok) ok = I2C_WriteByte(reg);
    if (!ok) { I2C_Stop(); return 0; }
    I2C_Start();
    if (!I2C_WriteByte((u8)((m->addr << 1) | 1u))) {
        I2C_Stop(); return 0;
    }
    for (i = 0; i < len; i++) data[i] = I2C_ReadByte((i + 1u < len) ? 1u : 0u);
    I2C_Stop();
    return 1;
}

static int16_t ToS16(u8 high, u8 low)
{
    return (int16_t)(((u16)high << 8) | low);
}

static float Wrap180(float angle)
{
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

static void RawAngles(int16_t ax, int16_t ay, int16_t az,
                      float *roll_deg, float *pitch_deg)
{
    float fax = (float)ax;
    float fay = (float)ay;
    float faz = (float)az;
    *roll_deg = atan2f(fay, faz) * DEG_PER_RAD;
    *pitch_deg = atan2f(-fax, sqrtf(fay * fay + faz * faz)) * DEG_PER_RAD;
}

static u8 ReadRaw(MPU6050_State *m, int16_t *ax, int16_t *ay, int16_t *az,
                  int16_t *gx, int16_t *gy, int16_t *gz)
{
    u8 d[14];
    if (!MPU_ReadRegs(m, MPU_REG_ACCEL_XOUT_H, d, 14u)) return 0;
    *ax = ToS16(d[0], d[1]); *ay = ToS16(d[2], d[3]); *az = ToS16(d[4], d[5]);
    *gx = ToS16(d[8], d[9]); *gy = ToS16(d[10], d[11]); *gz = ToS16(d[12], d[13]);
    return 1;
}

u8 MPU6050_Init(MPU6050_State *m)
{
    GPIO_InitTypeDef gpio;
    u8 who = 0u;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    gpio.GPIO_Pin = MPU_SCL_PIN | MPU_SDA_PIN;
    gpio.GPIO_Mode = GPIO_Mode_Out_OD;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(MPU_PORT, &gpio);
    SCL(1); SDA(1);

    memset(m, 0, sizeof(*m));
    m->addr = MPU_ADDR_LOW;
    if (!MPU_ReadRegs(m, MPU_REG_WHO_AM_I, &who, 1u) ||
        (who != MPU_ADDR_LOW && who != MPU_ADDR_HIGH)) {
        m->addr = MPU_ADDR_HIGH;
        if (!MPU_ReadRegs(m, MPU_REG_WHO_AM_I, &who, 1u) ||
            (who != MPU_ADDR_LOW && who != MPU_ADDR_HIGH)) return 0;
    }

    if (!MPU_WriteReg(m, MPU_REG_PWR_MGMT_1, 0x80u)) return 0;
    Delay_ms(50u);
    if (!MPU_WriteReg(m, MPU_REG_PWR_MGMT_1, 0x01u) ||
        !MPU_WriteReg(m, MPU_REG_SMPLRT_DIV, 4u) ||
        !MPU_WriteReg(m, MPU_REG_CONFIG, 0x03u) ||
        !MPU_WriteReg(m, MPU_REG_GYRO_CONFIG, 0x00u) ||
        !MPU_WriteReg(m, MPU_REG_ACCEL_CONFIG, 0x00u)) return 0;
    m->online = 1;
    return 1;
}

u8 MPU6050_SetZero(MPU6050_State *m, u16 samples)
{
    u16 i, count = 0u;
    int16_t ax, ay, az, gx, gy, gz;
    float sax = 0.0f, say = 0.0f, saz = 0.0f;
    float sgx = 0.0f, sgy = 0.0f, sgz = 0.0f;

    if (!m->online || samples == 0u) return 0;
    for (i = 0; i < samples; i++) {
        if (ReadRaw(m, &ax, &ay, &az, &gx, &gy, &gz)) {
            sax += ax; say += ay; saz += az;
            sgx += gx; sgy += gy; sgz += gz;
            count++;
        }
        Delay_ms(2u);
    }
    if (count < (samples / 2u)) { m->online = 0; return 0; }

    m->gyro_x_bias = sgx / (float)count;
    m->gyro_y_bias = sgy / (float)count;
    m->gyro_z_bias = sgz / (float)count;
    RawAngles((int16_t)(sax / count), (int16_t)(say / count),
              (int16_t)(saz / count), &m->roll_zero_deg, &m->pitch_zero_deg);
    m->roll_deg = 0.0f;
    m->pitch_deg = 0.0f;
    m->roll_rate_dps = 0.0f;
    m->pitch_rate_dps = 0.0f;
    return 1;
}

u8 MPU6050_Update(MPU6050_State *m, float dt_s)
{
    int16_t ax, ay, az, gx, gy, gz;
    float acc_roll, acc_pitch;
    if (!m->online || dt_s <= 0.0f || !ReadRaw(m, &ax, &ay, &az, &gx, &gy, &gz)) {
        return 0;
    }
    RawAngles(ax, ay, az, &acc_roll, &acc_pitch);
    acc_roll = Wrap180(acc_roll - m->roll_zero_deg);
    acc_pitch = Wrap180(acc_pitch - m->pitch_zero_deg);
    m->roll_rate_dps = ((float)gx - m->gyro_x_bias) / 131.0f;
    m->pitch_rate_dps = ((float)gy - m->gyro_y_bias) / 131.0f;
    m->roll_deg = COMPLEMENTARY_ALPHA *
                  (m->roll_deg + m->roll_rate_dps * dt_s) +
                  (1.0f - COMPLEMENTARY_ALPHA) * acc_roll;
    m->pitch_deg = COMPLEMENTARY_ALPHA *
                   (m->pitch_deg + m->pitch_rate_dps * dt_s) +
                   (1.0f - COMPLEMENTARY_ALPHA) * acc_pitch;
    return 1;
}
