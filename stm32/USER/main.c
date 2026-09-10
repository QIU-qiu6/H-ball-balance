/* H-task single-axis steel-ball controller, STM32F103C8T6. */
#include "sys.h"
#include "stm32f10x.h"
#include "delay.h"
#include "uart1_ttl.h"
#include "uart3_ttl.h"
#include "zdt_proto.h"
#include "k230_proto.h"
#include "motor.h"
#include "gimbal.h"
#include "estop.h"
#include "oled.h"
#include <stdio.h>

#define KEY_COUNT                 5u
#define KEY_DEBOUNCE_SAMPLES      4u
#define KEY_SCAN_PERIOD_MS        5u
#define BALL_OUTER_PERIOD_MS       20u
#define MOTOR_FEEDBACK_PERIOD_MS   50u
#define PC_TELEMETRY_ENABLED        0u

/* 2026-08-01 VOFA 实测：按实体 K1 时 mode=4(MODE_CENTER_HOLD)，说明
 * 当前按键板的 K1/K2 与旧照片判断相反。按实机事件位重新对应。 */
#define KEY_Q3                    0x01u  /* 实体 K1 -> PB12 */
#define KEY_CENTER                0x02u  /* 实体 K2 -> PB13 */
#define KEY_CAPTURE               0x04u  /* PB14 */
#define KEY_LOCK                  0x08u  /* PB15 */
#define KEY_END                   0x10u  /* PA8  */

/* Board wiring: K1 is PB13, K2 is PB12.  Keep the legacy names below for
 * source compatibility, but override their event bits to match the PCB. */
#undef KEY_Q3
#undef KEY_CENTER
#define KEY_Q3                    0x02u
#define KEY_CENTER                0x01u

#define CAPTURE_SAMPLE_COUNT      10u

/* 第3项：静止小车上，钢球由 O 点到 +5cm，再折返到 -5cm。
 * +5cm 只要求到达后立即折返；-5cm 需进入比赛允许的 +/-1cm 区域并
 * 低速保持一小段时间，才记为完成。 */
/* 2026-08-01 physical-mark calibration:
 * track +5 cm -> OLED B=-49.6 mm, track -5 cm -> OLED B=+55.3 mm.
 * The physical sign is therefore opposite to the controller's B sign. */
#define Q3_PLUS_TARGET_MM         (-49.6f)
#define Q3_MINUS_TARGET_MM          55.3f
#define Q3_START_TOLERANCE_MM      10.0f
#define Q3_TURN_TOLERANCE_MM        5.0f
#define Q3_FINISH_TOLERANCE_MM     10.0f
#define Q3_FINISH_SPEED_MM_S        8.0f
#define Q3_FINISH_DWELL_MS        250u
#define Q3_TIME_LIMIT_MS         5000u

typedef struct {
    GPIO_TypeDef *port;
    u16 pin;
    u8 stable_pressed;
    u8 last_raw_pressed;
    u8 same_count;
} KeyInput;

typedef enum {
    MODE_MENU = 0,
    MODE_Q3_TO_PLUS5,
    MODE_Q3_TO_MINUS5,
    MODE_Q3_HOLD_MINUS5,
    MODE_CENTER_HOLD,
    MODE_FREE_CAPTURING,
    MODE_FREE_READY,
    MODE_FREE_HOLD,
    MODE_RETURN_ZERO
} BalanceMode;

volatile uint32_t g_sys_tick = 0;

static ZDT_Port g_motor_port;
static Motor_State g_motor;
static BallBalance_System g_balance;
static KeyInput g_keys[KEY_COUNT];

static u8 g_key_events = 0;
static u8 g_motor_ok = 0;
static u8 g_estop_handled = 0;
static u8 g_fault_latched = 0;
static u8 g_oled_ok = 0;
static u8 g_motor_diag = 0;
static u8 g_pos_fail = 0;
static u8 g_fault_code = 0;
static u8 g_last_key_event = 0;
static u8 g_key_reject = 0;

static BalanceMode g_mode = MODE_MENU;
static float g_capture_sum_px = 0.0f;
static float g_free_target_px = BALL_POS_ZERO_PX;
static u8 g_capture_count = 0;
static u8 g_saved_target_valid = 0;
static u32 g_capture_last_frame = 0;
static u32 g_q3_start_tick = 0;
static u32 g_q3_finish_enter_tick = 0;
static u32 g_q3_result_ms = 0;
static u8 g_q3_timeout = 0;

#if PC_TELEMETRY_ENABLED
static const char *const g_telemetry_names[56] = {
    "B", "T", "E", "V", "RPM", "A", "G", "Graw", "Mode",
    "X", "TX", "CX", "Kp", "Ki", "Kd", "KdEff", "IState",
    "IAng", "Inertia", "Step", "Stall", "Dip", "ARate",
    "DriveLim", "BrakeLim", "Friction", "DynZone", "MaxRPM",
    "Ctrl", "Vision", "RangeErr", "MotorErr", "Tracking", "Frame",
    "Time", "IDecay", "Pterm", "Vterm", "Iterm", "FfTerm",
    "ProfileV", "ProfileBrake", "TerminalRet", "EarlyGuard", "PreLimit",
    "TiltLim", "AngleErr", "InnerP", "InnerD", "InnerRaw", "InnerSat",
    "OuterSat", "ZeroPx", "ZeroMm", "TargetOffset", "CrossBrake"
};
#endif

void SysTick_Handler(void)
{
    g_sys_tick++;
    K230_Tick1ms();
    Estop_Tick1ms();
}

static void SysTick_Init(void)
{
    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);
    SysTick->LOAD = 72000 - 1;
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
                    SysTick_CTRL_TICKINT_Msk |
                    SysTick_CTRL_ENABLE_Msk;
}

static void NVIC_Config(void)
{
    NVIC_InitTypeDef n;

    n.NVIC_IRQChannel = USART2_IRQn;
    n.NVIC_IRQChannelPreemptionPriority = 1;
    n.NVIC_IRQChannelSubPriority = 0;
    n.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&n);

    n.NVIC_IRQChannel = USART3_IRQn;
    n.NVIC_IRQChannelPreemptionPriority = 1;
    n.NVIC_IRQChannelSubPriority = 1;
    n.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&n);

#if PC_TELEMETRY_ENABLED
    n.NVIC_IRQChannel = USART1_IRQn;
    n.NVIC_IRQChannelPreemptionPriority = 2;
    n.NVIC_IRQChannelSubPriority = 0;
    n.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&n);
#endif
}

static u8 KeyRawPressed(const KeyInput *key)
{
    return (GPIO_ReadInputDataBit(key->port, key->pin) == Bit_RESET) ? 1u : 0u;
}

static void Keys_Init(void)
{
    GPIO_InitTypeDef gpio;
    u8 i;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |
                           RCC_APB2Periph_GPIOB, ENABLE);

    gpio.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 |
                    GPIO_Pin_14 | GPIO_Pin_15;
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    gpio.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOB, &gpio);

    gpio.GPIO_Pin = GPIO_Pin_8;
    GPIO_Init(GPIOA, &gpio);

    g_keys[0].port = GPIOB; g_keys[0].pin = GPIO_Pin_12;
    g_keys[1].port = GPIOB; g_keys[1].pin = GPIO_Pin_13;
    g_keys[2].port = GPIOB; g_keys[2].pin = GPIO_Pin_14;
    g_keys[3].port = GPIOB; g_keys[3].pin = GPIO_Pin_15;
    g_keys[4].port = GPIOA; g_keys[4].pin = GPIO_Pin_8;

    for (i = 0; i < KEY_COUNT; i++) {
        g_keys[i].stable_pressed = KeyRawPressed(&g_keys[i]);
        g_keys[i].last_raw_pressed = g_keys[i].stable_pressed;
        g_keys[i].same_count = 0;
    }
    g_key_events = 0;
}

static void Keys_Scan(void)
{
    u8 i;
    u8 raw;

    for (i = 0; i < KEY_COUNT; i++) {
        raw = KeyRawPressed(&g_keys[i]);
        if (raw == g_keys[i].last_raw_pressed) {
            if (g_keys[i].same_count < KEY_DEBOUNCE_SAMPLES) {
                g_keys[i].same_count++;
            }
        } else {
            g_keys[i].last_raw_pressed = raw;
            g_keys[i].same_count = 1;
        }

        if (g_keys[i].same_count >= KEY_DEBOUNCE_SAMPLES &&
            g_keys[i].stable_pressed != raw) {
            g_keys[i].stable_pressed = raw;
            if (raw) g_key_events |= (u8)(1u << i);
        }
    }
}

static u8 Keys_GetEvents(void)
{
    u8 events = g_key_events;
    g_key_events = 0;
    return events;
}

static u8 VisionValid(void)
{
    return (K230_LinkAlive() && g_balance.target.has_target &&
            !g_balance.range_fault) ? 1u : 0u;
}

static void SetMode(BalanceMode mode)
{
    g_mode = mode;
}

static void SetTargetPxAndRun(float target_px)
{
    BallBalance_SetTargetPx(&g_balance, target_px);
    if (!g_balance.control_enabled) {
        BallBalance_SetControlEnabled(&g_balance, 1);
    }
}

static float AbsFloat(float value)
{
    return (value < 0.0f) ? -value : value;
}

static u8 ReachedDirectionalTarget(float position_mm, float target_mm,
                                   float tolerance_mm)
{
    if (target_mm >= 0.0f) {
        return (position_mm >= target_mm - tolerance_mm) ? 1u : 0u;
    }
    return (position_mm <= target_mm + tolerance_mm) ? 1u : 0u;
}

static void HandleKeyEvents(u8 events)
{
    if (!events) return;

    /* K2 (PB12) and K5 (PA8) are unconditional stop/menu keys.  Handle them
     * before camera-validity checks, so a lost frame can never block stop. */
    if (events & (KEY_CENTER | KEY_END)) {
        g_key_reject = 0;
        BallBalance_SetControlEnabled(&g_balance, 0);
        g_capture_sum_px = 0.0f;
        g_capture_count = 0u;
        g_saved_target_valid = 0u;
        g_q3_finish_enter_tick = 0u;
        SetMode(MODE_MENU);
        return;
    }

    /* PB14/PB15 are intentionally unused in the simplified key interface. */
    if (events & (KEY_CAPTURE | KEY_LOCK)) {
        return;
    }

    g_last_key_event = events;
    if (!g_motor_ok) {
        g_key_reject = 1;  /* 电机未完成初始化 */
        return;
    }
    if (g_fault_latched) {
        g_key_reject = 2;  /* 系统故障已锁存 */
        return;
    }
    if (Estop_IsLatched()) {
        g_key_reject = 3;  /* PA0 急停已锁存 */
        return;
    }
    /* 启动第3项、回中心和记录坐标都需要有效钢珠坐标；停止键在
     * 摄像头丢失时也必须始终可用。 */
    if ((events & (KEY_Q3 | KEY_CENTER | KEY_CAPTURE | KEY_LOCK)) &&
        !VisionValid()) {
        g_key_reject = 4;  /* 摄像头或钢珠坐标尚未有效 */
        return;
    }
    g_key_reject = 0;

    /* 基础联调版：实体 K2 或 K5 都立即退出保持并停电机。 */
    if (events & KEY_END) {
        BallBalance_SetControlEnabled(&g_balance, 0);
        g_capture_sum_px = 0.0f;
        g_capture_count = 0;
        g_q3_finish_enter_tick = 0;
        SetMode(MODE_MENU);
        return;
    }

    /* K2：基础联调时作为停止键。 */
    if (events & KEY_CENTER) {
        BallBalance_SetControlEnabled(&g_balance, 0);
        g_capture_sum_px = 0.0f;
        g_capture_count = 0u;
        g_saved_target_valid = 0u;
        g_q3_finish_enter_tick = 0;
        SetMode(MODE_MENU);
        return;
    }

    /* K1：记录当前钢球位置，随后立即在该点保持。 */
    if (events & KEY_Q3) {
        /* This build has one control objective: calibrated physical zero. */
        BallBalance_SetTargetMm(&g_balance, BALL_TARGET_MM);
        BallBalance_SetControlEnabled(&g_balance, 1);
        g_q3_finish_enter_tick = 0;
        g_capture_sum_px = 0.0f;
        g_capture_count = 0u;
        g_saved_target_valid = 0u;
        SetMode(MODE_CENTER_HOLD);
        return;
    }

    /* K3/K4 保留原来的“记录任意点/运行到任意点”功能，便于后续
     * 第6项继续使用，同时不会占用第3项的 K1。 */
    if (events & KEY_LOCK) {
        if (!g_saved_target_valid) {
            g_key_reject = 6;
            return;
        }
        SetTargetPxAndRun(g_free_target_px);
        SetMode(MODE_FREE_HOLD);
        return;
    }

    if (events & KEY_CAPTURE) {
        BallBalance_SetControlEnabled(&g_balance, 0);
        g_capture_sum_px = 0.0f;
        g_capture_count = 0;
        g_saved_target_valid = 0;
        g_capture_last_frame = g_balance.frame_counter;
        SetMode(MODE_FREE_CAPTURING);
        return;
    }
}

static void UpdateMode(u32 tick)
{
    switch (g_mode) {
    case MODE_Q3_TO_PLUS5:
        if ((u32)(tick - g_q3_start_tick) > Q3_TIME_LIMIT_MS) {
            g_q3_timeout = 1u;
        }
        /* 到达 +5cm 前 5mm 即开始折返。控制器仍保持使能，只更新目标，
         * 因此不会再次执行开环脱困脉冲。 */
        if (ReachedDirectionalTarget(g_balance.ball_mm,
                                     Q3_PLUS_TARGET_MM,
                                     Q3_TURN_TOLERANCE_MM)) {
            BallBalance_SetTargetMm(&g_balance, Q3_MINUS_TARGET_MM);
            SetMode(MODE_Q3_TO_MINUS5);
        }
        break;

    case MODE_Q3_TO_MINUS5:
        if ((u32)(tick - g_q3_start_tick) > Q3_TIME_LIMIT_MS) {
            g_q3_timeout = 1u;
        }
        if (AbsFloat(g_balance.error_mm) <= Q3_FINISH_TOLERANCE_MM &&
            AbsFloat(g_balance.ball_velocity_mm_s) <=
                Q3_FINISH_SPEED_MM_S) {
            if (g_q3_finish_enter_tick == 0u) {
                g_q3_finish_enter_tick = tick;
            } else if ((u32)(tick - g_q3_finish_enter_tick) >=
                       Q3_FINISH_DWELL_MS) {
                g_q3_result_ms = (u32)(tick - g_q3_start_tick);
                if (g_q3_result_ms > Q3_TIME_LIMIT_MS) g_q3_timeout = 1u;
                SetMode(MODE_Q3_HOLD_MINUS5);
            }
        } else {
            g_q3_finish_enter_tick = 0u;
        }
        break;

    case MODE_FREE_CAPTURING:
        if (VisionValid()) {
            if (g_capture_last_frame == g_balance.frame_counter) break;
            g_capture_last_frame = g_balance.frame_counter;
            g_capture_sum_px += BallBalance_GetAxisPx(&g_balance);
            if (g_capture_count < CAPTURE_SAMPLE_COUNT) g_capture_count++;
            if (g_capture_count >= CAPTURE_SAMPLE_COUNT) {
                g_free_target_px = g_capture_sum_px /
                                   (float)CAPTURE_SAMPLE_COUNT;
                BallBalance_SetTargetPx(&g_balance, g_free_target_px);
                g_saved_target_valid = 1u;
                SetTargetPxAndRun(g_free_target_px);
                SetMode(MODE_FREE_HOLD);
            }
        } else {
            g_capture_sum_px = 0.0f;
            g_capture_count = 0;
            g_capture_last_frame = g_balance.frame_counter;
        }
        break;

    default:
        break;
    }
}

#if 0 /* full Chinese/status page retained as reference, not used at runtime */
static OLED_ZhText CurrentChineseStatus(void)
{
    if (Estop_IsLatched()) return OLED_ZH_ESTOP;
    if (g_fault_latched || !g_motor_ok) return OLED_ZH_FAULT;
    if (!VisionValid() && g_mode != MODE_MENU) return OLED_ZH_BALL_LOST;

    switch (g_mode) {
    case MODE_Q3_TO_PLUS5:
    case MODE_Q3_TO_MINUS5:      return OLED_ZH_Q3;
    case MODE_Q3_HOLD_MINUS5:    return OLED_ZH_HOLD;
    case MODE_CENTER_HOLD:       return OLED_ZH_CENTER_HOLD;
    case MODE_FREE_CAPTURING:    return OLED_ZH_CAPTURE;
    case MODE_FREE_READY:        return OLED_ZH_CAPTURED;
    case MODE_FREE_HOLD:         return OLED_ZH_FREE_HOLD;
    case MODE_RETURN_ZERO:       return OLED_ZH_RETURN_ZERO;
    case MODE_MENU:
    default:                     return OLED_ZH_MENU;
    }
}
#endif

static void ShowStatus(void)
{
    char line[24];
    float ball_mm, target_mm, error_mm, command_rpm;

    if (!g_oled_ok) return;
    /* Keep diagnostics available to the debugger without rendering them. */
    (void)g_motor_diag;
    (void)g_fault_code;
    (void)g_last_key_event;
    (void)g_key_reject;
    (void)g_q3_timeout;
    BallBalance_GetStatus(&g_balance, &ball_mm, &target_mm,
                          &error_mm, &command_rpm);

    /* Keep the display path deliberately small: the control loop and UART
     * telemetry must not wait for multiple Chinese/status lines. */
    OLED_Clear();
    sprintf(line, "B:%+.1f", ball_mm);
    OLED_ShowLine(2, line);
    sprintf(line, "T:%+.1f", target_mm);
    OLED_ShowLine(4, line);
    if (!OLED_Refresh()) g_oled_ok = 0;
}

int main(void)
{
    uint32_t last_keys, last_control, last_motor_read, last_inner;
    uint32_t last_oled, arm_tick, tick;
#if PC_TELEMETRY_ENABLED
    uint32_t last_log;
#endif
    u8 ret;

    Estop_Init();
    Keys_Init();
    SysTick_Init();
    Delay_Init();
    NVIC_Config();
#if PC_TELEMETRY_ENABLED
    UART1_TTL_Init(115200);
#endif
    UART3_TTL_Init(115200);
    K230_Init(115200);
    g_oled_ok = OLED_Init();
    Delay_ms(50);
    K230_SendString("\r\nBALL_BALANCE_KEYS_BOOT\r\n");

    g_motor_port.send = UART3_SendBytes;
    g_motor_port.flush = UART3_Flush;
    g_motor_port.recv = UART3_RecvBytes;
    g_motor_port.addr = 0x01;
    g_motor_port.last_rx_len = 0;
    g_motor_port.last_error = 0;
    /* 当前电机已通过角度工程确认使用固件 X，禁止再次自动探测。 */
    /* The current X42S has been confirmed as Emm firmware. */
    g_motor_port.firmware = ZDT_FW_EMM;
    g_motor_port.options = 0;
    Motor_Init(&g_motor, &g_motor_port);
    BallBalance_Init(&g_balance, &g_motor);

    if (!Estop_IsLatched()) {
        /* The Emm upper-computer has already verified this motor. Do not use
         * 0x1A at boot: some Emm releases do not expose the option bits used
         * by that legacy auto-detection path. */
        ret = Motor_Enable(&g_motor);
        g_motor_diag = ret;
        if (ret == 0) {
            ret = Motor_HomeSingleTurn(&g_motor, MOTOR_HOME_TIMEOUT_MS);
            g_motor_diag = ret;
        }
        if (ret == 0) {
            ret = Motor_ReadPosition(&g_motor);
            g_motor_diag = ret;
            if (ret == 0) {
                /* Synchronize the STM32 software angle with physical home. */
                ret = Motor_SetZero(&g_motor);
                g_motor_diag = ret;
            }
            if (ret == 0) {
                BallBalance_SetControlEnabled(&g_balance, 0);
                g_motor_ok = 1;
                K230_SendString("MOTOR:EMM:HOME_OK\r\n");
            }
        }
    }

    if (!g_motor_ok) {
        if (Estop_IsLatched()) {
            g_estop_handled = 1;
            K230_SendString("ESTOP_AT_BOOT\r\n");
        } else {
            g_fault_latched = 1;
            g_fault_code = 1;
            K230_SendString("MOTOR_INIT_FAIL\r\n");
        }
    } else {
        K230_SendString("BALL_BALANCE_KEYS_READY\r\n");
    }

    last_keys = last_control = last_motor_read = last_inner = g_sys_tick;
    last_oled = g_sys_tick;
#if PC_TELEMETRY_ENABLED
    last_log = g_sys_tick;
#endif
    arm_tick = g_sys_tick;

    while (1) {
        tick = g_sys_tick;

        if ((u32)(tick - last_keys) >= KEY_SCAN_PERIOD_MS) {
            last_keys = tick;
            Keys_Scan();
            HandleKeyEvents(Keys_GetEvents());
        }

        if (Estop_IsLatched() && !g_estop_handled) {
            BallBalance_EmergencyStop(&g_balance);
            g_motor_ok = 0;
            g_estop_handled = 1;
            g_fault_latched = 1;
            g_fault_code = 2;
            K230_SendString("ESTOP\r\n");
        }

        if ((u32)(tick - last_inner) >= 5u) {
            last_inner = tick;
            if (g_motor_ok && !g_fault_latched && !Estop_IsLatched() &&
                (u32)(tick - arm_tick) >= 1000u) {
                BallBalance_InnerUpdate(&g_balance);
            }
        }

        if ((u32)(tick - last_control) >= BALL_OUTER_PERIOD_MS) {
            last_control = tick;
            if (g_motor_ok && !g_fault_latched && !Estop_IsLatched() &&
                (u32)(tick - arm_tick) >= 1000u) {
                BallBalance_Update(&g_balance,
                                   (float)BALL_OUTER_PERIOD_MS * 0.001f);
                UpdateMode(tick);
            }
        }

        if ((u32)(tick - last_motor_read) >= MOTOR_FEEDBACK_PERIOD_MS) {
            last_motor_read = tick;
            if (g_motor_ok && !Estop_IsLatched()) {
                ret = Motor_ReadPosition(&g_motor);
                g_motor_diag = ret;
                if (ret == 0) g_pos_fail = 0;
                else if (g_pos_fail < 255u) g_pos_fail++;
                if (g_pos_fail >= 5u) {
                    BallBalance_EmergencyStop(&g_balance);
                    g_motor_ok = 0;
                    g_fault_latched = 1;
                    g_fault_code = 3;
                    K230_SendString("MOTOR_OFFLINE\r\n");
                }
            }
        }

        /* OLED is a human interface only; refresh at 2 Hz so it cannot
         * become the pacing element of the 10 ms vision/control loop. */
        if ((u32)(tick - last_oled) >= 500u) {
            last_oled = tick;
            ShowStatus();
        }

#if PC_TELEMETRY_ENABLED
        if ((u32)(tick - last_log) >= 100u) {
            float vofa[56];
            float ball_mm, target_mm, error_mm, command_rpm;
            float integral_angle_deg;
            last_log = tick;
            BallBalance_GetStatus(&g_balance, &ball_mm, &target_mm,
                                  &error_mm, &command_rpm);
            integral_angle_deg =
                (BALL_STICTION_KI_DEG_PER_MM_S + g_balance.pid.ki) *
                g_balance.stiction_integral_mm_s;
            if (integral_angle_deg > BALL_STALL_TILT_MAX_DEG)
                integral_angle_deg = BALL_STALL_TILT_MAX_DEG;
            if (integral_angle_deg < -BALL_STALL_TILT_MAX_DEG)
                integral_angle_deg = -BALL_STALL_TILT_MAX_DEG;

            /* Fixed plain ASCII telemetry field table: VOFA_PA9_CHANNELS.md. */
            vofa[0] = ball_mm;
            vofa[1] = target_mm;
            vofa[2] = error_mm;
            vofa[3] = g_balance.ball_velocity_mm_s;
            vofa[4] = command_rpm;
            vofa[5] = g_balance.track_angle_deg;
            vofa[6] = g_balance.desired_angle_deg;
            vofa[7] = g_balance.raw_desired_angle_deg;
            vofa[8] = (float)g_mode;
            vofa[9] = g_balance.filtered_axis_px;
            vofa[10] = BallBalance_MmToPixel(target_mm);
            vofa[11] = BALL_AXIS_CENTER_PX;
            vofa[12] = g_balance.pid.kp;
            vofa[13] = g_balance.pid.ki;
            vofa[14] = g_balance.pid.kd;
            vofa[15] = g_balance.effective_kd;
            vofa[16] = g_balance.stiction_integral_mm_s;
            vofa[17] = integral_angle_deg;
            vofa[18] = g_balance.inertia_compensation_deg;
            vofa[19] = g_balance.output_step_limit_deg;
            vofa[20] = g_balance.stall_boost_deg;
            vofa[21] = (float)g_balance.dip_escape_active;
            vofa[22] = g_balance.track_rate_dps;
            vofa[23] = g_balance.active_drive_limit_deg;
            vofa[24] = g_balance.active_brake_limit_deg;
            vofa[25] = BALL_FRICTION_FF_TILT_DEG;
            vofa[26] = BALL_DYNAMIC_ZONE_MM;
            vofa[27] = BALL_MOTOR_MAX_RPM;
            vofa[28] = (float)g_balance.control_enabled;
            vofa[29] = (float)g_balance.target.has_target;
            vofa[30] = (float)g_balance.range_fault;
            vofa[31] = (float)g_balance.motor_error;
            vofa[32] = (float)g_balance.tracking;
            vofa[33] = (float)g_balance.frame_counter;
            vofa[34] = (float)tick * 0.001f;
            vofa[35] = BALL_STICTION_I_DECAY_PER_S;
            vofa[36] = g_balance.position_term_deg;
            vofa[37] = g_balance.velocity_term_deg;
            vofa[38] = g_balance.stiction_term_deg;
            vofa[39] = g_balance.friction_term_deg;
            vofa[40] = g_balance.profile_speed_mm_s;
            vofa[41] = (float)g_balance.profile_braking;
            vofa[42] = (float)g_balance.terminal_return_active;
            vofa[43] = (float)g_balance.early_return_guard;
            vofa[44] = g_balance.pre_limit_desired_angle_deg;
            vofa[45] = g_balance.tilt_limit_deg;
            vofa[46] = g_balance.angle_error_deg;
            vofa[47] = g_balance.inner_p_rpm;
            vofa[48] = g_balance.inner_d_rpm;
            vofa[49] = g_balance.inner_speed_raw_rpm;
            vofa[50] = (float)g_balance.inner_output_saturated;
            vofa[51] = (float)g_balance.outer_output_saturated;
            vofa[52] = BALL_POS_ZERO_PX;
            vofa[53] = BALL_TARGET_MM;
            vofa[54] = target_mm - BALL_TARGET_MM;
            vofa[55] = (float)g_balance.crossing_brake_active;
            UART1_SendAsciiFrame(g_telemetry_names, vofa, 56u);
        }
#endif
    }
}
