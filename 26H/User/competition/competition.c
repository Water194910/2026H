#include "competition.h"

#include "main.h"
#include "gm6020_can.h"
#include "motor.h"
#include "oled.h"
#include "qiu_weizhi.h"
#include "vision_uart.h"

#include <stdio.h>
#include <string.h>

extern volatile uint8_t dianji_shineng;
extern volatile uint8_t jiaodu_moshi;

#define COMP_H2_SPEED_RPM             230.0f
#define COMP_H2_SLOW_SPEED_RPM        120.0f
#define COMP_H2_SLOW_COUNTS             19050L
#define COMP_H2_LAP_COUNTS              20492L
#define COMP_H4_SPEED_RPM             200.0f
#define COMP_H4_CRAWL_SPEED_RPM         40.0f
#define COMP_H4_DECEL_COUNTS             5500L
#define COMP_H4_AB_COUNTS                6500L
#define COMP_H4_MANUAL_CALIBRATION          0U
#define COMP_H4_RAMP_TIME_MS             2400U
#define COMP_H4_DECEL_TIME_MS             2000U
#define COMP_H5_SPEED_RPM             150.0f
#define COMP_H6_SPEED_RPM             170.0f
#define COMP_RAMP_TIME_MS             3000U
/* 按各任务smoothstep峰值加速度标定命令前馈：H4=5.5deg，H5/H6=4.0deg。 */
#define COMP_H4_CMD_FF_GAIN_DEG_PER_RPM_S  0.0440000f
#define COMP_H5_CMD_FF_GAIN_DEG_PER_RPM_S  0.0533333f
#define COMP_H6_CMD_FF_GAIN_DEG_PER_RPM_S  0.0470588f
#define COMP_PREHOLD_MS                300U
#define COMP_FINISH_ARM_MS           10000U
#define COMP_MARKER_LEAVE_SAMPLES       20U
#define COMP_MARKER_CLEAR_SAMPLES        2U
#define COMP_BRAKE_PWM                  20
#define COMP_BRAKE_STOP_RPM             12.0f
#define COMP_BRAKE_TIMEOUT_MS           120U
#define COMP_STOP_CONFIRM_TICKS           3U
#define COMP_H5_STOP_FF_TAIL_DEG          2.0f
#define COMP_H5_STOP_FF_RPM              12.0f
#define COMP_H5_STOP_FF_CONFIRM_TICKS      3U
#define COMP_H5_STOP_FF_RELEASE_MS       200U
#define COMP_BALL_PAUSE_MM               8.0f
#define COMP_BALL_RESUME_MM              5.0f
#define COMP_BALL_RESUME_TICKS            5U
#define COMP_RAMP_FORCE_RPM              40.0f
#define COMP_H3_POSITIVE_MM              50.0f
#define COMP_H3_CENTER_MM                 0.0f
#define COMP_H3_NEGATIVE_MM             -50.0f
/* 判定窗放到8mm：题目只要求±5cm处误差≤1cm，5mm窗等于自己把余量
 * 砍掉一半，负向靠脱困脉冲爬最后几毫米时很容易卡在窗外白等。 */
#define COMP_H3_WINDOW_MM                 8.0f
#define COMP_H3_SPEED_MM_S               30.0f
#define COMP_H3_POSITIVE_STABLE_MS       200U
#define COMP_H3_CENTER_STABLE_MS         120U
#define COMP_H3_NEGATIVE_STABLE_MS       300U
#define COMP_H3_POSITIVE_MIN_SAMPLES      10U
#define COMP_H3_CENTER_MIN_SAMPLES          6U
#define COMP_H3_NEGATIVE_MIN_SAMPLES      15U
#define COMP_H3_STUCK_TILT_DEG             3.4f
#define COMP_H3_NEGATIVE_STUCK_TILT_DEG    6.0f
#define COMP_H6_READY_WINDOW_MM             8.0f
#define COMP_H6_FAR_TARGET_MM              70.0f
#define COMP_H6_FAR_READY_WINDOW_MM        15.0f
#define COMP_H6_READY_MS                 300U
#define COMP_H6_READY_MIN_SAMPLES         15U
#define COMP_OLED_PERIOD_MS              100U

typedef enum
{
    COMP_TASK_H2 = 2,
    COMP_TASK_H3 = 3,
    COMP_TASK_H4 = 4,
    COMP_TASK_H5 = 5,
    COMP_TASK_H6 = 6
} Competition_Task;

typedef enum
{
    COMP_STATE_MENU = 0,
    COMP_STATE_PREHOLD,
    COMP_STATE_RUNNING,
    COMP_STATE_H3_POSITIVE,
    COMP_STATE_H3_CENTER,
    COMP_STATE_H3_NEGATIVE,
    COMP_STATE_CROSSING_FINISH,
    COMP_STATE_BRAKING,
    COMP_STATE_DECELERATING,
    COMP_STATE_DONE,
    COMP_STATE_ABORTED
} Competition_State;

typedef struct
{
    GPIO_TypeDef *port;
    uint16_t pin;
    GPIO_PinState raw;
    GPIO_PinState stable;
    uint32_t changed_ms;
    uint8_t pressed_event;
} Competition_Key;

enum
{
    COMP_KEY_PREVIOUS = 0,
    COMP_KEY_NEXT,
    COMP_KEY_START,
    COMP_KEY_STOP,
    COMP_KEY_COUNT
};

static Competition_Key comp_keys[COMP_KEY_COUNT];
static volatile Competition_Task comp_task = COMP_TASK_H2;
static volatile Competition_State comp_state = COMP_STATE_MENU;

static volatile uint32_t comp_start_ms = 0U;
static volatile uint32_t comp_locked_time_ms = 0U;
static volatile uint8_t comp_result_locked = 0U;
static volatile float comp_max_error_mm = 0.0f;
static volatile float comp_locked_max_error_mm = 0.0f;
static volatile float comp_track_speed_rpm = 0.0f;
static volatile uint8_t comp_oled_force = 1U;

static volatile uint8_t comp_left_start_line = 0U;
static volatile uint8_t comp_leave_samples = 0U;
static volatile uint8_t comp_clear_samples = 0U;
static volatile uint8_t comp_timing_started = 0U;

/* H2 正式运行：首次横线建立基准，第二次横线或圈长兜底触发停车。 */
static volatile int32_t comp_h2_left_counts = 0;
static volatile int32_t comp_h2_right_counts = 0;
static volatile int32_t comp_h2_latched_left = 0;
static volatile int32_t comp_h2_latched_right = 0;
static volatile int32_t comp_h2_latched_center = 0;
static volatile uint8_t comp_h2_counting = 0U;
static volatile uint8_t comp_h2_wait_clear = 0U;
static volatile uint8_t comp_h2_report_pending = 0U;
static volatile uint8_t comp_h2_finish_by_encoder = 0U;
static volatile uint32_t comp_h2_trigger_time_ms = 0U;

/* H4正式运行：A横线自动清零并直接起步，B点前平滑减速，通过B后自然停稳。 */
static volatile int32_t comp_h4_left_counts = 0;
static volatile int32_t comp_h4_right_counts = 0;
static volatile int32_t comp_h4_latched_left = 0;
static volatile int32_t comp_h4_latched_right = 0;
static volatile int32_t comp_h4_latched_center = 0;
static volatile uint8_t comp_h4_counting = 0U;
static volatile uint8_t comp_h4_passed = 0U;
static volatile uint8_t comp_h4_report_pending = 0U;
static volatile uint32_t comp_h4_trigger_time_ms = 0U;
static volatile uint32_t comp_h4_run_start_ms = 0U;
static volatile uint16_t comp_h4_cal_sample_count = 0U;
static volatile uint16_t comp_h4_latched_sample = 0U;

static volatile float comp_last_left_rpm = 0.0f;
static volatile float comp_last_right_rpm = 0.0f;
static volatile uint32_t comp_brake_start_ms = 0U;
static volatile int8_t comp_brake_left_direction = 1;
static volatile int8_t comp_brake_right_direction = 1;
static volatile uint8_t comp_brake_left_active = 0U;
static volatile uint8_t comp_brake_right_active = 0U;
static volatile uint8_t comp_stop_confirm_ticks = 0U;

static volatile float comp_ramp_progress = 0.0f;
static volatile float comp_decel_start_rpm = 0.0f;
static volatile uint8_t comp_ramp_paused = 0U;
static volatile uint8_t comp_ramp_resume_ticks = 0U;
static volatile float comp_h5_stop_ff_floor_deg = 0.0f;
static volatile uint8_t comp_h5_stop_ff_confirm_ticks = 0U;
static volatile uint8_t comp_h5_stop_ff_releasing = 0U;
static volatile uint8_t comp_h5_stop_ff_complete = 0U;
static volatile uint32_t comp_h5_stop_ff_release_start_ms = 0U;

static float comp_h3_positive_sum = 0.0f;
static float comp_h3_negative_sum = 0.0f;
static uint16_t comp_h3_stable_samples = 0U;
static uint32_t comp_h3_stable_start_ms = 0U;
static float comp_h3_positive_position_mm = 0.0f;
static float comp_h3_negative_position_mm = 0.0f;
static uint8_t comp_h3_positive_valid = 0U;
static uint8_t comp_h3_negative_valid = 0U;

static float comp_h6_locked_target_mm = 0.0f;
static float comp_h6_idle_target_mm = 1000.0f;
static uint32_t comp_h6_ready_start_ms = 0U;
static uint16_t comp_h6_ready_samples = 0U;

static uint32_t comp_last_vision_rx_count = 0U;
static uint32_t comp_last_oled_ms = 0U;
static uint8_t comp_last_oled_page = 0xFFU;
static char comp_menu_message[22] = "";
static uint32_t comp_menu_message_until_ms = 0U;

static float comp_absf(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static int32_t comp_abs_i32(int32_t value)
{
    return (value >= 0) ? value : -value;
}

static float comp_smoothstep(float progress)
{
    if (progress <= 0.0f)
    {
        return 0.0f;
    }
    if (progress >= 1.0f)
    {
        return 1.0f;
    }
    return progress * progress * (3.0f - 2.0f * progress);
}

static uint8_t comp_is_active(void)
{
    return comp_state != COMP_STATE_MENU &&
           comp_state != COMP_STATE_DONE &&
           comp_state != COMP_STATE_ABORTED;
}

static uint32_t comp_display_time_ms(uint32_t now)
{
    if ((comp_task == COMP_TASK_H2 ||
         comp_task == COMP_TASK_H5 || comp_task == COMP_TASK_H6) &&
        !comp_timing_started)
    {
        return 0U;
    }
    if (comp_result_locked)
    {
        return comp_locked_time_ms;
    }
    if (comp_state == COMP_STATE_MENU)
    {
        return 0U;
    }
    return now - comp_start_ms;
}

static void comp_set_message(const char *message, uint32_t now)
{
    (void)snprintf(comp_menu_message, sizeof(comp_menu_message), "%-21.21s", message);
    comp_menu_message_until_ms = now + 1500U;
    comp_oled_force = 1U;
}

static void comp_reset_marker(void)
{
    comp_left_start_line = 0U;
    comp_leave_samples = 0U;
    comp_clear_samples = 0U;
    comp_timing_started = 0U;
}

static void comp_reset_ramp(void)
{
    comp_ramp_progress = 0.0f;
    comp_decel_start_rpm = 0.0f;
    comp_ramp_paused = 0U;
    comp_ramp_resume_ticks = 0U;
    comp_track_speed_rpm = 0.0f;
    comp_h5_stop_ff_floor_deg = 0.0f;
    comp_h5_stop_ff_confirm_ticks = 0U;
    comp_h5_stop_ff_releasing = 0U;
    comp_h5_stop_ff_complete = 0U;
    comp_h5_stop_ff_release_start_ms = 0U;
}

static void comp_update_h5_stop_ff_tail(uint32_t now,
                                        float left_rpm,
                                        float right_rpm)
{
    uint32_t elapsed_ms;

    if (comp_task != COMP_TASK_H5 ||
        comp_state != COMP_STATE_DECELERATING ||
        comp_ramp_progress < 0.5f ||
        comp_h5_stop_ff_complete)
    {
        return;
    }

    if (!comp_h5_stop_ff_releasing)
    {
        comp_h5_stop_ff_floor_deg = -COMP_H5_STOP_FF_TAIL_DEG;
        if (comp_absf(left_rpm) <= COMP_H5_STOP_FF_RPM &&
            comp_absf(right_rpm) <= COMP_H5_STOP_FF_RPM)
        {
            if (comp_h5_stop_ff_confirm_ticks < COMP_H5_STOP_FF_CONFIRM_TICKS)
            {
                comp_h5_stop_ff_confirm_ticks++;
            }
            if (comp_h5_stop_ff_confirm_ticks >= COMP_H5_STOP_FF_CONFIRM_TICKS)
            {
                comp_h5_stop_ff_releasing = 1U;
                comp_h5_stop_ff_release_start_ms = now;
            }
        }
        else
        {
            comp_h5_stop_ff_confirm_ticks = 0U;
        }
        return;
    }

    elapsed_ms = now - comp_h5_stop_ff_release_start_ms;
    if (elapsed_ms >= COMP_H5_STOP_FF_RELEASE_MS)
    {
        comp_h5_stop_ff_floor_deg = 0.0f;
        comp_h5_stop_ff_complete = 1U;
    }
    else
    {
        comp_h5_stop_ff_floor_deg =
            -COMP_H5_STOP_FF_TAIL_DEG *
            (1.0f - (float)elapsed_ms / (float)COMP_H5_STOP_FF_RELEASE_MS);
    }
}

static void comp_enable_ball(float target_mm)
{
    qiu_mubiao_mm = target_mm;
    qiu_moshi = 1U;
    jiaodu_moshi = 1U;
    dianji_shineng = 1U;
    QiuWeizhi_Reset();
}

static void comp_continue_ball(float target_mm)
{
    qiu_mubiao_mm = target_mm;
    qiu_moshi = 1U;
    jiaodu_moshi = 1U;
    dianji_shineng = 1U;
}

static uint8_t comp_ball_is_holding(float target_mm)
{
    return qiu_moshi && jiaodu_moshi && dianji_shineng &&
           comp_absf(qiu_mubiao_mm - target_mm) <= 0.05f;
}

static void comp_abort(uint32_t now)
{
    if (!comp_is_active())
    {
        return;
    }

    comp_locked_time_ms = now - comp_start_ms;
    comp_locked_max_error_mm = comp_max_error_mm;
    comp_result_locked = 1U;
    comp_track_speed_rpm = 0.0f;
    qiu_kazhu_qingjiao = QIU_KAZHU_QINGJIAO;
    qiu_kazhu_qingjiao_fu = QIU_KAZHU_QINGJIAO;
    QiuWeizhi_SetNegativePulseMode(0U);
    QiuWeizhi_SetDashMode(0U);
    qiu_ff_enable = 1U;
    comp_state = COMP_STATE_ABORTED;
    comp_oled_force = 1U;
    Set_pwm(0, 0);
}

static void comp_finish_h3(uint32_t now)
{
    comp_locked_time_ms = now - comp_start_ms;
    comp_result_locked = 1U;
    comp_state = COMP_STATE_DONE;
    comp_oled_force = 1U;
}

static void comp_finish_lap(uint32_t now)
{
    comp_locked_time_ms = now - comp_start_ms;
    comp_locked_max_error_mm = comp_max_error_mm;
    comp_result_locked = 1U;
    comp_decel_start_rpm = comp_track_speed_rpm;
    comp_ramp_progress = 0.0f;
    comp_ramp_paused = 0U;
    comp_ramp_resume_ticks = 0U;
    comp_state = COMP_STATE_DECELERATING;
    comp_oled_force = 1U;
}

static void comp_begin_chassis_brake(uint32_t now)
{
    int16_t left_pwm;
    int16_t right_pwm;

    comp_state = COMP_STATE_BRAKING;
    comp_track_speed_rpm = 0.0f;
    comp_brake_start_ms = now;
    comp_brake_left_direction = (comp_last_left_rpm >= 0.0f) ? 1 : -1;
    comp_brake_right_direction = (comp_last_right_rpm >= 0.0f) ? 1 : -1;
    comp_brake_left_active = (comp_absf(comp_last_left_rpm) > COMP_BRAKE_STOP_RPM);
    comp_brake_right_active = (comp_absf(comp_last_right_rpm) > COMP_BRAKE_STOP_RPM);
    comp_stop_confirm_ticks = 0U;

    left_pwm = comp_brake_left_active
             ? (int16_t)(-comp_brake_left_direction * COMP_BRAKE_PWM) : 0;
    right_pwm = comp_brake_right_active
              ? (int16_t)(-comp_brake_right_direction * COMP_BRAKE_PWM) : 0;
    Set_pwm(left_pwm, right_pwm);
    comp_oled_force = 1U;
}

static void comp_finish_h2(uint32_t now, uint8_t by_encoder)
{
    uint32_t primask = __get_PRIMASK();
    uint8_t accepted = 0U;

    __disable_irq();
    if (comp_task == COMP_TASK_H2 &&
        comp_state == COMP_STATE_RUNNING && comp_h2_counting)
    {
        /* 先切换状态，防止5ms灰度中断和20ms底盘中断重复完成。 */
        comp_state = COMP_STATE_BRAKING;
        comp_h2_latched_left = comp_h2_left_counts;
        comp_h2_latched_right = comp_h2_right_counts;
        comp_h2_latched_center =
            (comp_abs_i32(comp_h2_latched_left) +
             comp_abs_i32(comp_h2_latched_right) + 1) / 2;
        comp_h2_finish_by_encoder = by_encoder ? 1U : 0U;
        comp_h2_trigger_time_ms = now - comp_start_ms;
        comp_h2_report_pending = 1U;
        accepted = 1U;
    }
    if (primask == 0U)
    {
        __enable_irq();
    }

    if (accepted)
    {
        comp_begin_chassis_brake(now);
    }
}

static void comp_mark_h4_passed(uint32_t now)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    if (comp_task == COMP_TASK_H4 &&
        (comp_state == COMP_STATE_RUNNING ||
         comp_state == COMP_STATE_DECELERATING) &&
        comp_h4_counting && !comp_h4_passed)
    {
        comp_h4_latched_left = comp_h4_left_counts;
        comp_h4_latched_right = comp_h4_right_counts;
        comp_h4_latched_center =
            (comp_abs_i32(comp_h4_latched_left) +
             comp_abs_i32(comp_h4_latched_right) + 1) / 2;
        comp_h4_trigger_time_ms = now - comp_start_ms;
        comp_locked_time_ms = comp_h4_trigger_time_ms;
        comp_locked_max_error_mm = comp_max_error_mm;
        comp_result_locked = 1U;
        comp_h4_report_pending = 1U;
        comp_h4_passed = 1U;
        comp_oled_force = 1U;
    }
    if (primask == 0U)
    {
        __enable_irq();
    }
}

static void comp_finish_h4_calibration(uint32_t now)
{
    uint32_t primask = __get_PRIMASK();
    uint8_t accepted = 0U;

    __disable_irq();
    if (COMP_H4_MANUAL_CALIBRATION &&
        comp_task == COMP_TASK_H4 &&
        comp_state == COMP_STATE_RUNNING && comp_h4_counting)
    {
        comp_h4_latched_left = comp_h4_left_counts;
        comp_h4_latched_right = comp_h4_right_counts;
        comp_h4_latched_center =
            (comp_abs_i32(comp_h4_latched_left) +
             comp_abs_i32(comp_h4_latched_right) + 1) / 2;
        comp_h4_trigger_time_ms = now - comp_start_ms;
        comp_locked_time_ms = comp_h4_trigger_time_ms;
        comp_locked_max_error_mm = comp_max_error_mm;
        comp_result_locked = 1U;
        comp_h4_cal_sample_count++;
        comp_h4_latched_sample = comp_h4_cal_sample_count;
        comp_h4_report_pending = 1U;
        comp_h4_counting = 0U;
        comp_h4_passed = 1U;
        comp_track_speed_rpm = 0.0f;
        comp_state = COMP_STATE_DONE;
        comp_oled_force = 1U;
        accepted = 1U;
    }
    if (primask == 0U)
    {
        __enable_irq();
    }

    if (accepted)
    {
        Set_pwm(0, 0);
    }
}

static void comp_key_init(Competition_Key *key, GPIO_TypeDef *port, uint16_t pin)
{
    GPIO_PinState state = HAL_GPIO_ReadPin(port, pin);

    key->port = port;
    key->pin = pin;
    key->raw = state;
    key->stable = state;
    key->changed_ms = HAL_GetTick();
    key->pressed_event = 0U;
}

static void comp_keys_update(uint32_t now)
{
    uint8_t index;

    for (index = 0U; index < COMP_KEY_COUNT; index++)
    {
        Competition_Key *key = &comp_keys[index];
        GPIO_PinState sample = HAL_GPIO_ReadPin(key->port, key->pin);

        key->pressed_event = 0U;
        if (sample != key->raw)
        {
            key->raw = sample;
            key->changed_ms = now;
        }
        else if (sample != key->stable &&
                 (now - key->changed_ms) >= 20U)
        {
            key->stable = sample;
            if (sample == GPIO_PIN_RESET)
            {
                key->pressed_event = 1U;
            }
        }
    }
}

static void comp_reset_h3_stability(void)
{
    comp_h3_stable_start_ms = 0U;
    comp_h3_stable_samples = 0U;
    comp_h3_positive_sum = 0.0f;
    comp_h3_negative_sum = 0.0f;
}

static uint8_t comp_h6_ready(uint32_t now)
{
    return vision.connected && vision.target_valid &&
           comp_h6_ready_start_ms != 0U &&
           (now - comp_h6_ready_start_ms) >= COMP_H6_READY_MS &&
           comp_h6_ready_samples >= COMP_H6_READY_MIN_SAMPLES;
}

static uint8_t comp_can_prepare_h6(void)
{
    return comp_task == COMP_TASK_H6 &&
           (comp_state == COMP_STATE_MENU ||
            comp_state == COMP_STATE_DONE ||
            comp_state == COMP_STATE_ABORTED);
}

static void comp_start_task(uint32_t now)
{
    float target_mm;

    if (comp_task != COMP_TASK_H2 && !vision.connected)
    {
        comp_set_message("NO VISION", now);
        return;
    }
    if (comp_task != COMP_TASK_H2 && motor_date[0].rx_count == 0U)
    {
        comp_set_message("NO BALL CAN", now);
        return;
    }
    if (comp_task == COMP_TASK_H6 && !comp_h6_ready(now))
    {
        comp_set_message("WAIT TARGET/BALL", now);
        return;
    }

    comp_start_ms = now;
    comp_locked_time_ms = 0U;
    comp_result_locked = 0U;
    comp_max_error_mm = 0.0f;
    comp_locked_max_error_mm = 0.0f;
    comp_h3_positive_valid = 0U;
    comp_h3_negative_valid = 0U;
    comp_h3_positive_position_mm = 0.0f;
    comp_h3_negative_position_mm = 0.0f;
    comp_reset_h3_stability();
    comp_reset_marker();
    comp_reset_ramp();
    if (comp_task == COMP_TASK_H2)
    {
        comp_h2_left_counts = 0;
        comp_h2_right_counts = 0;
        comp_h2_latched_left = 0;
        comp_h2_latched_right = 0;
        comp_h2_latched_center = 0;
        comp_h2_counting = 0U;
        comp_h2_wait_clear = 0U;
        comp_h2_report_pending = 0U;
        comp_h2_finish_by_encoder = 0U;
        comp_h2_trigger_time_ms = 0U;
    }
    else if (comp_task == COMP_TASK_H4)
    {
        comp_h4_left_counts = 0;
        comp_h4_right_counts = 0;
        comp_h4_latched_left = 0;
        comp_h4_latched_right = 0;
        comp_h4_latched_center = 0;
        comp_h4_counting = 0U;
        comp_h4_passed = 0U;
        comp_h4_report_pending = 0U;
        comp_h4_trigger_time_ms = 0U;
        comp_h4_run_start_ms = 0U;
    }
    comp_last_vision_rx_count = vision.rx_count;
    comp_last_oled_page = 0xFFU;
    qiu_kazhu_qingjiao = QIU_KAZHU_QINGJIAO;
    qiu_kazhu_qingjiao_fu = QIU_KAZHU_QINGJIAO;
    QiuWeizhi_SetNegativePulseMode(0U);
    QiuWeizhi_SetDashMode(0U);
    qiu_ff_enable = 1U;

    if (comp_task == COMP_TASK_H2)
    {
        comp_enable_ball(0.0f);
        comp_track_speed_rpm = COMP_H2_SPEED_RPM;
        comp_state = COMP_STATE_RUNNING;
    }
    else if (comp_task == COMP_TASK_H3)
    {
        qiu_kazhu_qingjiao = COMP_H3_STUCK_TILT_DEG;
        qiu_kazhu_qingjiao_fu = COMP_H3_NEGATIVE_STUCK_TILT_DEG;
        qiu_ff_enable = 0U;
        /* H3是唯一要跑50~100mm长距离到点的任务：换可行刹车曲线，
         * 顺带让近区限幅在球还快时放开，不然物理上必然冲过头。 */
        QiuWeizhi_SetDashMode(1U);
        comp_enable_ball(COMP_H3_POSITIVE_MM);
        comp_state = COMP_STATE_H3_POSITIVE;
    }
    else if (comp_task == COMP_TASK_H4)
    {
        comp_enable_ball(0.0f);
        comp_track_speed_rpm = 0.0f;
        comp_h4_left_counts = 0;
        comp_h4_right_counts = 0;
        comp_h4_run_start_ms = now;
        comp_h4_counting = 1U;
        comp_state = COMP_STATE_RUNNING;
    }
    else
    {
        target_mm = (comp_task == COMP_TASK_H6) ? vision.target_mm : 0.0f;
        comp_h6_locked_target_mm = target_mm;
        if (comp_task == COMP_TASK_H6 && comp_ball_is_holding(target_mm))
        {
            /* H6在菜单阶段已经稳住指定位置。启动时保留位置积分、球速历史
             * 和当前倾角，避免远端曲率补偿被清空后重新产生大偏差。 */
            comp_continue_ball(target_mm);
        }
        else
        {
            comp_enable_ball(target_mm);
        }
        /* H5/H6按键即为正式起点；前10秒横线由终点防误触时间窗忽略。 */
        comp_timing_started = 1U;
        comp_left_start_line = 1U;
        comp_state = COMP_STATE_RUNNING;
    }

    comp_oled_force = 1U;
}

static void comp_update_h6_ready(uint32_t now)
{
    float error;
    float ready_window_mm;

    if (!comp_can_prepare_h6() || !vision.connected || !vision.target_valid)
    {
        comp_h6_ready_start_ms = 0U;
        comp_h6_ready_samples = 0U;
        return;
    }

    error = comp_absf(vision.distance_mm - vision.target_mm);
    ready_window_mm = (comp_absf(vision.target_mm) > COMP_H6_FAR_TARGET_MM)
                    ? COMP_H6_FAR_READY_WINDOW_MM
                    : COMP_H6_READY_WINDOW_MM;
    if (error <= ready_window_mm &&
        comp_absf(qiu_su_mm) <= COMP_H3_SPEED_MM_S)
    {
        if (comp_h6_ready_start_ms == 0U)
        {
            comp_h6_ready_start_ms = now;
            comp_h6_ready_samples = 0U;
        }
        comp_h6_ready_samples++;
    }
    else
    {
        comp_h6_ready_start_ms = 0U;
        comp_h6_ready_samples = 0U;
    }
}

static void comp_update_h3(uint32_t now)
{
    float position = vision.distance_mm;
    float speed = comp_absf(qiu_su_mm);
    uint8_t in_window;
    uint32_t required_ms;
    uint16_t required_samples;

    if (comp_state == COMP_STATE_H3_POSITIVE)
    {
        in_window = comp_absf(position - COMP_H3_POSITIVE_MM) <= COMP_H3_WINDOW_MM;
        required_ms = COMP_H3_POSITIVE_STABLE_MS;
        required_samples = COMP_H3_POSITIVE_MIN_SAMPLES;
    }
    else if (comp_state == COMP_STATE_H3_CENTER)
    {
        in_window = comp_absf(position - COMP_H3_CENTER_MM) <= COMP_H3_WINDOW_MM;
        required_ms = COMP_H3_CENTER_STABLE_MS;
        required_samples = COMP_H3_CENTER_MIN_SAMPLES;
    }
    else if (comp_state == COMP_STATE_H3_NEGATIVE)
    {
        in_window = comp_absf(position - COMP_H3_NEGATIVE_MM) <= COMP_H3_WINDOW_MM;
        required_ms = COMP_H3_NEGATIVE_STABLE_MS;
        required_samples = COMP_H3_NEGATIVE_MIN_SAMPLES;
    }
    else
    {
        return;
    }

    if (in_window && speed <= COMP_H3_SPEED_MM_S)
    {
        if (comp_h3_stable_start_ms == 0U)
        {
            comp_h3_stable_start_ms = now;
            comp_h3_stable_samples = 0U;
            comp_h3_positive_sum = 0.0f;
            comp_h3_negative_sum = 0.0f;
        }

        if (comp_state == COMP_STATE_H3_POSITIVE)
        {
            comp_h3_positive_sum += position;
        }
        else
        {
            comp_h3_negative_sum += position;
        }
        comp_h3_stable_samples++;

        if ((now - comp_h3_stable_start_ms) >= required_ms &&
            comp_h3_stable_samples >= required_samples)
        {
            if (comp_state == COMP_STATE_H3_POSITIVE)
            {
                comp_h3_positive_position_mm =
                    comp_h3_positive_sum / (float)comp_h3_stable_samples;
                comp_h3_positive_valid = 1U;
                comp_reset_h3_stability();
                /* 题目只要求"到达后折返"，不要求在中心停一下。
                 * 中间那次停稳白吃1秒多，直接一口气奔-50。 */
                QiuWeizhi_SetNegativePulseMode(1U);
                comp_enable_ball(COMP_H3_NEGATIVE_MM);
                comp_state = COMP_STATE_H3_NEGATIVE;
            }
            else if (comp_state == COMP_STATE_H3_CENTER)
            {
                comp_reset_h3_stability();
                QiuWeizhi_SetNegativePulseMode(1U);
                comp_enable_ball(COMP_H3_NEGATIVE_MM);
                comp_state = COMP_STATE_H3_NEGATIVE;
            }
            else
            {
                comp_h3_negative_position_mm =
                    comp_h3_negative_sum / (float)comp_h3_stable_samples;
                comp_h3_negative_valid = 1U;
                comp_finish_h3(now);
            }
            comp_oled_force = 1U;
        }
    }
    else
    {
        comp_reset_h3_stability();
    }
}

static void comp_process_vision_sample(uint32_t now)
{
    float target;
    float error;
    uint8_t track_error = 0U;

    comp_update_h6_ready(now);

    if (comp_state == COMP_STATE_H3_POSITIVE ||
        comp_state == COMP_STATE_H3_CENTER ||
        comp_state == COMP_STATE_H3_NEGATIVE)
    {
        comp_update_h3(now);
        return;
    }

    if (comp_task == COMP_TASK_H4 &&
        comp_h4_counting && !comp_h4_passed &&
        (comp_state == COMP_STATE_RUNNING ||
         comp_state == COMP_STATE_DECELERATING))
    {
        target = 0.0f;
        track_error = 1U;
    }
    else if ((comp_task == COMP_TASK_H5 || comp_task == COMP_TASK_H6) &&
             (comp_state == COMP_STATE_PREHOLD ||
              comp_state == COMP_STATE_RUNNING ||
              comp_state == COMP_STATE_CROSSING_FINISH))
    {
        target = (comp_task == COMP_TASK_H6) ? comp_h6_locked_target_mm : 0.0f;
        track_error = 1U;
    }

    if (track_error)
    {
        error = comp_absf(vision.distance_mm - target);
        if (error > comp_max_error_mm)
        {
            comp_max_error_mm = error;
        }
    }
}

static void comp_update_idle_ball_target(void)
{
    float target = 0.0f;

    if (comp_state == COMP_STATE_MENU)
    {
        qiu_kazhu_qingjiao = QIU_KAZHU_QINGJIAO;
        qiu_kazhu_qingjiao_fu = QIU_KAZHU_QINGJIAO;
        QiuWeizhi_SetNegativePulseMode(0U);
        QiuWeizhi_SetDashMode(0U);
        /* 菜单也是串口位置环调参入口，保留 FE0/FE1 的人工选择。
         * 各比赛任务启动时会按题目显式设置前馈，H3 仍固定关闭。 */
    }

    if (comp_state != COMP_STATE_MENU && !comp_can_prepare_h6())
    {
        return;
    }
    if (comp_task == COMP_TASK_H6 && vision.target_valid)
    {
        target = vision.target_mm;
    }

    if (comp_absf(target - comp_h6_idle_target_mm) > 0.05f)
    {
        comp_h6_idle_target_mm = target;
        /* 预备/菜单阶段只更新球目标，不覆盖串口 FE0/FE1。
         * H6 真正启动时 comp_start_task() 会显式开启前馈。 */
        comp_enable_ball(target);
        comp_h6_ready_start_ms = 0U;
        comp_h6_ready_samples = 0U;
    }
}

static void comp_format_time(char *output, size_t size, uint32_t milliseconds)
{
    uint32_t seconds = milliseconds / 1000U;
    uint32_t centiseconds = (milliseconds % 1000U) / 10U;

    (void)snprintf(output, size, "%2lu.%02lus",
                   (unsigned long)seconds, (unsigned long)centiseconds);
}

static void comp_format_signed_tenth(char *output, size_t size, float value)
{
    int32_t tenths = (value >= 0.0f)
                   ? (int32_t)(value * 10.0f + 0.5f)
                   : (int32_t)(value * 10.0f - 0.5f);
    uint32_t magnitude = (tenths >= 0) ? (uint32_t)tenths : (uint32_t)(-tenths);

    (void)snprintf(output, size, "%c%lu.%01lu",
                   (tenths >= 0) ? '+' : '-',
                   (unsigned long)(magnitude / 10U),
                   (unsigned long)(magnitude % 10U));
}

static void comp_oled_line(uint8_t y, const char *text)
{
    char line[22];

    (void)snprintf(line, sizeof(line), "%-21.21s", text);
    OLED_ShowString(0U, y, (u8 *)line, 12U, 1U);
}

static uint8_t comp_oled_page(void)
{
    if (comp_state == COMP_STATE_MENU)
    {
        return (uint8_t)comp_task;
    }
    return (uint8_t)(0x20U + (uint8_t)comp_task);
}

static void comp_render_menu(uint32_t now)
{
    char line[22];
    char value[12];

    switch (comp_task)
    {
        case COMP_TASK_H2:
            comp_oled_line(0U, "SELECT H2");
            comp_oled_line(16U, "LAP AND STOP");
            break;
        case COMP_TASK_H3:
            comp_oled_line(0U, "SELECT H3");
            comp_oled_line(16U, "+50 THEN -50");
            break;
        case COMP_TASK_H4:
            comp_oled_line(0U, "SELECT H4");
            comp_oled_line(16U, "AUTO A TO B");
            break;
        case COMP_TASK_H5:
            comp_oled_line(0U, "SELECT H5");
            comp_oled_line(16U, "LAP HOLD CENTER");
            break;
        case COMP_TASK_H6:
            comp_oled_line(0U, "SELECT H6");
            if (vision.target_valid)
            {
                comp_format_signed_tenth(value, sizeof(value), vision.target_mm);
                (void)snprintf(line, sizeof(line), "TGT: %smm", value);
            }
            else
            {
                (void)snprintf(line, sizeof(line), "TGT: ----");
            }
            comp_oled_line(16U, line);
            break;
        default:
            break;
    }

    comp_oled_line(32U, "K1< K2> K3 START");
    if (now < comp_menu_message_until_ms)
    {
        comp_oled_line(48U, comp_menu_message);
    }
    else if (comp_task == COMP_TASK_H6 && vision.connected)
    {
        comp_format_signed_tenth(value, sizeof(value), vision.distance_mm);
        (void)snprintf(line, sizeof(line), "POS: %smm", value);
        comp_oled_line(48U, line);
    }
    else
    {
        comp_oled_line(48U, "K4 STOP/BACK");
    }
}

static void comp_render_task(uint32_t now)
{
    char line[22];
    char value_a[12];
    char time_text[12];

    comp_format_time(time_text, sizeof(time_text), comp_display_time_ms(now));

    if (comp_task == COMP_TASK_H2)
    {
        size_t time_chars = strlen(time_text);
        uint16_t time_width = (uint16_t)(time_chars * 12U);
        uint8_t time_x = (time_width < 128U)
                       ? (uint8_t)((128U - time_width) / 2U) : 0U;

        OLED_ShowString(48U, 4U, (u8 *)"TIME", 16U, 1U);
        OLED_ShowString(time_x, 28U, (u8 *)time_text, 24U, 1U);
        return;
    }
    else if (comp_task == COMP_TASK_H3)
    {
        if (comp_h3_positive_valid)
        {
            comp_format_signed_tenth(value_a, sizeof(value_a),
                                     comp_h3_positive_position_mm);
            (void)snprintf(line, sizeof(line), "+50: %smm", value_a);
        }
        else
        {
            (void)snprintf(line, sizeof(line), "+50: ---- mm");
        }
        comp_oled_line(4U, line);

        if (comp_h3_negative_valid)
        {
            comp_format_signed_tenth(value_a, sizeof(value_a),
                                     comp_h3_negative_position_mm);
            (void)snprintf(line, sizeof(line), "-50: %smm", value_a);
        }
        else
        {
            (void)snprintf(line, sizeof(line), "-50: ---- mm");
        }
        comp_oled_line(24U, line);
        (void)snprintf(line, sizeof(line), "TIME: %s", time_text);
        comp_oled_line(44U, line);
    }
    else if (comp_task == COMP_TASK_H4)
    {
        OLED_ShowString(32U, 24U, (u8 *)"H4 START", 16U, 1U);
    }
    else if (comp_task == COMP_TASK_H5)
    {
        OLED_ShowString(32U, 24U, (u8 *)"H5 START", 16U, 1U);
    }
    else if (comp_task == COMP_TASK_H6)
    {
        OLED_ShowString(52U, 24U, (u8 *)"TGT", 16U, 1U);
    }
}

static void comp_render_oled(uint32_t now)
{
    uint8_t page = comp_oled_page();

    if (page != comp_last_oled_page)
    {
        OLED_Clear();
        comp_last_oled_page = page;
    }

    if (comp_state == COMP_STATE_MENU)
    {
        comp_render_menu(now);
    }
    else
    {
        comp_render_task(now);
    }
    OLED_Refresh();
}

static uint8_t comp_read_gray_bits(void)
{
    uint8_t bits = 0U;

    if (HAL_GPIO_ReadPin(S1_GPIO_Port, S1_Pin) == GPIO_PIN_RESET) bits |= 0x01U;
    if (HAL_GPIO_ReadPin(S2_GPIO_Port, S2_Pin) == GPIO_PIN_RESET) bits |= 0x02U;
    if (HAL_GPIO_ReadPin(S3_GPIO_Port, S3_Pin) == GPIO_PIN_RESET) bits |= 0x04U;
    if (HAL_GPIO_ReadPin(S4_GPIO_Port, S4_Pin) == GPIO_PIN_RESET) bits |= 0x08U;
    if (HAL_GPIO_ReadPin(S5_GPIO_Port, S5_Pin) == GPIO_PIN_RESET) bits |= 0x10U;
    if (HAL_GPIO_ReadPin(S6_GPIO_Port, S6_Pin) == GPIO_PIN_RESET) bits |= 0x20U;
    if (HAL_GPIO_ReadPin(S7_GPIO_Port, S7_Pin) == GPIO_PIN_RESET) bits |= 0x40U;
    if (HAL_GPIO_ReadPin(S8_GPIO_Port, S8_Pin) == GPIO_PIN_RESET) bits |= 0x80U;
    return bits;
}

static uint8_t comp_popcount8(uint8_t value)
{
    uint8_t count = 0U;

    while (value != 0U)
    {
        count = (uint8_t)(count + (value & 1U));
        value >>= 1;
    }
    return count;
}

static uint8_t comp_is_marker3(uint8_t bits)
{
    return ((bits & 0x07U) == 0x07U) ||
           ((bits & 0x0EU) == 0x0EU) ||
           ((bits & 0x1CU) == 0x1CU) ||
           ((bits & 0x38U) == 0x38U) ||
           ((bits & 0x70U) == 0x70U) ||
           ((bits & 0xE0U) == 0xE0U);
}

static uint8_t comp_has_marker4(uint8_t bits)
{
    return ((bits & 0x0FU) == 0x0FU) ||
           ((bits & 0x1EU) == 0x1EU) ||
           ((bits & 0x3CU) == 0x3CU) ||
           ((bits & 0x78U) == 0x78U) ||
           ((bits & 0xF0U) == 0xF0U);
}

static float comp_current_ball_error(void)
{
    float target = (comp_task == COMP_TASK_H6) ? comp_h6_locked_target_mm : 0.0f;
    return comp_absf(vision.distance_mm - target);
}

static uint8_t comp_ramp_can_advance(void)
{
    float error = comp_current_ball_error();
    float pause_mm = COMP_BALL_PAUSE_MM;
    float resume_mm = COMP_BALL_RESUME_MM;

    if (!vision.connected)
    {
        return 0U;
    }

    if (comp_task == COMP_TASK_H4 ||
        comp_task == COMP_TASK_H5 ||
        comp_task == COMP_TASK_H6)
    {
        /* 起步目标过低时底盘尚未克服静摩擦，此时暂停斜坡会让目标
         * 永久卡在几RPM。先推进到可运动速度，再启用球误差保护。 */
        if (comp_state == COMP_STATE_RUNNING &&
            comp_track_speed_rpm < COMP_RAMP_FORCE_RPM)
        {
            comp_ramp_paused = 0U;
            comp_ramp_resume_ticks = 0U;
            return 1U;
        }
    }

    if (error > pause_mm)
    {
        comp_ramp_paused = 1U;
        comp_ramp_resume_ticks = 0U;
    }
    else if (comp_ramp_paused)
    {
        if (error <= resume_mm)
        {
            if (comp_ramp_resume_ticks < COMP_BALL_RESUME_TICKS)
            {
                comp_ramp_resume_ticks++;
            }
            if (comp_ramp_resume_ticks >= COMP_BALL_RESUME_TICKS)
            {
                comp_ramp_paused = 0U;
            }
        }
        else
        {
            comp_ramp_resume_ticks = 0U;
        }
    }
    return !comp_ramp_paused;
}

void Competition_Init(void)
{
    comp_key_init(&comp_keys[COMP_KEY_PREVIOUS], KEY1_GPIO_Port, KEY1_Pin);
    comp_key_init(&comp_keys[COMP_KEY_NEXT], KEY2_GPIO_Port, KEY2_Pin);
    comp_key_init(&comp_keys[COMP_KEY_START], KEY3_GPIO_Port, KEY3_Pin);
    comp_key_init(&comp_keys[COMP_KEY_STOP], KEY4_GPIO_Port, KEY4_Pin);

    comp_task = COMP_TASK_H2;
    comp_state = COMP_STATE_MENU;
    comp_result_locked = 0U;
    comp_h6_idle_target_mm = 1000.0f;
    comp_last_vision_rx_count = vision.rx_count;
    comp_last_oled_ms = HAL_GetTick();
    comp_last_oled_page = 0xFFU;
    comp_oled_force = 1U;
    comp_update_idle_ball_target();
    comp_render_oled(HAL_GetTick());
}

void Competition_Service(void)
{
    uint32_t now = HAL_GetTick();

    comp_keys_update(now);

    if (comp_h2_report_pending)
    {
        int32_t left = comp_h2_latched_left;
        int32_t right = comp_h2_latched_right;
        int32_t center = comp_h2_latched_center;
        uint8_t by_encoder = comp_h2_finish_by_encoder;
        uint32_t trigger_ms = comp_h2_trigger_time_ms;

        comp_h2_report_pending = 0U;
        printf("# H2ENC left=%ld right=%ld center=%ld lap=%ld diff=%ld"
               " source=%s trigger_ms=%lu\r\n",
               (long)left, (long)right, (long)center,
               (long)COMP_H2_LAP_COUNTS,
               (long)(center - COMP_H2_LAP_COUNTS),
               by_encoder ? "encoder" : "marker",
               (unsigned long)trigger_ms);
    }

    if (comp_h4_report_pending)
    {
        int32_t left = comp_h4_latched_left;
        int32_t right = comp_h4_latched_right;
        int32_t center = comp_h4_latched_center;
        uint32_t trigger_ms = comp_h4_trigger_time_ms;
        uint16_t sample = comp_h4_latched_sample;

        comp_h4_report_pending = 0U;
        if (COMP_H4_MANUAL_CALIBRATION)
        {
            printf("# H4ENC CAL sample=%u left=%ld right=%ld center=%ld"
                   " elapsed_ms=%lu maxerr=%.1f\r\n",
                   (unsigned int)sample,
                   (long)left, (long)right, (long)center,
                   (unsigned long)trigger_ms,
                   (double)comp_locked_max_error_mm);
        }
        else
        {
            printf("# H4ENC left=%ld right=%ld center=%ld target=%ld diff=%ld"
                   " trigger_ms=%lu maxerr=%.1f\r\n",
                   (long)left, (long)right, (long)center,
                   (long)COMP_H4_AB_COUNTS,
                   (long)(center - COMP_H4_AB_COUNTS),
                   (unsigned long)trigger_ms,
                   (double)comp_locked_max_error_mm);
        }
    }

    if (comp_keys[COMP_KEY_STOP].pressed_event)
    {
        if (comp_is_active())
        {
            comp_abort(now);
        }
        else if (comp_state == COMP_STATE_DONE ||
                 comp_state == COMP_STATE_ABORTED)
        {
            comp_state = COMP_STATE_MENU;
            comp_result_locked = 0U;
            comp_oled_force = 1U;
        }
    }

    if (COMP_H4_MANUAL_CALIBRATION &&
        comp_task == COMP_TASK_H4 &&
        comp_state == COMP_STATE_RUNNING &&
        comp_keys[COMP_KEY_START].pressed_event)
    {
        comp_finish_h4_calibration(now);
    }
    else if (comp_state == COMP_STATE_MENU)
    {
        if (comp_keys[COMP_KEY_PREVIOUS].pressed_event)
        {
            comp_task = (comp_task == COMP_TASK_H2)
                      ? COMP_TASK_H6 : (Competition_Task)(comp_task - 1);
            comp_h6_idle_target_mm = 1000.0f;
            comp_oled_force = 1U;
        }
        if (comp_keys[COMP_KEY_NEXT].pressed_event)
        {
            comp_task = (comp_task == COMP_TASK_H6)
                      ? COMP_TASK_H2 : (Competition_Task)(comp_task + 1);
            comp_h6_idle_target_mm = 1000.0f;
            comp_oled_force = 1U;
        }
        if (comp_keys[COMP_KEY_START].pressed_event)
        {
            comp_start_task(now);
        }
    }
    else if ((comp_state == COMP_STATE_DONE ||
              comp_state == COMP_STATE_ABORTED) &&
             comp_keys[COMP_KEY_START].pressed_event)
    {
        comp_start_task(now);
    }

    comp_update_idle_ball_target();

    if (comp_is_active() && comp_task != COMP_TASK_H2 && !vision.connected)
    {
        comp_abort(now);
    }

    if (comp_state == COMP_STATE_PREHOLD &&
        (now - comp_start_ms) >= COMP_PREHOLD_MS)
    {
        comp_state = COMP_STATE_RUNNING;
        comp_oled_force = 1U;
    }

    if (vision.rx_count != comp_last_vision_rx_count)
    {
        comp_last_vision_rx_count = vision.rx_count;
        comp_process_vision_sample(now);
    }

    if (comp_oled_force || (now - comp_last_oled_ms) >= COMP_OLED_PERIOD_MS)
    {
        comp_last_oled_ms = now;
        comp_oled_force = 0U;
        comp_render_oled(now);
    }
}

void Competition_MarkerTick5ms(void)
{
    uint32_t now;
    uint8_t bits;
    uint8_t active_count;
    uint8_t marker_event = 0U;

    if (!(comp_task == COMP_TASK_H2 ||
          comp_task == COMP_TASK_H4 ||
          comp_task == COMP_TASK_H5 ||
          comp_task == COMP_TASK_H6))
    {
        return;
    }
    if (!(comp_state == COMP_STATE_RUNNING ||
          comp_state == COMP_STATE_CROSSING_FINISH))
    {
        return;
    }

    now = HAL_GetTick();
    bits = comp_read_gray_bits();
    active_count = comp_popcount8(bits);

    if (comp_task == COMP_TASK_H2)
    {
        marker_event = comp_has_marker4(bits) || comp_is_marker3(bits);

        /* 起点处第一次识别横线只建立计数基准，不发送数据。 */
        if (!comp_h2_counting)
        {
            if (marker_event)
            {
                comp_start_ms = now;
                comp_h2_left_counts = 0;
                comp_h2_right_counts = 0;
                comp_h2_counting = 1U;
                comp_h2_wait_clear = 1U;
                comp_timing_started = 1U;
                comp_leave_samples = 0U;
                comp_oled_force = 1U;
            }
            return;
        }

        /* 必须完全离开当前横线后才能识别下一次，防止同一横线重复触发。 */
        if (comp_h2_wait_clear)
        {
            if (active_count <= 2U)
            {
                if (comp_leave_samples < COMP_MARKER_LEAVE_SAMPLES)
                {
                    comp_leave_samples++;
                }
                if (comp_leave_samples >= COMP_MARKER_LEAVE_SAMPLES)
                {
                    comp_h2_wait_clear = 0U;
                    comp_leave_samples = 0U;
                    comp_oled_force = 1U;
                }
            }
            else
            {
                comp_leave_samples = 0U;
            }
            return;
        }

        if ((now - comp_start_ms) < COMP_FINISH_ARM_MS || !marker_event)
        {
            return;
        }

        comp_finish_h2(now, 0U);
        return;
    }

    if (comp_task == COMP_TASK_H4)
    {
        marker_event = comp_has_marker4(bits) || comp_is_marker3(bits);
        if (!comp_h4_counting && marker_event)
        {
            comp_h4_left_counts = 0;
            comp_h4_right_counts = 0;
            comp_h4_run_start_ms = now;
            comp_h4_counting = 1U;
            comp_oled_force = 1U;
        }
        return;
    }

    if (!comp_left_start_line)
    {
        marker_event = comp_has_marker4(bits) || comp_is_marker3(bits);
        if (!comp_timing_started)
        {
            if (marker_event)
            {
                /* H5/H6以首次识别起点横线为计时和最大误差统计起点。 */
                comp_start_ms = now;
                comp_max_error_mm = 0.0f;
                comp_locked_max_error_mm = 0.0f;
                comp_timing_started = 1U;
                comp_leave_samples = 0U;
                comp_oled_force = 1U;
            }
            return;
        }

        if (active_count <= 2U)
        {
            if (comp_leave_samples < COMP_MARKER_LEAVE_SAMPLES)
            {
                comp_leave_samples++;
            }
            if (comp_leave_samples >= COMP_MARKER_LEAVE_SAMPLES)
            {
                comp_left_start_line = 1U;
            }
        }
        else
        {
            comp_leave_samples = 0U;
        }
        return;
    }

    if (comp_state == COMP_STATE_CROSSING_FINISH)
    {
        if (active_count <= 2U)
        {
            if (++comp_clear_samples >= COMP_MARKER_CLEAR_SAMPLES)
            {
                comp_clear_samples = 0U;
                comp_finish_lap(now);
            }
        }
        else
        {
            comp_clear_samples = 0U;
        }
        return;
    }

    if ((now - comp_start_ms) < COMP_FINISH_ARM_MS)
    {
        return;
    }

    /* 三点和四点启停线使用相同判定：单个5ms采样命中即触发。 */
    if (comp_has_marker4(bits) || comp_is_marker3(bits))
    {
        marker_event = 1U;
    }

    if (marker_event)
    {
        comp_state = COMP_STATE_CROSSING_FINISH;
        comp_clear_samples = 0U;
        comp_oled_force = 1U;
    }
}

Competition_ChassisMode Competition_ChassisTick20ms(float left_rpm,
                                                     float right_rpm,
                                                     int32_t left_encoder_delta,
                                                     int32_t right_encoder_delta,
                                                     int16_t *left_pwm,
                                                     int16_t *right_pwm)
{
    uint32_t now = HAL_GetTick();
    float target_speed;
    float ramp_time_ms = (comp_task == COMP_TASK_H4)
                       ? (float)COMP_H4_RAMP_TIME_MS
                       : (float)COMP_RAMP_TIME_MS;
    float ramp_step = 20.0f / ramp_time_ms;
    int32_t h2_center_counts = 0;
    int32_t h4_center_counts = 0;
    uint8_t h2_encoder_stop = 0U;
    uint8_t h4_pass_event = 0U;
    uint8_t h4_start_decel = 0U;

    comp_last_left_rpm = left_rpm;
    comp_last_right_rpm = right_rpm;
    if (comp_task == COMP_TASK_H2 && comp_state == COMP_STATE_RUNNING)
    {
        uint32_t primask = __get_PRIMASK();

        __disable_irq();
        comp_h2_left_counts += left_encoder_delta;
        comp_h2_right_counts += right_encoder_delta;
        if (comp_h2_counting)
        {
            h2_center_counts =
                (comp_abs_i32(comp_h2_left_counts) +
                 comp_abs_i32(comp_h2_right_counts) + 1) / 2;
            if ((now - comp_start_ms) >= COMP_FINISH_ARM_MS &&
                h2_center_counts >= COMP_H2_LAP_COUNTS)
            {
                h2_encoder_stop = 1U;
            }
        }
        if (primask == 0U)
        {
            __enable_irq();
        }
        if (h2_encoder_stop)
        {
            comp_finish_h2(now, 1U);
        }
    }
    if (comp_task == COMP_TASK_H4 &&
        (comp_state == COMP_STATE_RUNNING ||
         comp_state == COMP_STATE_DECELERATING) &&
        comp_h4_counting)
    {
        uint32_t primask = __get_PRIMASK();

        __disable_irq();
        comp_h4_left_counts += left_encoder_delta;
        comp_h4_right_counts += right_encoder_delta;
        h4_center_counts =
            (comp_abs_i32(comp_h4_left_counts) +
             comp_abs_i32(comp_h4_right_counts) + 1) / 2;
        if (!COMP_H4_MANUAL_CALIBRATION)
        {
            if (!comp_h4_passed && h4_center_counts >= COMP_H4_AB_COUNTS)
            {
                h4_pass_event = 1U;
            }
            if (comp_state == COMP_STATE_RUNNING &&
                h4_center_counts >= COMP_H4_DECEL_COUNTS)
            {
                h4_start_decel = 1U;
            }
        }
        if (primask == 0U)
        {
            __enable_irq();
        }
        if (h4_start_decel)
        {
            comp_decel_start_rpm = comp_track_speed_rpm;
            comp_ramp_progress = 0.0f;
            comp_ramp_paused = 0U;
            comp_ramp_resume_ticks = 0U;
            comp_state = COMP_STATE_DECELERATING;
            comp_oled_force = 1U;
        }
        if (h4_pass_event)
        {
            comp_mark_h4_passed(now);
        }
    }
    *left_pwm = 0;
    *right_pwm = 0;

    if (HAL_GPIO_ReadPin(KEY4_GPIO_Port, KEY4_Pin) == GPIO_PIN_RESET &&
        comp_is_active())
    {
        comp_abort(now);
    }

    if (comp_state == COMP_STATE_BRAKING)
    {
        uint8_t brake_timeout =
            (now - comp_brake_start_ms) >= COMP_BRAKE_TIMEOUT_MS;
        uint8_t left_stopped =
            comp_absf(left_rpm) <= COMP_BRAKE_STOP_RPM;
        uint8_t right_stopped =
            comp_absf(right_rpm) <= COMP_BRAKE_STOP_RPM;

        if (left_encoder_delta == 0 && right_encoder_delta == 0)
        {
            if (comp_stop_confirm_ticks < COMP_STOP_CONFIRM_TICKS)
            {
                comp_stop_confirm_ticks++;
            }
        }
        else
        {
            comp_stop_confirm_ticks = 0U;
        }

        if (brake_timeout)
        {
            comp_brake_left_active = 0U;
            comp_brake_right_active = 0U;
        }
        else if (comp_brake_left_active && left_stopped)
        {
            comp_brake_left_active = 0U;
        }
        if (!brake_timeout && comp_brake_right_active && right_stopped)
        {
            comp_brake_right_active = 0U;
        }

        *left_pwm = comp_brake_left_active
                  ? (int16_t)(-comp_brake_left_direction * COMP_BRAKE_PWM) : 0;
        *right_pwm = comp_brake_right_active
                   ? (int16_t)(-comp_brake_right_direction * COMP_BRAKE_PWM) : 0;

        if (comp_stop_confirm_ticks >= COMP_STOP_CONFIRM_TICKS)
        {
            comp_locked_time_ms = now - comp_start_ms;
            comp_result_locked = 1U;
            comp_state = COMP_STATE_DONE;
            comp_oled_force = 1U;
            return COMP_CHASSIS_STOP;
        }
        return COMP_CHASSIS_DIRECT;
    }

    if (comp_state == COMP_STATE_RUNNING ||
        comp_state == COMP_STATE_CROSSING_FINISH)
    {
        if (comp_task == COMP_TASK_H2)
        {
            comp_track_speed_rpm =
                (comp_h2_counting && h2_center_counts >= COMP_H2_SLOW_COUNTS)
                ? COMP_H2_SLOW_SPEED_RPM : COMP_H2_SPEED_RPM;
            return COMP_CHASSIS_TRACK;
        }
        if (comp_task == COMP_TASK_H4 ||
            comp_task == COMP_TASK_H5 ||
            comp_task == COMP_TASK_H6)
        {
            if (comp_task == COMP_TASK_H4 && !comp_h4_counting)
            {
                comp_track_speed_rpm = 0.0f;
                return COMP_CHASSIS_STOP;
            }
            if (comp_task == COMP_TASK_H5 && !comp_timing_started)
            {
                comp_track_speed_rpm = 0.0f;
                return COMP_CHASSIS_STOP;
            }

            if (comp_task == COMP_TASK_H4)
            {
                target_speed = COMP_H4_SPEED_RPM;
            }
            else if (comp_task == COMP_TASK_H5)
            {
                target_speed = COMP_H5_SPEED_RPM;
            }
            else
            {
                target_speed = COMP_H6_SPEED_RPM;
            }
            if (comp_ramp_can_advance() && comp_ramp_progress < 1.0f)
            {
                comp_ramp_progress += ramp_step;
                if (comp_ramp_progress > 1.0f)
                {
                    comp_ramp_progress = 1.0f;
                }
            }
            comp_track_speed_rpm = target_speed * comp_smoothstep(comp_ramp_progress);
            return COMP_CHASSIS_TRACK;
        }
    }

    if (comp_state == COMP_STATE_DECELERATING)
    {
        float decel_step = (comp_task == COMP_TASK_H4)
                         ? (20.0f / (float)COMP_H4_DECEL_TIME_MS)
                         : ramp_step;
        uint8_t decel_can_advance = (comp_task == COMP_TASK_H4)
                                  ? 1U : comp_ramp_can_advance();

        if (decel_can_advance && comp_ramp_progress < 1.0f)
        {
            comp_ramp_progress += decel_step;
            if (comp_ramp_progress > 1.0f)
            {
                comp_ramp_progress = 1.0f;
            }
        }
        comp_track_speed_rpm = comp_decel_start_rpm *
                               (1.0f - comp_smoothstep(comp_ramp_progress));
        comp_update_h5_stop_ff_tail(now, left_rpm, right_rpm);
        if (comp_ramp_progress >= 1.0f)
        {
            if (comp_task == COMP_TASK_H4 && !comp_h4_passed)
            {
                comp_track_speed_rpm = COMP_H4_CRAWL_SPEED_RPM;
                return COMP_CHASSIS_TRACK;
            }
            comp_track_speed_rpm = 0.0f;
            if (comp_task == COMP_TASK_H5 && !comp_h5_stop_ff_complete)
            {
                /* 目标速度已经为零，但车轮仍在滑行时保持停车前馈尾段。
                 * 此处返回STOP，只断PWM，不让轮速PI产生反向制动。 */
                return COMP_CHASSIS_STOP;
            }
            if (comp_task == COMP_TASK_H4 &&
                (comp_absf(left_rpm) > COMP_BRAKE_STOP_RPM ||
                 comp_absf(right_rpm) > COMP_BRAKE_STOP_RPM))
            {
                /* 目标速度到0后只断PWM自然滑停，不调用速度环产生反向力。 */
                return COMP_CHASSIS_STOP;
            }
            comp_state = COMP_STATE_DONE;
            comp_oled_force = 1U;
            return COMP_CHASSIS_STOP;
        }
        return COMP_CHASSIS_TRACK;
    }

    comp_track_speed_rpm = 0.0f;
    return COMP_CHASSIS_STOP;
}

float Competition_GetTrackSpeedRPM(void)
{
    return comp_track_speed_rpm;
}

uint8_t Competition_CommandFeedforwardActive(void)
{
    uint8_t active_state =
        comp_state == COMP_STATE_RUNNING ||
        comp_state == COMP_STATE_CROSSING_FINISH ||
        comp_state == COMP_STATE_DECELERATING;

    /* 仅行车稳球题启用。H3由题目白名单和冲刺模式双重隔离。 */
    return active_state &&
           (comp_task == COMP_TASK_H4 ||
            comp_task == COMP_TASK_H5 ||
            comp_task == COMP_TASK_H6);
}

float Competition_GetCommandFeedforwardGain(void)
{
    if (comp_task == COMP_TASK_H4)
    {
        return COMP_H4_CMD_FF_GAIN_DEG_PER_RPM_S;
    }
    if (comp_task == COMP_TASK_H5)
    {
        return COMP_H5_CMD_FF_GAIN_DEG_PER_RPM_S;
    }
    if (comp_task == COMP_TASK_H6)
    {
        return COMP_H6_CMD_FF_GAIN_DEG_PER_RPM_S;
    }
    return 0.0f;
}

float Competition_AdjustCommandFeedforwardDeg(float planned_deg)
{
    float floor_deg = comp_h5_stop_ff_floor_deg;

    if (floor_deg < 0.0f && planned_deg > floor_deg)
    {
        return floor_deg;
    }
    return planned_deg;
}

uint8_t Competition_BallTrackProfileActive(void)
{
    uint8_t active_state =
        comp_state == COMP_STATE_RUNNING ||
        comp_state == COMP_STATE_CROSSING_FINISH ||
        comp_state == COMP_STATE_DECELERATING;

    return active_state &&
           (comp_task == COMP_TASK_H5 || comp_task == COMP_TASK_H6);
}

uint8_t Competition_PeriodicDebugEnabled(void)
{
    /* 位置环标定期间始终保留USART1遥测，待机和H2/H4也必须可观测。 */
    return 1U;
}

void Competition_TrackLost(void)
{
    comp_abort(HAL_GetTick());
}
