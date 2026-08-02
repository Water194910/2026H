/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/*
                   _ooOoo_
                  o8888888o
                  88" . "88
                  (| -_- |)
                  O\  =  /O
               ____/`---'\____
             .'  \\|     |//  `.
            /  \\|||  :  |||//  \
           /  _||||| -:- |||||-  \
           |   | \\\  -  /// |   |
           | \_|  ''\---/''  |   |
           \  .-\__  `-`  ___/-. /
         ___`. .'  /--.--\  `. . __
      ."" '<  `.___\_<|>_/___.'  >'"".
     | | :  `- \`.;`\ _ /`;.`/ - ` : | |
     \  \ `-.   \_ __\ /__ _/   .-` /  /
======`-.____`-.___\_____/___.-`____.-'======
                   `=---='
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
            佛祖保佑       永无BUG
*/
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "gm6020_can.h"
#include "debug_uart.h"
#include "pid.h"
#include "flash_params.h"
#include "chassis_pid.h"
#include "motor.h"
#include "gray_sensor.h"
#include "oled.h"
#include "vision_uart.h"
#include "qiu_weizhi.h"
#include "bno085_shtp.h"
#include "competition.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* chassis tracking (ONE), TIM5 20ms
 * 基础速度不再是常量：改由 competition 按任务/斜坡给
 * （Competition_GetTrackSpeedRPM），起停才能缓合。 */
#define TRACK_SENSOR_REVERSED  0
#define LEFT_ENCODER_SIGN      (-1)
#define RIGHT_ENCODER_SIGN     1

#define PID_DEBUG_DISABLED        0
#define PID_DEBUG_LEFT            1
#define PID_DEBUG_RIGHT           2
#define PID_DEBUG_BOTH            3
#define PID_DEBUG_MOTOR           PID_DEBUG_DISABLED
#define PID_DEBUG_TARGET_RPM      230.0f
/* OLED 刷新和按键消抖都搬进 competition 了，这里不再需要。
 * KEY1~4 现在是比赛菜单键（上一项/下一项/启动/停止），
 * 不再是电机启停——手动启停请用串口 E/S。 */

#define ENCODER_COUNTS_PER_REV 28.0f
#define MOTOR_GEAR_RATIO       13.0f
#define CONTROL_PERIOD_SECONDS 0.020f
#define ENCODER_RPM_PER_COUNT  \
  (60.0f / (ENCODER_COUNTS_PER_REV * MOTOR_GEAR_RATIO * CONTROL_PERIOD_SECONDS))

/* 平台角度已到位且转子停住时，逐拍释放速度环历史积分。
 * 避免角度环目标归零后仍带着上一段运动的保持电压。 */
#define SUDU_JINGZHI_MUBIAO_RPM  0.30f
#define SUDU_JINGZHI_SHICE_RPM   1.00f
#define SUDU_JIFEN_SHUAIJIAN     0.95f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile float   mubiao_sudu    = 0.0f;
volatile uint8_t dianji_shineng = 1;
PID_t sudu_pid;                 /* 速度环 2ms */
PID_t jiaodu_pid;               /* 角度环 5ms */

/* 0=纯速度(V)  1=角度环(T)：T 给相对水平倾角(度) */
volatile uint8_t  jiaodu_moshi     = 0;
volatile float    mubiao_qingjiao  = 0.0f;   /* 目标倾角 °，T 命令写入 */
volatile float    mubiao_raw_f     = (float)JIAODU_PINGHENG_RAW;
volatile float    jiaodu_rpm_cmd   = 0.0f;   /* 角度环输出的转速指令 */

volatile int16_t  dianya_shuchu = 0;
volatile uint32_t fasong_count  = 0;
volatile int8_t   zhuanxiang   = 1;
volatile int8_t   xianwei_zt   = 0;
volatile uint32_t xianwei_cnt  = 0;

static GraySensor_HandleTypeDef tracking_sensor;
#if PID_DEBUG_MOTOR == PID_DEBUG_DISABLED
static uint32_t tracking_lost_time = 0U;
static uint8_t tracking_state = TRACK_STATE_NORMAL;
#endif
volatile int32_t encoder_left_count = 0;
volatile int32_t encoder_right_count = 0;
volatile float motor_rpm_left = 0.0f;
volatile float motor_rpm_right = 0.0f;
volatile int16_t debug_left_pwm = 0;
volatile int16_t debug_right_pwm = 0;
volatile float competition_target_rpm = 0.0f;
volatile float competition_target_accel_peak_rpm_s = 0.0f;
BNO085_SHTP_Data_t bno085_shtp_data = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static float jiaodu_xianwei(float mubiao)
{
    int16_t dangqian = motor_date[0].rotor_angle;
    float wai_zheng = mubiao * (float)zhuanxiang;

    if (dangqian >= JIAODU_RUAN_MAX && wai_zheng > 0.0f)
    {
        xianwei_zt = 1;
        xianwei_cnt++;
        PID_Reset(&sudu_pid);
        return 0.0f;
    }
    if (dangqian <= JIAODU_RUAN_MIN && wai_zheng < 0.0f)
    {
        xianwei_zt = -1;
        xianwei_cnt++;
        PID_Reset(&sudu_pid);
        return 0.0f;
    }

    xianwei_zt = 0;
    return mubiao;
}

/* 倾角(°) → 目标 raw，夹在工作区（Flash 可改 jiaodu_gongzuo_du）且不超出软限 */
static float qingjiao_dao_raw(float qingjiao_du)
{
    float raw;
    float lo;
    float hi;
    float banjing = jiaodu_gongzuo_du;

    if (banjing < 0.1f)
    {
        banjing = JIAODU_GONGZUO_DU;
    }
    if (qingjiao_du > banjing)
    {
        qingjiao_du = banjing;
    }
    if (qingjiao_du < -banjing)
    {
        qingjiao_du = -banjing;
    }

    raw = (float)jiaodu_pingheng_raw + qingjiao_du * JIAODU_TICK_PER_DEG;
    lo  = (float)JIAODU_RUAN_MIN;
    hi  = (float)JIAODU_RUAN_MAX;
    if (raw < lo)
    {
        raw = lo;
    }
    if (raw > hi)
    {
        raw = hi;
    }
    return raw;
}

/* 角度环 5ms：目标 raw vs 实测 raw → 目标 RPM（写入 mubiao_sudu） */
static void jiaodu_huan_gengxin(void)
{
    static float shangci_mubiao_raw = 0.0f;
    static uint32_t shangci_ms = 0U;
    static uint8_t qian_kui_youxiao = 0U;
    float dangqian;
    float fankui_out;
    float qian_kui_rpm = 0.0f;
    float out;
    uint32_t now_ms;

    mubiao_raw_f = qingjiao_dao_raw(mubiao_qingjiao);
    dangqian = (float)motor_date[0].rotor_angle;
    now_ms = HAL_GetTick();

    /* 对最终限幅目标求速度，直接给出电机所需RPM。首次运行或停顿后
     * 只同步历史目标，避免停机期间积累的目标差在重启时形成尖峰。 */
    if (qian_kui_youxiao)
    {
        uint32_t dt_ms = now_ms - shangci_ms;

        if (dt_ms > 0U && dt_ms <= JIAODU_RATE_FF_TIMEOUT_MS)
        {
            float mubiao_sulv_deg_s =
                ((mubiao_raw_f - shangci_mubiao_raw) / JIAODU_TICK_PER_DEG) *
                (1000.0f / (float)dt_ms);

            qian_kui_rpm = mubiao_sulv_deg_s /
                           JIAODU_DEG_S_PER_RPM * JIAODU_RATE_FF_GAIN;
            qian_kui_rpm = PID_LIMIT_MIN_MAX(qian_kui_rpm,
                                              -JIAODU_RATE_FF_RPM_MAX,
                                               JIAODU_RATE_FF_RPM_MAX);
        }
    }
    shangci_mubiao_raw = mubiao_raw_f;
    shangci_ms = now_ms;
    qian_kui_youxiao = 1U;

    /* 复用 PID_SpeedLoop 形：actual/target 单位都是 raw tick
     * 输出当作 RPM。Ki 默认 0。近零误差仍连续输出，不能硬清零，
     * 否则平台会在到位边界失去制动并形成粘滑。增益按5ms调。 */
    fankui_out = PID_SpeedLoop(&jiaodu_pid, dangqian, mubiao_raw_f);
    out = fankui_out + qian_kui_rpm;
    out = PID_LIMIT_MIN_MAX(out, -JIAODU_RPM_MAX, JIAODU_RPM_MAX);

    jiaodu_rpm_cmd = out;
    mubiao_sudu = out;
}

/* MotorButtons_Update / OLED_UpdateMotorRPM 已移除：
 * KEY1~4 和 OLED 现在归 competition 模块（比赛菜单+计时+球位置），
 * 底盘启停由 Competition_ChassisTick20ms 的返回值决定。
 * 手动启停电机改用串口 E / S。 */
/* TIM6 2ms 速度环+CAN；TIM7 5ms 角度环+打印分频；TIM5 20ms 底盘
 *
 * 无 CAN 反馈时绝不闭环：角度恒为0，限位会失真。
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6)
    {
        uint8_t fankui_huo = (motor_date[0].rx_count > 0U);

        if (dianji_shineng && fankui_huo)
        {
            float mubiao_anquan = jiaodu_xianwei(mubiao_sudu);
            float shice_sudu = (float)motor_date[0].rotor_speed;

            /* 只在平台既不要求转动、转子也已静止时衰减旧积分。
             * 运动中不碰积分，避免损失跟随能力。 */
            if (mubiao_anquan > -SUDU_JINGZHI_MUBIAO_RPM &&
                mubiao_anquan <  SUDU_JINGZHI_MUBIAO_RPM &&
                shice_sudu > -SUDU_JINGZHI_SHICE_RPM &&
                shice_sudu <  SUDU_JINGZHI_SHICE_RPM)
            {
                sudu_pid.err_sum *= SUDU_JIFEN_SHUAIJIAN;
            }

            float out = PID_SpeedLoop(&sudu_pid, shice_sudu, mubiao_anquan);
            dianya_shuchu = (int16_t)out;

            int16_t jiao = motor_date[0].rotor_angle;
            float  xiang = (float)dianya_shuchu * (float)zhuanxiang;
            if ((jiao >= JIAODU_MAX && xiang > 0.0f) ||
                (jiao <= JIAODU_MIN && xiang < 0.0f))
            {
                dianya_shuchu = 0;
            }
        }
        else
        {
            PID_Reset(&sudu_pid);
            dianya_shuchu = 0;
            if (jiaodu_moshi)
            {
                PID_Reset(&jiaodu_pid);
                jiaodu_rpm_cmd = 0.0f;
            }
            /* 位置环也一起清：停机期间积分照攒，E1 那一刻会猛甩一下 */
            if (qiu_moshi)
            {
                QiuWeizhi_Reset();
            }
        }

        CAN_Transmit(dianya_shuchu, 0, 0, 0, Voltage);
        fasong_count++;
    }
    else if (htim->Instance == TIM7)
    {
        /* 灰度标记检测要 5ms 拍，比 20ms 底盘环快，才不会漏掉窄标记 */
        Competition_MarkerTick5ms();

        /* 5ms位置环服务：新视觉帧才入队，每2帧算一次外环；
         * 其余拍只把倾角命令按M限速送给200Hz角度环。 */
        if (qiu_moshi && dianji_shineng &&
            (motor_date[0].rx_count > 0U))
        {
            QiuWeizhi_Update();
        }

        /* 5ms：角度模式且有反馈时跑外环 */
        if (jiaodu_moshi && dianji_shineng &&
            (motor_date[0].rx_count > 0U))
        {
            jiaodu_huan_gengxin();
        }

        /* 位置环调试打印10Hz；调参固件在待机及所有比赛任务中均开启。 */
        {
            static uint8_t dayin_fenpin = 0;
            if (!dbg_stream_enable || !Competition_PeriodicDebugEnabled())
            {
                dayin_fenpin = 0;
                dbg_print_pending = 0;
            }
            else if (++dayin_fenpin >= QIU_DAYIN_FENPIN)
            {
                dayin_fenpin = 0;
                if (dbg_print_pending) dbg_print_lost++;
                else                   dbg_print_pending = 1;
            }
        }
    }
    else if (htim->Instance == TIM5)
    {
        Competition_ChassisMode competition_mode;
        int16_t competition_left_pwm = 0;
        int16_t competition_right_pwm = 0;
#if PID_DEBUG_MOTOR == PID_DEBUG_LEFT || PID_DEBUG_MOTOR == PID_DEBUG_RIGHT
        float debug_pwm;
#else
        float left_pwm;
        float right_pwm;
#endif

        encoder_left_count = LEFT_ENCODER_SIGN *
                             (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
        encoder_right_count = RIGHT_ENCODER_SIGN *
                              (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
        __HAL_TIM_SET_COUNTER(&htim2, 0U);
        __HAL_TIM_SET_COUNTER(&htim3, 0U);

        motor_rpm_left = (float)encoder_left_count * ENCODER_RPM_PER_COUNT;
        motor_rpm_right = (float)encoder_right_count * ENCODER_RPM_PER_COUNT;

        /* 底盘启停权交给 competition：STOP=停、DIRECT=直行给定PWM、
         * TRACK=往下走循迹。原来的 motor_run_enabled + KEY2 已撤。 */
        competition_mode = Competition_ChassisTick20ms(motor_rpm_left,
                                                        motor_rpm_right,
                                                        encoder_left_count,
                                                        encoder_right_count,
                                                        &competition_left_pwm,
                                                        &competition_right_pwm);
        {
            static float shangci_dipan_mubiao_rpm = 0.0f;
            static uint8_t dipan_yidong_yisuocun = 0U;
            float dangqian_dipan_mubiao_rpm = Competition_GetTrackSpeedRPM();
            float mubiao_jiasudu_rpm_s =
                (dangqian_dipan_mubiao_rpm - shangci_dipan_mubiao_rpm) / 0.020f;
            uint8_t cmd_ff_active = Competition_CommandFeedforwardActive();
            float cmd_ff_gain = Competition_GetCommandFeedforwardGain();
            float cmd_ff_deg = Competition_AdjustCommandFeedforwardDeg(
                cmd_ff_gain * mubiao_jiasudu_rpm_s);
            int32_t left_count_abs = (encoder_left_count >= 0)
                                   ? encoder_left_count : -encoder_left_count;
            int32_t right_count_abs = (encoder_right_count >= 0)
                                    ? encoder_right_count : -encoder_right_count;
            float abs_jiasudu = (mubiao_jiasudu_rpm_s >= 0.0f)
                              ? mubiao_jiasudu_rpm_s : -mubiao_jiasudu_rpm_s;
            float abs_fengzhi = (competition_target_accel_peak_rpm_s >= 0.0f)
                              ? competition_target_accel_peak_rpm_s
                              : -competition_target_accel_peak_rpm_s;

            competition_target_rpm = dangqian_dipan_mubiao_rpm;
            if (!cmd_ff_active)
            {
                dipan_yidong_yisuocun = 0U;
            }
            else if ((left_count_abs + right_count_abs) >= 2)
            {
                /* 两个编码器计数约等于平均轮速8.24RPM，确认底盘已克服
                 * 静摩擦后再允许命令前馈；锁存避免低速量化反复开关。 */
                dipan_yidong_yisuocun = 1U;
            }
            QiuWeizhi_SetChassisCommandAccelFF(
                (cmd_ff_gain != 0.0f) ? (cmd_ff_deg / cmd_ff_gain) : 0.0f,
                cmd_ff_gain,
                cmd_ff_active && dipan_yidong_yisuocun);
            if (abs_jiasudu > abs_fengzhi)
            {
                competition_target_accel_peak_rpm_s = mubiao_jiasudu_rpm_s;
            }
            shangci_dipan_mubiao_rpm = dangqian_dipan_mubiao_rpm;
        }
        if (competition_mode == COMP_CHASSIS_STOP)
        {
            Chassis_PID_Reset(&piderr);
            Chassis_PID_Reset(&pidMotor1Speed);
            Chassis_PID_Reset(&pidMotor2Speed);
            debug_left_pwm = 0;
            debug_right_pwm = 0;
            Set_pwm(0, 0);
            return;
        }
        if (competition_mode == COMP_CHASSIS_DIRECT)
        {
            Chassis_PID_Reset(&piderr);
            Chassis_PID_Reset(&pidMotor1Speed);
            Chassis_PID_Reset(&pidMotor2Speed);
            debug_left_pwm = competition_left_pwm;
            debug_right_pwm = competition_right_pwm;
            Set_pwm(competition_left_pwm, competition_right_pwm);
            return;
        }

#if PID_DEBUG_MOTOR == PID_DEBUG_LEFT
        debug_pwm = PID_realize(&pidMotor1Speed, motor_rpm_left);
        debug_left_pwm = (int16_t)debug_pwm;
        debug_right_pwm = 0;
        Set_pwm(debug_left_pwm, 0);
        return;
#elif PID_DEBUG_MOTOR == PID_DEBUG_RIGHT
        debug_pwm = PID_realize(&pidMotor2Speed, motor_rpm_right);
        debug_left_pwm = 0;
        debug_right_pwm = (int16_t)debug_pwm;
        Set_pwm(0, debug_right_pwm);
        return;
#elif PID_DEBUG_MOTOR == PID_DEBUG_BOTH
        left_pwm = PID_realize(&pidMotor1Speed, motor_rpm_left);
        right_pwm = PID_realize(&pidMotor2Speed, motor_rpm_right);
        debug_left_pwm = (int16_t)left_pwm;
        debug_right_pwm = (int16_t)right_pwm;
        Set_pwm(debug_left_pwm, debug_right_pwm);
        return;
#else
        GraySensor_Update(&tracking_sensor);
        XinZhiStateMachine(&tracking_sensor,
                           &piderr,
                           &pidMotor1Speed,
                           &pidMotor2Speed,
                           Competition_GetTrackSpeedRPM(),
                           &tracking_lost_time,
                           &tracking_state,
                           Competition_BallTrackProfileActive());

        if (tracking_state == TRACK_STATE_STOP)
        {
            Chassis_PID_Reset(&pidMotor1Speed);
            Chassis_PID_Reset(&pidMotor2Speed);
            Set_pwm(0, 0);
            /* 告知 competition 丢线，让它退出本圈而不是干等 */
            Competition_TrackLost();
            return;
        }

        left_pwm = PID_realize(&pidMotor1Speed, motor_rpm_left);
        right_pwm = PID_realize(&pidMotor2Speed, motor_rpm_right);
        Set_pwm((int)left_pwm, (int)right_pwm);
#endif
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        Debug_RxByte();
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */

  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  /* 尽早启动BNO085 DMA，避免后续OLED/Flash初始化期间错过冷启动广告。 */
  BNO085_SHTP_Init();
  Configure_Filter();

  /* 先装宏默认，再开机自动加载 Flash（有存档则覆盖） */
  PID_Init(&sudu_pid, SUDU_KP_DEFAULT, SUDU_KI_DEFAULT, SUDU_KD_DEFAULT);
  PID_Init(&jiaodu_pid, JIAODU_KP_DEFAULT, JIAODU_KI_DEFAULT, JIAODU_KD_DEFAULT);
  QiuWeizhi_Init();          /* 位置环宏默认，紧接着可被 Flash 覆盖 */
  jiaodu_pingheng_raw = JIAODU_PINGHENG_RAW;
  jiaodu_gongzuo_du   = JIAODU_GONGZUO_DU;
  (void)FlashParams_Load(&sudu_pid, &jiaodu_pid, &zhuanxiang);

  /* 位置环启动最大倾角与角度环工作半宽一致，旧 Flash 的 H 不覆盖此默认。 */
  qiu_qingjiao_max = jiaodu_gongzuo_du;

  mubiao_qingjiao = 0.0f;
  mubiao_raw_f = (float)jiaodu_pingheng_raw;
  /* 位置环和电机默认使能；无CAN反馈时速度环仍会保持零输出。 */
  jiaodu_moshi = 1;
  qiu_moshi = 1;
  dianji_shineng = 1;
  QiuWeizhi_Reset();

  Chassis_PID_init();
  OLED_Init();
#if PID_DEBUG_MOTOR == PID_DEBUG_LEFT
  pidMotor1Speed.target_val = PID_DEBUG_TARGET_RPM;
  pidMotor2Speed.target_val = 0.0f;
#elif PID_DEBUG_MOTOR == PID_DEBUG_RIGHT
  pidMotor1Speed.target_val = 0.0f;
  pidMotor2Speed.target_val = PID_DEBUG_TARGET_RPM;
#elif PID_DEBUG_MOTOR == PID_DEBUG_BOTH
  pidMotor1Speed.target_val = PID_DEBUG_TARGET_RPM;
  pidMotor2Speed.target_val = PID_DEBUG_TARGET_RPM;
#endif

#if TRACK_SENSOR_REVERSED
  GraySensor_Init(&tracking_sensor,
                  GPIOD, GPIO_PIN_7,
                  GPIOD, GPIO_PIN_6,
                  GPIOD, GPIO_PIN_5,
                  GPIOD, GPIO_PIN_4,
                  GPIOD, GPIO_PIN_3,
                  GPIOD, GPIO_PIN_2,
                  GPIOD, GPIO_PIN_1,
                  GPIOD, GPIO_PIN_0);
#else
  GraySensor_Init(&tracking_sensor,
                  GPIOD, GPIO_PIN_0,
                  GPIOD, GPIO_PIN_1,
                  GPIOD, GPIO_PIN_2,
                  GPIOD, GPIO_PIN_3,
                  GPIOD, GPIO_PIN_4,
                  GPIOD, GPIO_PIN_5,
                  GPIOD, GPIO_PIN_6,
                  GPIOD, GPIO_PIN_7);
#endif

  Set_pwm(0, 0);
  __HAL_TIM_SET_COUNTER(&htim2, 0U);
  __HAL_TIM_SET_COUNTER(&htim3, 0U);

  printf("\r\n# GM6020 ball cascade ready (vision/2 outer, 4-frame velocity)\r\n");
  printf("# qiu and motor enabled at boot; CAN feedback required for output\r\n");
  printf("# pos X=%.2f Y=%.3f Z=%.2f | vel C=%.3f O=%.3f\r\n",
         qiu_pid.Kp, qiu_pid.Ki, qiu_pid.Kd,
         (double)qiu_vel_kp, (double)qiu_vel_kd);
  printf("# G=%.1fmm H=±%.1fdeg U=%.1fmm M=%.0fdeg/s N=%d\r\n",
         (double)qiu_mubiao_mm, (double)qiu_qingjiao_max,
         (double)qiu_siqu_mm, (double)qiu_sulv_max, (int)qiu_fuhao);

  Vision_Init();   /* USART2 DMA+IDLE 收 MaixCam 球位置；USART3 留给 IMU */
  Competition_Init();  /* 比赛任务状态机：KEY1~4 菜单 + OLED + 底盘调度 */

  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_Base_Start_IT(&htim7);

  if (HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1) != HAL_OK ||
      HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2) != HAL_OK ||
      HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL) != HAL_OK ||
      HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL) != HAL_OK ||
      HAL_TIM_Base_Start_IT(&htim5) != HAL_OK)
  {
    Error_Handler();
  }

  Debug_RxStart();
  Debug_PrintParams();
  /* 开机首屏由 Competition_Init 里的 comp_render_oled 画 */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    Debug_ParseCommand();

    Vision_Poll();   /* 超时清 vision.connected */
    (void)BNO085_SHTP_Read(&bno085_shtp_data);

    if (dbg_print_pending)
    {
      if (dbg_stream_enable && Competition_PeriodicDebugEnabled())
      {
        dianji_dayin();
      }
      dbg_print_pending = 0;
    }

    /* 比赛状态机：按键消抖、任务推进、OLED 刷新都在里面 */
    Competition_Service();
    /* 位置环调试走 dianji_dayin()（TIM7→USART1）：
     * A0: 100ms 位置环数据行
     * A1: 原始角度/视觉；A2: IMU线加速度标定；Q 查询完整参数 */
  /* USER CODE END 3 */
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
