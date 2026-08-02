#ifndef __PID_H
#define __PID_H

#include <stdint.h>

/* ── GM6020 速度环 PID ──────────────────────────────────────────
 *
 * ⚠️ 速度单位统一用 RPM（CAN 反馈原始单位），不要用 度/秒
 *    两者差 6 倍，同样 Kp 用错单位等效增益就是 6 倍，会剧烈振荡
 *    → 输入必须是 motor_date[x].rotor_speed，不是 rotor_du_speed
 *
 * ⚠️ 本实现是"位置式PID"，积分项直接累加误差、没有乘 dt
 *    所以参数和调用周期是绑死的：Kp/Ki/Kd 只在 2ms 周期下成立
 *    以后改了定时器周期，Ki 和 Kd 必须重调
 * ─────────────────────────────────────────────────────────── */

/* 输出限幅：电压模式给定范围 ±25000 */
#define PID_VOLTAGE_MAX          25000.0f
#define PID_VOLTAGE_MIN         (-25000.0f)

/* 积分限幅（抗积分饱和）
 * 速度环默认已改为 P55/I0.55/D0；I 变小后同样 12000 限幅更温和。
 * 若仍"晃完回不来"，可再降 I 或略降本限幅 */
#define PID_ERR_SUM_MAX          12000.0f
#define PID_ERR_SUM_MIN         (-12000.0f)

/* 速度环默认（2ms）：用户联调值 P55 I0.55 D0 */
#define SUDU_KP_DEFAULT          55.0f
#define SUDU_KI_DEFAULT          0.55f
#define SUDU_KD_DEFAULT          0.0f

/* ── 角度软限位（手动标定实测值）───────────────────────────────
 * raw 5702 ~ 7557  →  250.6° ~ 332.1°，行程 1855 tick ≈ 81.5°
 *
 * 两端都落在 0~8191 内、且 min<max，说明行程不跨 0/8191 断点，
 * 所以直接比 rotor_angle 就行：不用圆周归一化，也不用 turn_count
 *
 * RUAN_* 是软限位，从机械极限往里退 YULIANG 个tick 当刹车距离；
 * MIN/MAX 本身当硬限位，只在发CAN前做最后兜底
 * ─────────────────────────────────────────────────────────── */
#define JIAODU_MIN         5702
#define JIAODU_MAX         7557
#define JIAODU_YULIANG     100                            // ≈4.4°
#define JIAODU_RUAN_MIN   (JIAODU_MIN + JIAODU_YULIANG)   // 5802
#define JIAODU_RUAN_MAX   (JIAODU_MAX - JIAODU_YULIANG)   // 7457

/* ── 角度环（外环）─────────────────────────────────────────────
 * 水平零点：raw=6529（实测 T1.6 时球停最接近 0mm，把 1.6°吃进零点）。
 *   旧值6493偏了1.6°=36tick，会把可用倾角范围整体推偏，
 *   造成一侧推力不足、球在中心附近难以回正。
 * ⚠️ 改这个宏不一定生效：main 里先装宏、紧接着 FlashParams_Load
 *    会用 flash 存档覆盖。flash 里有旧值时要发 PH 命令再 W 存档。
 * 工作半宽：±20°（相对水平零点）→ raw 6062~6972，软限位 5802~7457 内
 * 周期：TIM7 5ms；输出目标 RPM 交给 2ms 速度环
 * T 命令给的是相对水平的倾角(度)，内部再换成 raw
 * ─────────────────────────────────────────────────────────── */
#define JIAODU_PINGHENG_RAW      6517
#define JIAODU_GONGZUO_DU        20.0f
#define JIAODU_TICK_PER_DEG      (8192.0f / 360.0f)   // ≈22.756
#define JIAODU_GONGZUO_TICK      (JIAODU_GONGZUO_DU * JIAODU_TICK_PER_DEG)
#define JIAODU_RPM_MAX           40.0f   // 角度环输出转速限幅
#define JIAODU_RATE_FF_GAIN      0.0f    // 实机逐拍差分引发颤抖，先关闭并保留实现
#define JIAODU_RATE_FF_RPM_MAX   34.0f   // 前馈单项限幅，给角度反馈保留余量
#define JIAODU_DEG_S_PER_RPM     6.0f    // 1 RPM = 6 deg/s
#define JIAODU_RATE_FF_TIMEOUT_MS 10U    // 超时后仅同步目标，禁止重启尖峰
#define JIAODU_KP_DEFAULT        0.28f   // 无抖动实机联调：兼顾40ms跟随与小球回中
#define JIAODU_KI_DEFAULT        0.0f    // 先不加积分
#define JIAODU_KD_DEFAULT        0.06f   // 0.08滞后和越界反弹，保留0.06

/* 上下限夹紧 */
#define PID_LIMIT_MIN_MAX(x, min, max) \
    ((x) <= (min) ? (min) : ((x) >= (max) ? (max) : (x)))

typedef struct
{
    float Kp;
    float Ki;
    float Kd;

    float target_val;    // 目标值
    float actual_val;    // 实测值
    float err;           // 本次误差
    float err_last;      // 上次误差（微分用）
    float err_sum;       // 误差累积（积分用）
    float output;        // 上次输出，方便Watch窗口观察
} PID_t;

typedef PID_t *PID_Handle_t;

void  PID_Init(PID_Handle_t pid, float kp, float ki, float kd);
void  PID_Reset(PID_Handle_t pid);
float PID_SpeedLoop(PID_Handle_t pid, float actual_val, float target_val);

#endif
