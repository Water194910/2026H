#ifndef __GM6020_CAN_H
#define __GM6020_CAN_H

#include <stdint.h>
#include <string.h>
#include "can.h"
#include "main.h"

/* ── 控制模式 ────────────────────────────────────────────────
 * Voltage：发 0x1FF，电机内部走速度环，出厂默认模式
 * Current：发 0x1FE，转矩电流直接给定（开环恒力矩）
 *          需固件 >= 1.0.11.2 且在 RM Assistant 里打开电流环开关
 * ─────────────────────────────────────────────────────────── */
#define Voltage 0
#define Current 1

/* 电流给定范围：-16384 ~ 16384  对应  -3A ~ 3A */
#define MOTOR_Current_MAX            16384
#define MOTOR_Current_MIN           (-16384)
/* 转矩常数 741 mN·m/A，满量程 3A 约 2.22 N·m（超额定，别长时间跑） */
#define CURRENT_LSB_PER_AMP          5461.3f   // 16384 / 3.0

/* 单位换算 */
#define ENCODER_RESOLUTION           8192.0f
#define RPM_TO_DEGS                  6.0f
#define RPM_TO_RADS                  0.104719755f
#define RAD_PER_TICK                (6.2831853f / ENCODER_RESOLUTION)
#define DEG_PER_TICK                (360.0f / ENCODER_RESOLUTION)

/* 反馈报文标识符 = 0x204 + 拨码ID */
typedef enum
{
    CAN_6020Moto1_ID = 0x205,
    CAN_6020Moto2_ID = 0x206,
    CAN_6020Moto3_ID = 0x207,
    CAN_6020Moto4_ID = 0x208,
} CAN_Message_ID;

/* ── 电机反馈数据 ───────────────────────────────────────────── */
typedef struct
{
    int16_t  rotor_angle;        // 机械角度原始值，减去 offset_angle 后的结果
    int16_t  rotor_last_angle;   // 上一帧角度，用于多圈计数
    float    rotor_du_angle;     // 机械角度（度）
    float    rotor_rad_angle;    // 机械角度（弧度）

    int16_t  rotor_speed;        // 转速（rpm）← CAN原始单位，PID优先用这个
    float    rotor_du_speed;     // 转速（度/秒）= rpm × 6
    float    rotor_rad_speed;    // 转速（弧度/秒）

    int16_t  torque_current;     // 实际转矩电流（同样是 -16384~16384 量纲）
    uint8_t  motor_temperature;  // 电机温度（℃）

    int32_t  turn_count;         // 累计圈数
    int32_t  total_ticks;        // 多圈总角度（tick）
    float    total_du;           // 多圈总角度（度）
    float    total_rad;          // 多圈总角度（弧度）

    uint16_t offset_angle;       // 机械零点偏移，测试阶段设0看原始值(0~8191)
    uint32_t rx_count;           // 收到反馈的帧数，用来判断通信通没通
} Motor_Date;

extern volatile Motor_Date motor_date[4];
extern volatile uint32_t   can_tx_err_count;   // 发送失败计数
extern uint8_t             TxData[8];

void Configure_Filter(void);
void CAN_Transmit(int16_t data1, int16_t data2, int16_t data3, int16_t data4, uint8_t mode);

#endif
