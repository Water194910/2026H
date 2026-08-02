/**
 * @file    bno085_shtp.h
 * @brief   BNO085 UART-SHTP 模式驱动
 *
 * 接线要求：
 *   BNO085 TX  -> MCU UART_RX (PB11=USART3_RX)
 *   BNO085 RX  <- MCU UART_TX (PB10=USART3_TX)
 *   BNO085 H_INTN -> PA4 (可选)
 *   PS0        -> GND
 *   PS1        -> VCC
 *   波特率 3Mbps，8N1，OVER8
 *   需要外部 32.768kHz 晶振
 */

#ifndef __BNO085_SHTP_H
#define __BNO085_SHTP_H

#include "main.h"

/* ======================== 移植配置区 ======================== */

#ifndef BNO085_UART_HANDLE
#define BNO085_UART_HANDLE      huart3
#endif

#ifndef BNO085_DMA_BUF_SIZE
#define BNO085_DMA_BUF_SIZE     512
#endif

/* ======================== SHTP 协议常量 ======================== */

#define SHTP_CH_COMMAND         0
#define SHTP_CH_EXECUTABLE      1
#define SHTP_CH_CONTROL         2
#define SHTP_CH_INPUT_REPORT    3
#define SHTP_CH_WAKE_REPORT     4
#define SHTP_CH_GYRO_RV         5

#define SENSOR_REPORTID_CAL_GYRO        0x02
#define SENSOR_REPORTID_LINEAR_ACCEL    0x04
#define SENSOR_REPORTID_ROTATION_VECTOR 0x05
#define SENSOR_REPORTID_GAME_RV         0x08

#define SHTP_CMD_SET_FEATURE    0xFD
#define SHTP_CMD_GET_FEATURE    0xFE
#define SHTP_CMD_PRODUCT_ID     0xF9

#define SHTP_ADV_PRODUCT_ID     0xF8

typedef enum {
    SHTP_STATE_WAIT_RESET = 0,
    SHTP_STATE_READY,
    SHTP_STATE_CONFIGURING,
    SHTP_STATE_RUNNING,
} BNO085_SHTP_State_t;

typedef struct {
    float gyro_x;            /* 角速度 X (deg/s) */
    float gyro_y;            /* 角速度 Y (deg/s) */
    float gyro_z;            /* 角速度 Z (deg/s) */
    float accel_x;           /* 去重力线加速度 X (m/s^2) */
    float accel_y;           /* 去重力线加速度 Y (m/s^2) */
    float accel_z;           /* 去重力线加速度 Z (m/s^2) */
    float yaw;               /* Game Rotation Vector (deg) */
    float pitch;
    float roll;
    float mag_yaw;           /* Rotation Vector，使用磁力计 (deg) */
    float mag_pitch;
    float mag_roll;
    uint8_t accuracy;
    uint8_t mag_accuracy;
    uint8_t accel_accuracy;
    uint8_t gyro_fresh;
    uint8_t accel_fresh;
    uint8_t euler_fresh;
    uint8_t mag_euler_fresh;
    uint8_t fresh;
    uint8_t state;
    uint32_t raw_bytes;
    uint32_t rx_bytes;
    uint16_t rx_packets;
    uint16_t rx_bad;
    uint16_t last_len;
    uint8_t last_channel;
    uint8_t last_report;
    uint16_t dma_pos;
    uint32_t idle_irq;
    uint32_t uart_error;
    uint32_t accel_updates;
    uint32_t accel_last_ms;
    uint8_t rx_status;
    uint16_t tx_space;
} BNO085_SHTP_Data_t;

void BNO085_SHTP_Init(void);
uint8_t BNO085_SHTP_Read(BNO085_SHTP_Data_t *data);
void BNO085_SHTP_IRQHandler(void);
void BNO085_SHTP_RestartRx(void);

#endif /* __BNO085_SHTP_H */
