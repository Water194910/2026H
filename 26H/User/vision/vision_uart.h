#ifndef __VISION_UART_H
#define __VISION_UART_H

#include <stdint.h>
#include "usart.h"

/* 视觉(MaixCam) → USART2
 *
 * 协议（main.py）：
 *   找到球时发一行 ASCII："P,+球位置,+目标位置\n"
 *   例如："P,+12.34,+63.50\n"
 *   两个位置均相对绿色水管中心，单位mm，右正左负。
 *   兼容旧版单数字位置帧，但旧帧不更新target_mm。
 *   丢球时不发 —— 靠超时判定
 *
 * 接收：DMA + UART IDLE（HAL_UARTEx_ReceiveToIdle_DMA）
 *   变长行帧，IDLE 一到就收齐一整行，比定长 DMA 合适
 */

#define VISION_RX_BUF_SIZE   48U
#define VISION_TIMEOUT_MS    200U   /* 超时认为丢球 */

typedef struct
{
    volatile float    distance_mm;   /* 最新球位置 mm，右正左负 */
    volatile float    target_mm;     /* MaixCam2确认的H6目标位置 mm */
    volatile uint8_t  target_valid;  /* 1=至少收到过一帧目标位置 */
    volatile uint8_t  valid;         /* 1=当前帧有效 */
    volatile uint8_t  connected;     /* 1=超时内收到过数据 */
    volatile uint32_t rx_count;      /* 成功解析帧数 */
    volatile uint32_t target_rx_count; /* 成功接收目标位置帧数 */
    volatile uint32_t err_count;     /* 解析失败 / 超长 */
    volatile uint32_t last_rx_ms;    /* 最近一帧 HAL_GetTick */
    volatile float    last_raw;      /* 最近解析值，便于调试 */
} Vision_Data_t;

/* 控制环使用的一致性快照，避免同时读到不同视觉帧的字段。 */
typedef struct
{
    float    distance_mm;
    uint8_t  valid;
    uint8_t  connected;
    uint32_t rx_count;
    uint32_t last_rx_ms;
} Vision_Snapshot_t;

extern Vision_Data_t vision;

void Vision_Init(void);              /* main 里调一次，开 DMA+IDLE */
void Vision_IdleCallback(uint16_t size); /* 中断里：拷贝长度后解析 */
void Vision_Poll(void);              /* 主循环：超时清 connected */
uint8_t Vision_GetSnapshot(Vision_Snapshot_t *snapshot);

#endif /* __VISION_UART_H */
