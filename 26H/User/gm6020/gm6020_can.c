/* GM6020 CAN 驱动 —— 电流模式测试版
 * 移植自 ytbeif 项目，去掉 PID 依赖，只保留通信收发
 */
#include "gm6020_can.h"

static CAN_FilterTypeDef  sFilterConfig;
static CAN_TxHeaderTypeDef TxMessage;
static CAN_RxHeaderTypeDef RxMessage;

uint8_t TxData[8];
static uint8_t aData[8];
static uint32_t TxMailbox;

volatile uint32_t can_tx_err_count = 0;
volatile Motor_Date motor_date[4];

/* 过滤器全通过 + 启动CAN + 开FIFO0接收中断 */
void Configure_Filter(void)
{
    sFilterConfig.FilterIdHigh         = 0x0000;
    sFilterConfig.FilterIdLow          = 0x0000;
    sFilterConfig.FilterMaskIdHigh     = 0x0000;   // 掩码全0 = 不筛，全收
    sFilterConfig.FilterMaskIdLow      = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    sFilterConfig.FilterBank           = 0;        // CAN1 用 0~13
    sFilterConfig.FilterScale          = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterMode           = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterActivation     = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_CAN_Start(&hcan1) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        Error_Handler();
    }

    /* 测试阶段偏移设0，直接看 0~8191 原始角度 */
    for (uint8_t i = 0; i < 4; i++)
    {
        motor_date[i].offset_angle = 0;
    }
}

/* 一帧同时控制 ID1~ID4 四个电机
 * mode = Voltage(0x1FF) 或 Current(0x1FE)
 * ⚠️ 模式发错电机会橙灯常亮报警，不会动
 */
void CAN_Transmit(int16_t data1, int16_t data2, int16_t data3, int16_t data4, uint8_t mode)
{
    memset(&TxMessage, 0, sizeof(TxMessage));

    switch (mode)
    {
        case Voltage: TxMessage.StdId = 0x1FF; break;
        case Current: TxMessage.StdId = 0x1FE; break;
        default: return;                        // 非法mode直接丢弃
    }

    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 8;

    TxData[0] = (uint8_t)(data1 >> 8);   TxData[1] = (uint8_t)(data1);
    TxData[2] = (uint8_t)(data2 >> 8);   TxData[3] = (uint8_t)(data2);
    TxData[4] = (uint8_t)(data3 >> 8);   TxData[5] = (uint8_t)(data3);
    TxData[6] = (uint8_t)(data4 >> 8);   TxData[7] = (uint8_t)(data4);

    /* 邮箱满说明总线有问题（线松/电机断电）
     * 这里只计数不死机，否则总线一断整机失联 */
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0U)
    {
        can_tx_err_count++;
        return;
    }
    if (HAL_CAN_AddTxMessage(&hcan1, &TxMessage, TxData, &TxMailbox) != HAL_OK)
    {
        can_tx_err_count++;
    }
}

/* 反馈报文接收中断，电机以 1kHz 主动上报 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    uint8_t MotorID;

    if (hcan->Instance != CAN1) return;
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxMessage, aData) != HAL_OK) return;

    switch (RxMessage.StdId)
    {
        case CAN_6020Moto1_ID: MotorID = 0; break;
        case CAN_6020Moto2_ID: MotorID = 1; break;
        case CAN_6020Moto3_ID: MotorID = 2; break;
        case CAN_6020Moto4_ID: MotorID = 3; break;
        default: return;
    }

    volatile Motor_Date *m = &motor_date[MotorID];

    /* 角度：0~8191 对应 0~360° */
    m->rotor_angle     = (int16_t)(((aData[0] << 8) | aData[1]) - m->offset_angle);
    m->rotor_du_angle  = m->rotor_angle * DEG_PER_TICK;
    m->rotor_rad_angle = m->rotor_angle * RAD_PER_TICK;

    /* 转速：有符号16位，单位 rpm */
    m->rotor_speed     = (int16_t)((aData[2] << 8) | aData[3]);
    m->rotor_du_speed  = m->rotor_speed * RPM_TO_DEGS;
    m->rotor_rad_speed = m->rotor_speed * RPM_TO_RADS;

    m->torque_current    = (int16_t)((aData[4] << 8) | aData[5]);
    m->motor_temperature = aData[6];

    /* 多圈计数：跨过 0/8191 边界时角度会突跳半圈以上 */
    int16_t delta = m->rotor_angle - m->rotor_last_angle;
    if      (delta >  4096) m->turn_count--;
    else if (delta < -4096) m->turn_count++;

    m->total_ticks     = m->turn_count * 8192 + m->rotor_angle;
    m->total_du        = m->total_ticks * DEG_PER_TICK;
    m->total_rad       = m->total_ticks * RAD_PER_TICK;
    m->rotor_last_angle = m->rotor_angle;

    m->rx_count++;   // 通信活着的证据
}
