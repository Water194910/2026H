/**
 * @file    bno085_shtp.c
 * @brief   BNO085 UART-SHTP mode driver
 *
 * UART-SHTP 使用 3Mbps、8N1。USART3 RX 必须使用循环 DMA。
 * UART 线上还有 0x7E/0x7D framing，不能直接把 DMA 数据当 SHTP 包。
 */

#include "bno085_shtp.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define bno085_uart BNO085_UART_HANDLE
extern UART_HandleTypeDef bno085_uart;

#define UART_FLAG_BYTE      0x7E
#define UART_ESCAPE_BYTE    0x7D
#define UART_ESCAPE_XOR     0x20
#define UART_PROTO_BSQ_BSN  0x00
#define UART_PROTO_SHTP     0x01

static uint8_t dma_buf[BNO085_DMA_BUF_SIZE];
static volatile uint8_t frame_ready;
static uint16_t last_read_pos;

static uint8_t uart_frame[512];
static uint16_t uart_frame_len;
static uint8_t uart_in_frame;
static uint8_t uart_escape;
static uint16_t uart_tx_space;
static uint32_t last_bsq_ms;
static uint32_t last_reset_ms;

static BNO085_SHTP_State_t drv_state = SHTP_STATE_WAIT_RESET;
static uint8_t tx_seq[6];
static volatile uint8_t tx_busy;

static BNO085_SHTP_Data_t shujv;
static uint8_t new_data;

static inline uint8_t dma_read(uint16_t pos)
{
    return dma_buf[pos % BNO085_DMA_BUF_SIZE];
}

static int16_t read_i16(const uint8_t *p)
{
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static void delay_us(uint32_t us);
static void uart_send_byte(uint8_t b);
static void uart_send_escaped(uint8_t b);
static void send_bsq(void);
static void send_soft_reset(void);
static void send_shtp_command(const uint8_t *payload, uint16_t len, uint8_t channel);
static void set_feature(uint8_t report_id, uint32_t interval_us);
static void enable_reports(void);
static void parse_rx_stream(void);
static void parse_uart_byte(uint8_t b);
static void parse_uart_frame(const uint8_t *frame, uint16_t len);
static void parse_shtp_packet(const uint8_t *pkt, uint16_t len);
static void parse_sensor_report(uint8_t channel, const uint8_t *payload, uint16_t len);
static void parse_linear_accel(const uint8_t *payload, uint16_t len);
static void parse_rotation_vector(const uint8_t *payload, uint16_t len,
                                  uint8_t use_magnetometer);
static void quat_to_euler(float qi, float qj, float qk, float qr,
                          float *yaw, float *pitch, float *roll);

void BNO085_SHTP_Init(void)
{
    memset(dma_buf, 0, sizeof(dma_buf));
    memset(&shujv, 0, sizeof(shujv));
    memset(tx_seq, 0, sizeof(tx_seq));

    frame_ready = 0;
    last_read_pos = 0;
    uart_frame_len = 0;
    uart_in_frame = 0;
    uart_escape = 0;
    uart_tx_space = 0;
    last_bsq_ms = 0;
    last_reset_ms = HAL_GetTick() - 1000u;
    drv_state = SHTP_STATE_WAIT_RESET;
    tx_busy = 0;
    new_data = 0;

    __HAL_UART_ENABLE_IT(&bno085_uart, UART_IT_IDLE);
    shujv.rx_status = HAL_UART_Receive_DMA(&bno085_uart, dma_buf, sizeof(dma_buf));
}

uint8_t BNO085_SHTP_Read(BNO085_SHTP_Data_t *data)
{
    uint32_t now;

    if (data == NULL) {
        return 0;
    }

    now = HAL_GetTick();

    if (bno085_uart.hdmarx != NULL) {
        shujv.dma_pos = (uint16_t)(sizeof(dma_buf) -
                                   __HAL_DMA_GET_COUNTER(bno085_uart.hdmarx));
    }
    shujv.uart_error = HAL_UART_GetError(&bno085_uart);

    if (frame_ready || shujv.dma_pos != last_read_pos) {
        frame_ready = 0;
        parse_rx_stream();
    }

    shujv.tx_space = uart_tx_space;

    if (drv_state == SHTP_STATE_WAIT_RESET && !tx_busy) {
        /* UART-SHTP冷启动先复位SH-2；仅发BSQ无法启动一个静默的传感器。 */
        if (now - last_reset_ms >= 1000u) {
            last_reset_ms = now;
            send_soft_reset();
        }
    } else if (drv_state == SHTP_STATE_READY && !tx_busy) {
        if (uart_tx_space >= 96u) {
            enable_reports();
            uart_tx_space = 0;
            drv_state = SHTP_STATE_CONFIGURING;
        } else {
            if (now - last_bsq_ms >= 20u) {
                last_bsq_ms = now;
                send_bsq();
            }
        }
    }

    if (new_data) {
        new_data = 0;
        shujv.state = drv_state;
        shujv.fresh = 1;
        *data = shujv;
        shujv.gyro_fresh = 0;
        shujv.accel_fresh = 0;
        shujv.euler_fresh = 0;
        shujv.mag_euler_fresh = 0;
        return 1;
    }

    shujv.state = drv_state;
    shujv.fresh = 0;
    *data = shujv;
    return 0;
}

void BNO085_SHTP_IRQHandler(void)
{
    if (__HAL_UART_GET_FLAG(&bno085_uart, UART_FLAG_IDLE)) {
        __HAL_UART_CLEAR_IDLEFLAG(&bno085_uart);
        shujv.idle_irq++;
        frame_ready = 1;
    }
}

void BNO085_SHTP_RestartRx(void)
{
    shujv.uart_error = HAL_UART_GetError(&bno085_uart);

    __HAL_UART_CLEAR_OREFLAG(&bno085_uart);
    __HAL_UART_CLEAR_NEFLAG(&bno085_uart);
    __HAL_UART_CLEAR_FEFLAG(&bno085_uart);
    (void)HAL_UART_AbortReceive(&bno085_uart);

    memset(dma_buf, 0, sizeof(dma_buf));
    last_read_pos = 0;
    frame_ready = 0;
    uart_frame_len = 0;
    uart_in_frame = 0;
    uart_escape = 0;

    __HAL_UART_ENABLE_IT(&bno085_uart, UART_IT_IDLE);
    shujv.rx_status = HAL_UART_Receive_DMA(&bno085_uart, dma_buf, sizeof(dma_buf));
}

static void parse_rx_stream(void)
{
    uint16_t write_pos;

    if (bno085_uart.hdmarx == NULL) {
        return;
    }

    write_pos = (uint16_t)(sizeof(dma_buf) -
                           __HAL_DMA_GET_COUNTER(bno085_uart.hdmarx));

    while (last_read_pos != write_pos) {
        parse_uart_byte(dma_read(last_read_pos));
        shujv.raw_bytes++;
        last_read_pos = (uint16_t)((last_read_pos + 1u) % BNO085_DMA_BUF_SIZE);
    }
}

static void parse_uart_byte(uint8_t b)
{
    if (b == UART_FLAG_BYTE) {
        if (uart_in_frame && uart_frame_len > 0u) {
            parse_uart_frame(uart_frame, uart_frame_len);
        }
        uart_in_frame = 1;
        uart_escape = 0;
        uart_frame_len = 0;
        return;
    }

    if (!uart_in_frame) {
        return;
    }

    if (b == UART_ESCAPE_BYTE) {
        uart_escape = 1;
        return;
    }

    if (uart_escape) {
        b ^= UART_ESCAPE_XOR;
        uart_escape = 0;
    }

    if (uart_frame_len < sizeof(uart_frame)) {
        uart_frame[uart_frame_len++] = b;
    } else {
        shujv.rx_bad++;
        uart_in_frame = 0;
        uart_escape = 0;
        uart_frame_len = 0;
    }
}

static void parse_uart_frame(const uint8_t *frame, uint16_t len)
{
    if (len < 1u) {
        return;
    }

    if (frame[0] == UART_PROTO_BSQ_BSN) {
        if (len >= 3u) {
            uart_tx_space = (uint16_t)frame[1] | ((uint16_t)frame[2] << 8);
            if (drv_state == SHTP_STATE_WAIT_RESET && uart_tx_space > 0u) {
                drv_state = SHTP_STATE_READY;
            }
        }
        return;
    }

    if (frame[0] == UART_PROTO_SHTP) {
        parse_shtp_packet(frame + 1, (uint16_t)(len - 1u));
    }
}

static void parse_shtp_packet(const uint8_t *pkt, uint16_t len)
{
    uint16_t pkt_len;
    uint16_t payload_len;
    uint8_t channel;
    const uint8_t *payload;

    if (len < 4u) {
        shujv.rx_bad++;
        return;
    }

    pkt_len = (uint16_t)pkt[0] | ((uint16_t)(pkt[1] & 0x7F) << 8);
    channel = pkt[2];

    if (pkt_len < 4u || pkt_len > len) {
        shujv.rx_bad++;
        return;
    }

    payload_len = (uint16_t)(pkt_len - 4u);
    payload = pkt + 4;

    shujv.rx_bytes += pkt_len;
    shujv.rx_packets++;
    shujv.last_len = pkt_len;
    shujv.last_channel = channel;
    shujv.last_report = payload_len > 0u ? payload[0] : 0u;

    if (drv_state == SHTP_STATE_WAIT_RESET) {
        drv_state = SHTP_STATE_READY;
    }

    switch (channel) {
    case SHTP_CH_EXECUTABLE:
        if (payload_len > 0u && payload[0] == 0x01u) {
            drv_state = SHTP_STATE_READY;
        }
        break;

    case SHTP_CH_INPUT_REPORT:
    case SHTP_CH_WAKE_REPORT:
    case SHTP_CH_GYRO_RV:
        parse_sensor_report(channel, payload, payload_len);
        break;

    default:
        break;
    }
}

static void parse_sensor_report(uint8_t channel, const uint8_t *payload, uint16_t len)
{
    uint8_t status;
    int16_t raw_x;
    int16_t raw_y;
    int16_t raw_z;
    const float q9_to_deg_s = (180.0f / M_PI) / 512.0f;

    (void)channel;

    while (len > 0u) {
        uint8_t report_id = payload[0];

        if (report_id == 0xFBu) {
            if (len < 5u) {
                return;
            }
            payload += 5;
            len = (uint16_t)(len - 5u);
            if (len > 0u) {
                shujv.last_report = payload[0];
            }
            continue;
        }

        if (report_id == SENSOR_REPORTID_GAME_RV) {
            parse_rotation_vector(payload, len, 0);
            return;
        }

        if (report_id == SENSOR_REPORTID_ROTATION_VECTOR) {
            parse_rotation_vector(payload, len, 1);
            return;
        }

        if (report_id == SENSOR_REPORTID_LINEAR_ACCEL) {
            parse_linear_accel(payload, len);
            return;
        }

        if (report_id != SENSOR_REPORTID_CAL_GYRO) {
            return;
        }

        break;
    }

    if (len < 10u) {
        return;
    }

    status = payload[2];
    raw_x = read_i16(payload + 4);
    raw_y = read_i16(payload + 6);
    raw_z = read_i16(payload + 8);

    shujv.gyro_x = (float)raw_x * q9_to_deg_s;
    shujv.gyro_y = (float)raw_y * q9_to_deg_s;
    shujv.gyro_z = (float)raw_z * q9_to_deg_s;
    shujv.accuracy = status & 0x03u;
    shujv.gyro_fresh = 1;
    new_data = 1;

    if (drv_state == SHTP_STATE_CONFIGURING) {
        drv_state = SHTP_STATE_RUNNING;
    }
}

static void parse_linear_accel(const uint8_t *payload, uint16_t len)
{
    const float q8_to_m_s2 = 1.0f / 256.0f;

    if (len < 10u) {
        return;
    }

    shujv.accel_x = (float)read_i16(payload + 4) * q8_to_m_s2;
    shujv.accel_y = (float)read_i16(payload + 6) * q8_to_m_s2;
    shujv.accel_z = (float)read_i16(payload + 8) * q8_to_m_s2;
    shujv.accel_accuracy = payload[2] & 0x03u;
    shujv.accel_fresh = 1;
    shujv.accel_updates++;
    shujv.accel_last_ms = HAL_GetTick();
    new_data = 1;

    if (drv_state == SHTP_STATE_CONFIGURING) {
        drv_state = SHTP_STATE_RUNNING;
    }
}

static void parse_rotation_vector(const uint8_t *payload, uint16_t len,
                                  uint8_t use_magnetometer)
{
    int16_t raw_i;
    int16_t raw_j;
    int16_t raw_k;
    int16_t raw_r;
    const float q14 = 1.0f / 16384.0f;

    if (len < 12u) {
        return;
    }

    raw_i = read_i16(payload + 4);
    raw_j = read_i16(payload + 6);
    raw_k = read_i16(payload + 8);
    raw_r = read_i16(payload + 10);

    if (use_magnetometer) {
        quat_to_euler((float)raw_i * q14, (float)raw_j * q14,
                      (float)raw_k * q14, (float)raw_r * q14,
                      &shujv.mag_yaw, &shujv.mag_pitch, &shujv.mag_roll);
        shujv.mag_accuracy = payload[2] & 0x03u;
        shujv.mag_euler_fresh = 1;
    } else {
        quat_to_euler((float)raw_i * q14, (float)raw_j * q14,
                      (float)raw_k * q14, (float)raw_r * q14,
                      &shujv.yaw, &shujv.pitch, &shujv.roll);
        shujv.accuracy = payload[2] & 0x03u;
        shujv.euler_fresh = 1;
    }
    new_data = 1;

    if (drv_state == SHTP_STATE_CONFIGURING) {
        drv_state = SHTP_STATE_RUNNING;
    }
}

static void quat_to_euler(float qi, float qj, float qk, float qr,
                          float *yaw, float *pitch, float *roll)
{
    float sinr_cosp = 2.0f * (qr * qi + qj * qk);
    float cosr_cosp = 1.0f - 2.0f * (qi * qi + qj * qj);
    float sinp = 2.0f * (qr * qj - qk * qi);
    float siny_cosp = 2.0f * (qr * qk + qi * qj);
    float cosy_cosp = 1.0f - 2.0f * (qj * qj + qk * qk);
    const float rad_to_deg = 180.0f / M_PI;

    *roll = atan2f(sinr_cosp, cosr_cosp) * rad_to_deg;
    if (sinp >= 1.0f) {
        *pitch = 90.0f;
    } else if (sinp <= -1.0f) {
        *pitch = -90.0f;
    } else {
        *pitch = asinf(sinp) * rad_to_deg;
    }
    *yaw = atan2f(siny_cosp, cosy_cosp) * rad_to_deg;
}

static void enable_reports(void)
{
    /* MCU复位不会让BNO085断电，显式关闭上次会话可能仍在运行的报告。 */
    set_feature(SENSOR_REPORTID_CAL_GYRO, 0u);
    set_feature(SENSOR_REPORTID_GAME_RV, 0u);
    set_feature(SENSOR_REPORTID_ROTATION_VECTOR, 0u);
    set_feature(SENSOR_REPORTID_LINEAR_ACCEL, 2500u);
}

static void set_feature(uint8_t report_id, uint32_t interval_us)
{
    uint8_t cmd[17];

    cmd[0] = SHTP_CMD_SET_FEATURE;
    cmd[1] = report_id;
    cmd[2] = 0x00;
    cmd[3] = 0x00;
    cmd[4] = 0x00;
    cmd[5] = (uint8_t)(interval_us & 0xFFu);
    cmd[6] = (uint8_t)((interval_us >> 8) & 0xFFu);
    cmd[7] = (uint8_t)((interval_us >> 16) & 0xFFu);
    cmd[8] = (uint8_t)((interval_us >> 24) & 0xFFu);
    cmd[9] = 0x00;
    cmd[10] = 0x00;
    cmd[11] = 0x00;
    cmd[12] = 0x00;
    cmd[13] = 0x00;
    cmd[14] = 0x00;
    cmd[15] = 0x00;
    cmd[16] = 0x00;

    send_shtp_command(cmd, sizeof(cmd), SHTP_CH_CONTROL);
}

static void send_shtp_command(const uint8_t *payload, uint16_t len, uint8_t channel)
{
    uint16_t pkt_len = (uint16_t)(len + 4u);
    uint8_t pkt[32];
    uint16_t i;

    if (pkt_len > sizeof(pkt) || channel >= sizeof(tx_seq)) {
        return;
    }

    pkt[0] = (uint8_t)(pkt_len & 0xFFu);
    pkt[1] = (uint8_t)((pkt_len >> 8) & 0x7Fu);
    pkt[2] = channel;
    pkt[3] = tx_seq[channel]++;
    memcpy(pkt + 4, payload, len);

    tx_busy = 1;
    uart_send_byte(UART_FLAG_BYTE);
    uart_send_byte(UART_PROTO_SHTP);
    for (i = 0; i < pkt_len; i++) {
        uart_send_escaped(pkt[i]);
    }
    uart_send_byte(UART_FLAG_BYTE);
    tx_busy = 0;
}

static void send_bsq(void)
{
    tx_busy = 1;
    uart_send_byte(UART_FLAG_BYTE);
    uart_send_byte(UART_PROTO_BSQ_BSN);
    uart_send_byte(UART_FLAG_BYTE);
    tx_busy = 0;
}

static void send_soft_reset(void)
{
    const uint8_t reset_cmd = 0x01u;

    send_shtp_command(&reset_cmd, 1u, SHTP_CH_EXECUTABLE);
}

static void uart_send_escaped(uint8_t b)
{
    if (b == UART_FLAG_BYTE || b == UART_ESCAPE_BYTE) {
        uart_send_byte(UART_ESCAPE_BYTE);
        uart_send_byte((uint8_t)(b ^ UART_ESCAPE_XOR));
    } else {
        uart_send_byte(b);
    }
}

static void uart_send_byte(uint8_t b)
{
    (void)HAL_UART_Transmit(&bno085_uart, &b, 1, 10);
    delay_us(120);
}

static void delay_us(uint32_t us)
{
    uint32_t cycles_per_us;
    uint32_t cycles;
    uint32_t start;

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    cycles_per_us = HAL_RCC_GetHCLKFreq() / 1000000u;
    cycles = cycles_per_us * us;
    start = DWT->CYCCNT;
    while ((uint32_t)(DWT->CYCCNT - start) < cycles) {
        __NOP();
    }
}
