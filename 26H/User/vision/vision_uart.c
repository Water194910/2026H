#include "vision_uart.h"
#include "bno085_shtp.h"
#include "debug_uart.h"
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

/* 与 usart.c 里 Cube 生成的句柄链接 */
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef  hdma_usart2_rx;

Vision_Data_t vision = {0};

/* DMA 写入缓冲。NORMAL 模式，每帧收完后要重新启动 */
static uint8_t vision_dma_buf[VISION_RX_BUF_SIZE];

/* 中断里只做轻量解析：atof偏重，这里手写一个够用的小数解析。 */
static int vision_parse_number(const char *s, uint16_t n,
                               uint16_t *index, float *out)
{
    uint16_t i = *index;
    int sign = 1;
    float val = 0.0f;
    float frac = 0.1f;
    uint8_t saw_digit = 0U;
    uint8_t in_frac = 0U;

    while (i < n && (s[i] == ' ' || s[i] == '\t'))
    {
        i++;
    }
    if (i >= n)
    {
        return 0;
    }

    if (s[i] == '+')
    {
        i++;
    }
    else if (s[i] == '-')
    {
        sign = -1;
        i++;
    }

    for (; i < n; i++)
    {
        char c = s[i];
        if (c == ',' || c == '\r' || c == '\n')
        {
            break;
        }
        if (c >= '0' && c <= '9')
        {
            saw_digit = 1U;
            if (!in_frac)
            {
                val = val * 10.0f + (float)(c - '0');
            }
            else
            {
                val += (float)(c - '0') * frac;
                frac *= 0.1f;
            }
        }
        else if (c == '.' && !in_frac)
        {
            in_frac = 1U;
        }
        else
        {
            return 0;
        }
    }

    if (!saw_digit)
    {
        return 0;
    }

    *out = val * (float)sign;
    *index = i;
    return 1;
}

static int vision_parse_line(const char *s, uint16_t n,
                             float *position_mm, float *target_mm,
                             uint8_t *has_target)
{
    uint16_t i = 0U;

    while (i < n && (s[i] == ' ' || s[i] == '\t'))
    {
        i++;
    }

    *has_target = 0U;
    if (i + 1U < n && s[i] == 'P' && s[i + 1U] == ',')
    {
        i += 2U;
        if (!vision_parse_number(s, n, &i, position_mm) ||
            i >= n || s[i] != ',')
        {
            return 0;
        }
        i++;
        if (!vision_parse_number(s, n, &i, target_mm))
        {
            return 0;
        }
        if (*target_mm < -110.0f || *target_mm > 110.0f)
        {
            return 0;
        }
        *has_target = 1U;
        return 1;
    }

    /* 兼容旧版：单个数字表示相对水管中心的球位置。 */
    return vision_parse_number(s, n, &i, position_mm);
}

static void vision_restart_dma(void)
{
    /* 关掉半满中断：行很短，半满只会多打一次没用的回调 */
    if (huart2.hdmarx != NULL)
    {
        __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
    }

    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart2, vision_dma_buf, VISION_RX_BUF_SIZE) != HAL_OK)
    {
        /* 状态异常时先 Abort 再试一次 */
        HAL_UART_AbortReceive(&huart2);
        (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart2, vision_dma_buf, VISION_RX_BUF_SIZE);
        if (huart2.hdmarx != NULL)
        {
            __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
        }
    }
}

void Vision_Init(void)
{
    memset((void *)&vision, 0, sizeof(vision));
    vision_restart_dma();
}

void Vision_IdleCallback(uint16_t size)
{
    float position_mm = 0.0f;
    float target_mm = 0.0f;
    uint8_t has_target = 0U;

    if (size == 0U || size > VISION_RX_BUF_SIZE)
    {
        vision.err_count++;
        vision_restart_dma();
        return;
    }

    /* 在 DMA 缓冲上就地解析；解析完立刻重启 DMA，缩短窗口 */
    if (vision_parse_line((const char *)vision_dma_buf, size,
                          &position_mm, &target_mm, &has_target))
    {
        vision.distance_mm = position_mm;
        vision.last_raw = position_mm;
        if (has_target)
        {
            vision.target_mm = target_mm;
            vision.target_valid = 1U;
            vision.target_rx_count++;
        }
        vision.valid = 1U;
        vision.connected = 1U;
        vision.last_rx_ms = HAL_GetTick();
        vision.rx_count++;
    }
    else
    {
        vision.err_count++;
    }

    vision_restart_dma();
}

void Vision_Poll(void)
{
    if (vision.connected &&
        (HAL_GetTick() - vision.last_rx_ms) > VISION_TIMEOUT_MS)
    {
        vision.connected = 0U;
        vision.valid = 0U;
    }
}

uint8_t Vision_GetSnapshot(Vision_Snapshot_t *snapshot)
{
    uint32_t primask;

    if (snapshot == NULL)
    {
        return 0U;
    }

    /* distance/count/time 必须来自同一次发布。临界区只有几个32位读取。 */
    primask = __get_PRIMASK();
    __disable_irq();
    snapshot->distance_mm = vision.distance_mm;
    snapshot->valid       = vision.valid;
    snapshot->connected   = vision.connected;
    snapshot->rx_count    = vision.rx_count;
    snapshot->last_rx_ms  = vision.last_rx_ms;
    if (primask == 0U)
    {
        __enable_irq();
    }

    return 1U;
}

/* HAL 回调：IDLE 或收满缓冲都会进这里
 * Size = 本次实际收到的字节数（到 IDLE 为止） */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART2)
    {
        Vision_IdleCallback(Size);
    }
}

/* DMA/UART 出错时把接收拉起来，避免视觉整路死掉 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        vision.err_count++;
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        HAL_UART_AbortReceive(huart);
        vision_restart_dma();
    }
    else if (huart->Instance == USART3)
    {
        BNO085_SHTP_RestartRx();
    }
    else if (huart->Instance == USART1)
    {
        /* 调参串口发生 ORE/FE/NE 后 HAL 可能停止单字节接收。
         * 对齐 ytbeif 的做法，丢弃残缺行并重新挂起接收。 */
        dbg_rx_idx = 0U;
        (void)HAL_UART_Receive_IT(&huart1, &dbg_rx_byte, 1U);
    }
}
