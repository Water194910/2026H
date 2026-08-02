#include "gray_sensor.h"
#include "chassis_pid.h"
#include <stdlib.h>

#define TRACK_STEERING_MAX_RPM  57.3f
#define TRACK_MAX_SPEED_RPM     382.0f
#define TRACK_FULL_SPEED_DEVIATION  15.0f
#define TRACK_MIN_SPEED_DEVIATION   30.0f
#define TRACK_MIN_SPEED_FACTOR       0.60f
#define TRACK_BALL_MIN_SPEED_FACTOR  0.95f

/* S1在最左、S8在最右。权值以阵列中心为0，避免偶数传感器没有中心点。 */
static const int8_t gray_weights[GRAY_SENSOR_CHANNELS] = {
    -35, -25, -15, -5, 5, 15, 25, 35
};

static float GraySensor_ClampSpeed(float speed)
{
    if (speed > TRACK_MAX_SPEED_RPM) return TRACK_MAX_SPEED_RPM;
    if (speed < -TRACK_MAX_SPEED_RPM) return -TRACK_MAX_SPEED_RPM;
    return speed;
}

// ============================================
// 初始化传感器
// ============================================

void GraySensor_Init(GraySensor_HandleTypeDef* gray,
                     GPIO_TypeDef* gpio_port1, uint16_t gpio_pin1,
                     GPIO_TypeDef* gpio_port2, uint16_t gpio_pin2,
                     GPIO_TypeDef* gpio_port3, uint16_t gpio_pin3,
                     GPIO_TypeDef* gpio_port4, uint16_t gpio_pin4,
                     GPIO_TypeDef* gpio_port5, uint16_t gpio_pin5,
                     GPIO_TypeDef* gpio_port6, uint16_t gpio_pin6,
                     GPIO_TypeDef* gpio_port7, uint16_t gpio_pin7,
                     GPIO_TypeDef* gpio_port8, uint16_t gpio_pin8)
{
    // 参数检查
    if (gray == NULL) return;
    
    // 设置GPIO端口和引脚
    gray->gpio_port[0] = gpio_port1; gray->gpio_pin[0] = gpio_pin1;
    gray->gpio_port[1] = gpio_port2; gray->gpio_pin[1] = gpio_pin2;
    gray->gpio_port[2] = gpio_port3; gray->gpio_pin[2] = gpio_pin3;
    gray->gpio_port[3] = gpio_port4; gray->gpio_pin[3] = gpio_pin4;
    gray->gpio_port[4] = gpio_port5; gray->gpio_pin[4] = gpio_pin5;
    gray->gpio_port[5] = gpio_port6; gray->gpio_pin[5] = gpio_pin6;
    gray->gpio_port[6] = gpio_port7; gray->gpio_pin[6] = gpio_pin7;
    gray->gpio_port[7] = gpio_port8; gray->gpio_pin[7] = gpio_pin8;
    
    // 初始化轨迹参数
    gray->line_position = 50;      // 默认中间位置
    gray->line_status = 0;         // 离线状态
    gray->all_white = 0;           // 非全白状态
    gray->sensor_bits = 0U;
    gray->active_count = 0U;
    gray->last_line_deviation = 0; // 最后检测到的线偏差值
}

// ============================================
// 更新传感器 - 读取所有通道的传感器值
// ============================================

void GraySensor_Update(GraySensor_HandleTypeDef* gray)
{
    if (gray == NULL) return;
    
    uint8_t black_count = 0U;
    uint8_t sensor_bits = 0U;
    int16_t weighted_sum = 0;
    
    /* 赛道整体向右，最左侧S1/S2会被场地左侧黑色字母误触发。
     * 仅循迹位置计算忽略S1/S2；比赛横线检测仍直接读取完整八路GPIO。 */
    for (uint8_t i = 2U; i < GRAY_SENSOR_CHANNELS; i++) {
        if (HAL_GPIO_ReadPin(gray->gpio_port[i], gray->gpio_pin[i]) == GRAY_BLACK_LEVEL) {
            black_count++;
            sensor_bits |= (uint8_t)(1U << i);
            weighted_sum += gray_weights[i];
        }
    }

    gray->sensor_bits = sensor_bits;
    gray->active_count = black_count;
    gray->all_white = (black_count == 0U) ? 1U : 0U;
    
    if (black_count > 0) {
        // 检测到线
        gray->line_status = 1;
        
        int16_t deviation = weighted_sum / (int16_t)black_count;

        /* 加权结果为-35..35，映射到原接口的0..100。 */
        gray->line_position = (uint8_t)(50 + (deviation * 10) / 7);
        
        // 更新最后检测到的线偏差值
        gray->last_line_deviation = (int8_t)gray->line_position - 50;
    } else {
        // 未检测到线
        gray->line_status = 0;
        gray->line_position = 50;
        // 注意：不更新last_line_deviation，保持最后检测到的值
    }
    
}


// ============================================
// 轨迹检测函数
// ============================================

// 读取轨迹偏差（-50到+50，0表示在轨迹中心）
int8_t GraySensor_GetLineDeviation(GraySensor_HandleTypeDef* gray)
{
    if (gray == NULL) return 0;
    if (gray->line_status == 0) return gray->last_line_deviation;
    
    int8_t deviation = (int8_t)gray->line_position - 50;
    
    // 限制范围
    if (deviation > 50) deviation = 50;
    if (deviation < -50) deviation = -50;
    
    return deviation;
}

// 循迹速度控制
void XinZhiGuoDu(GraySensor_HandleTypeDef* gray, tPid* pidErr,
                 tPid* pidMotor1, tPid* pidMotor2, float baseSpeed,
                 uint8_t ball_profile)
{
    int8_t lineDeviation = GraySensor_GetLineDeviation(gray);
    float absDeviation = (float)abs(lineDeviation);
    float steeringAdjust = PID_realize(pidErr, (float)lineDeviation);
    float speedFactor = 1.0f;
    float minSpeedFactor = ball_profile
                         ? TRACK_BALL_MIN_SPEED_FACTOR
                         : TRACK_MIN_SPEED_FACTOR;

    /* H5/H6只轻降到95%，H2/H4仍降到60%；两者都在15~30之间线性变化。 */
    if (absDeviation >= TRACK_MIN_SPEED_DEVIATION)
    {
        speedFactor = minSpeedFactor;
    }
    else if (absDeviation > TRACK_FULL_SPEED_DEVIATION)
    {
        float progress = (absDeviation - TRACK_FULL_SPEED_DEVIATION) /
                         (TRACK_MIN_SPEED_DEVIATION - TRACK_FULL_SPEED_DEVIATION);
        speedFactor = 1.0f - (1.0f - minSpeedFactor) * progress;
    }

    {
        float actualBaseSpeed = baseSpeed * speedFactor;

        if (steeringAdjust > TRACK_STEERING_MAX_RPM)
        {
            steeringAdjust = TRACK_STEERING_MAX_RPM;
        }
        if (steeringAdjust < -TRACK_STEERING_MAX_RPM)
        {
            steeringAdjust = -TRACK_STEERING_MAX_RPM;
        }

        pidMotor1->target_val = actualBaseSpeed - steeringAdjust;
        pidMotor2->target_val = actualBaseSpeed + steeringAdjust;
    }

    pidMotor1->target_val = GraySensor_ClampSpeed(pidMotor1->target_val);
    pidMotor2->target_val = GraySensor_ClampSpeed(pidMotor2->target_val);
}

// 循迹状态机控制
void XinZhiStateMachine(GraySensor_HandleTypeDef* gray, tPid* pidErr,
                        tPid* pidMotor1, tPid* pidMotor2, float baseSpeed,
                        uint32_t* lost_time, uint8_t* state,
                        uint8_t ball_profile)
{
    if (gray == NULL || pidErr == NULL || pidMotor1 == NULL ||
        pidMotor2 == NULL || lost_time == NULL || state == NULL)
    {
        return;
    }

    /* 不再进入丢线检查、原地寻找或超时停车。全白时
     * GraySensor_GetLineDeviation() 会沿用最后一次有效偏差。 */
    if (*state != TRACK_STATE_NORMAL)
    {
        Chassis_PID_Reset(pidErr);
    }
    *state = TRACK_STATE_NORMAL;
    *lost_time = 0U;
    XinZhiGuoDu(gray, pidErr, pidMotor1, pidMotor2, baseSpeed, ball_profile);
}
