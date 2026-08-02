#ifndef GRAY_SENSOR_H
#define GRAY_SENSOR_H

#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdint.h>

// ============================================
// 传感器选项（根据实际硬件选择）
// ============================================

// 传感器通道数量：8通道灰度传感器
#define GRAY_SENSOR_CHANNELS 8

// GPIO电平定义（参考硬件设计）
// 逻辑高电平=检测到白色，逻辑低电平=检测到黑色
#define GRAY_DETECT_WHITE  1  // GPIO高电平
#define GRAY_DETECT_BLACK  0  // GPIO低电平
#define GRAY_BLACK_LEVEL   GPIO_PIN_RESET

#define TRACK_STATE_NORMAL      0U
#define TRACK_STATE_CHECKING    1U
#define TRACK_STATE_LEFT_TURN   2U
#define TRACK_STATE_RIGHT_TURN  3U
#define TRACK_STATE_STOP        4U

// GPIO工作模式设置（参考硬件设计）
// 1. 传感器供电为5V时：需要外部上拉，设置GPIO为输入模式
// 2. 传感器供电为3.3V时：无需外部上拉，设置GPIO为上拉输入模式
// 注意：这些设置需要在CubeMX中初始化配置

// ============================================
// 传感器状态结构体
// ============================================

typedef struct {
    // GPIO配置
    GPIO_TypeDef *gpio_port[GRAY_SENSOR_CHANNELS];  // GPIO端口
    uint16_t gpio_pin[GRAY_SENSOR_CHANNELS];        // GPIO引脚
    
    // 轨迹检测参数
    uint8_t line_position;       // 线位置，0-100，50表示正中间
    uint8_t line_status;         // 线状态，0=离线，1=检测到线
    uint8_t all_white;           // 全白状态，1=所有传感器都检测到白色，0=否则
    uint8_t sensor_bits;         // 黑线位图：bit0=S1(最左)，bit7=S8(最右)
    uint8_t active_count;        // 当前检测到黑线的传感器数量
    int8_t last_line_deviation;  // 最后检测到的线偏差值，用于判断转弯方向
} GraySensor_HandleTypeDef;

// ============================================
// 函数声明
// ============================================

// 初始化传感器
void GraySensor_Init(GraySensor_HandleTypeDef* gray, 
                     GPIO_TypeDef* gpio_port1, uint16_t gpio_pin1,
                     GPIO_TypeDef* gpio_port2, uint16_t gpio_pin2,
                     GPIO_TypeDef* gpio_port3, uint16_t gpio_pin3,
                     GPIO_TypeDef* gpio_port4, uint16_t gpio_pin4,
                     GPIO_TypeDef* gpio_port5, uint16_t gpio_pin5,
                     GPIO_TypeDef* gpio_port6, uint16_t gpio_pin6,
                     GPIO_TypeDef* gpio_port7, uint16_t gpio_pin7,
                     GPIO_TypeDef* gpio_port8, uint16_t gpio_pin8);

// 更新传感器 - 读取所有通道的传感器值
void GraySensor_Update(GraySensor_HandleTypeDef* gray);

// 轨迹检测函数
int8_t GraySensor_GetLineDeviation(GraySensor_HandleTypeDef* gray); // -50~+50

// 循迹速度控制（底盘 PID，勿 include GM6020 的 pid.h）
#include "chassis_pid.h"
void XinZhiGuoDu(GraySensor_HandleTypeDef* gray, tPid* pidErr,
                 tPid* pidMotor1, tPid* pidMotor2, float baseSpeed,
                 uint8_t ball_profile);

// 循迹状态机控制
void XinZhiStateMachine(GraySensor_HandleTypeDef* gray, tPid* pidErr,
                        tPid* pidMotor1, tPid* pidMotor2, float baseSpeed,
                        uint32_t* lost_time, uint8_t* state,
                        uint8_t ball_profile);

#endif // GRAY_SENSOR_H
