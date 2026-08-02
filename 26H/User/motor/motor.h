#ifndef MOTOR_H_
#define MOTOR_H_

// 包含必要的HAL库头文件
#include "tim.h"

#define PWMA(pwm)    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, pwm)
#define PWMB(pwm)    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, pwm)

// GPIO宏定义
#define AIN1_set     HAL_GPIO_WritePin(AN1_GPIO_Port, AN1_Pin, GPIO_PIN_SET)
#define AIN1_reset   HAL_GPIO_WritePin(AN1_GPIO_Port, AN1_Pin, GPIO_PIN_RESET)
#define AIN2_set     HAL_GPIO_WritePin(AN2_GPIO_Port, AN2_Pin, GPIO_PIN_SET)
#define AIN2_reset   HAL_GPIO_WritePin(AN2_GPIO_Port, AN2_Pin, GPIO_PIN_RESET)
#define BIN1_set     HAL_GPIO_WritePin(BN1_GPIO_Port, BN1_Pin, GPIO_PIN_SET)
#define BIN1_reset   HAL_GPIO_WritePin(BN1_GPIO_Port, BN1_Pin, GPIO_PIN_RESET)
#define BIN2_set     HAL_GPIO_WritePin(BN2_GPIO_Port, BN2_Pin, GPIO_PIN_SET)
#define BIN2_reset   HAL_GPIO_WritePin(BN2_GPIO_Port, BN2_Pin, GPIO_PIN_RESET)

void Set_pwm(int motor1,int motor2);
#endif 
