#ifndef __CHASSIS_PID_H
#define __CHASSIS_PID_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* 底盘/循迹用 PID，与 User/pid（GM6020）隔离，避免 PID_Reset 等符号撞车 */

#define CHASSIS_PID_LIMIT(x, min, max) \
    ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))

typedef struct
{
    float target_val;
    float actual_val;
    float err;
    float err_last;
    float err_sum;
    float Kp, Ki, Kd;
    float int_max;
    float out_max;
} tPid;

extern tPid pidMotor1Speed;
extern tPid pidMotor2Speed;
extern tPid piderr;
extern tPid pidmpu;

void Chassis_PID_init(void);
float PID_realize(tPid *pid, float actual_val);
float PID1_realize(tPid *pid, float actual_val);
float PID2_realize(tPid *pid, float actual_val);
void Chassis_PID_Reset(tPid *pid);

#ifdef __cplusplus
}
#endif

#endif /* __CHASSIS_PID_H */
