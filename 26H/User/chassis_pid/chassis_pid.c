#include "chassis_pid.h"

#define LEFT_SPEED_PID_KP       0.07069f
#define LEFT_SPEED_PID_KI       0.005236f
#define LEFT_SPEED_PID_KD       0.0f
#define LEFT_SPEED_PID_INT_MAX  11460.0f

#define RIGHT_SPEED_PID_KP       0.07069f
#define RIGHT_SPEED_PID_KI       0.005236f
#define RIGHT_SPEED_PID_KD       0.0f
#define RIGHT_SPEED_PID_INT_MAX  11460.0f

tPid pidMotor1Speed, pidMotor2Speed, piderr, pidmpu;

static void Chassis_PID_Config(tPid *pid, float target, float kp, float ki, float kd,
                               float int_max, float out_max)
{
    pid->target_val = target;
    pid->actual_val = 0.0f;
    pid->err = 0.0f;
    pid->err_last = 0.0f;
    pid->err_sum = 0.0f;
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->int_max = int_max;
    pid->out_max = out_max;
}

void Chassis_PID_init(void)
{
    Chassis_PID_Config(&pidMotor1Speed, 230.0f,
                       LEFT_SPEED_PID_KP, LEFT_SPEED_PID_KI, LEFT_SPEED_PID_KD,
                       LEFT_SPEED_PID_INT_MAX, 99.0f);
    Chassis_PID_Config(&pidMotor2Speed, 230.0f,
                       RIGHT_SPEED_PID_KP, RIGHT_SPEED_PID_KI, RIGHT_SPEED_PID_KD,
                       RIGHT_SPEED_PID_INT_MAX, 99.0f);
    Chassis_PID_Config(&piderr, 0.0f, 26.737f, 0.0f, 22.918f, 2.0f, 57.3f);
    Chassis_PID_Config(&pidmpu, 0.0f, 1.0f, 0.0f, 0.4f, 1.0f, 99.0f);
}

void Chassis_PID_Reset(tPid *pid)
{
    if (pid == 0)
    {
        return;
    }

    pid->actual_val = 0.0f;
    pid->err = 0.0f;
    pid->err_last = 0.0f;
    pid->err_sum = 0.0f;
}

float PID_realize(tPid *pid, float actual_val)
{
    float output;

    if (pid == 0)
    {
        return 0.0f;
    }

    pid->actual_val = actual_val;
    pid->err = pid->target_val - pid->actual_val;
    pid->err_sum += pid->err;
    pid->err_sum = CHASSIS_PID_LIMIT(pid->err_sum, -pid->int_max, pid->int_max);

    output = pid->Kp * pid->err
           + pid->Ki * pid->err_sum
           + pid->Kd * (pid->err - pid->err_last);
    pid->err_last = pid->err;

    return CHASSIS_PID_LIMIT(output, -pid->out_max, pid->out_max);
}

float PID1_realize(tPid *pid, float actual_val)
{
    return CHASSIS_PID_LIMIT(PID_realize(pid, actual_val), -6.0f, 6.0f);
}

float PID2_realize(tPid *pid, float actual_val)
{
    return CHASSIS_PID_LIMIT(PID_realize(pid, actual_val), -4.0f, 4.0f);
}
