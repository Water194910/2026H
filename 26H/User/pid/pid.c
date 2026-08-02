#include "pid.h"

void PID_Init(PID_Handle_t pid, float kp, float ki, float kd)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;

    pid->target_val = 0.0f;
    pid->actual_val = 0.0f;
    pid->err        = 0.0f;
    pid->err_last   = 0.0f;
    pid->err_sum    = 0.0f;
    pid->output     = 0.0f;
}

/* 清历史状态。停机、切目标、或者刚上电时调一下，
 * 否则残留的积分会让电机一启动就猛冲 */
void PID_Reset(PID_Handle_t pid)
{
    pid->err      = 0.0f;
    pid->err_last = 0.0f;
    pid->err_sum  = 0.0f;
    pid->output   = 0.0f;
}

/* 速度环：输入实测转速和目标转速（都是 RPM），返回电压给定 ±25000 */
float PID_SpeedLoop(PID_Handle_t pid, float actual_val, float target_val)
{
    pid->target_val = target_val;
    pid->actual_val = actual_val;
    pid->err        = target_val - actual_val;

    /* 积分累加后立刻限幅，防止长时间大误差把积分撑爆
     * （比如电机被卡住，误差一直存在，积分会无限涨） */
    pid->err_sum += pid->err;
    pid->err_sum  = PID_LIMIT_MIN_MAX(pid->err_sum, PID_ERR_SUM_MIN, PID_ERR_SUM_MAX);

    float out = pid->Kp * pid->err
              + pid->Ki * pid->err_sum
              + pid->Kd * (pid->err - pid->err_last);

    pid->err_last = pid->err;
    pid->output   = PID_LIMIT_MIN_MAX(out, PID_VOLTAGE_MIN, PID_VOLTAGE_MAX);

    return pid->output;
}
