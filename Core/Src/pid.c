/**
  ******************************************************************************
  * @file    pid.c
  * @brief   通用位置式 PID 控制器实现
  ******************************************************************************
  */

#include "pid.h"

void PID_Init(PID_t *p, float kp, float ki, float kd, float out_min, float out_max)
{
    p->kp = kp;
    p->ki = ki;
    p->kd = kd;
    p->integral = 0.0f;
    p->prev_error = 0.0f;
    p->out_min = out_min;
    p->out_max = out_max;
    p->int_min = -1.0e6f;
    p->int_max =  1.0e6f;
}

void PID_Reset(PID_t *p)
{
    p->integral = 0.0f;
    p->prev_error = 0.0f;
}

void PID_SetIntLimit(PID_t *p, float int_min, float int_max)
{
    p->int_min = int_min;
    p->int_max = int_max;
}

float PID_Compute_Error(PID_t *p, float error, float dt)
{
    float p_term, i_term, d_term, out;

    if (dt <= 0.0f)
    {
        dt = 0.001f;
    }

    /* 积分(带限幅抗饱和) */
    p->integral += error * dt;
    if (p->integral > p->int_max) p->integral = p->int_max;
    if (p->integral < p->int_min) p->integral = p->int_min;

    /* 微分(对误差微分) */
    d_term = (error - p->prev_error) / dt;
    p->prev_error = error;

    p_term = p->kp * error;
    i_term = p->ki * p->integral;
    d_term = p->kd * d_term;

    out = p_term + i_term + d_term;
    if (out > p->out_max) out = p->out_max;
    if (out < p->out_min) out = p->out_min;

    return out;
}

float PID_Compute(PID_t *p, float setpoint, float measurement, float dt)
{
    return PID_Compute_Error(p, setpoint - measurement, dt);
}
