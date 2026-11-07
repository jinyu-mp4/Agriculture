/**
  ******************************************************************************
  * @file    pid.h
  * @brief   通用增量式/位置式 PID 控制器
  *
  *          位置式：out = Kp*e + Ki*∫e dt + Kd*de/dt，带输出限幅与积分限幅。
  ******************************************************************************
  */
#ifndef __PID_H
#define __PID_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float kp;
    float ki;
    float kd;
    float integral;     /* 积分累计 */
    float prev_error;   /* 上一次误差(微分用) */
    float out_min;      /* 输出下限 */
    float out_max;      /* 输出上限 */
    float int_min;      /* 积分限幅(抗饱和) */
    float int_max;
} PID_t;

/** 初始化 PID。out_min/out_max 为输出限幅；积分默认不限幅。 */
void PID_Init(PID_t *p, float kp, float ki, float kd, float out_min, float out_max);

/** 复位积分与微分记忆 */
void PID_Reset(PID_t *p);

/** 设置积分限幅(抗饱和) */
void PID_SetIntLimit(PID_t *p, float int_min, float int_max);

/** 计算 PID：error = setpoint - measurement */
float PID_Compute(PID_t *p, float setpoint, float measurement, float dt);

/** 直接给定误差计算 PID（角度环等需先对误差做最短角归一化时使用） */
float PID_Compute_Error(PID_t *p, float error, float dt);

#ifdef __cplusplus
}
#endif

#endif /* __PID_H */
