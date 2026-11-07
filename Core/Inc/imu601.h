/**
  ******************************************************************************
  * @file    imu601.h
  * @brief   汇电籽-601 姿态模块驱动（M0 控制 ICM42688，串口输出姿态）
  *
  *          串口协议（参考 others/car_06_IMU601/imu601.c）：
  *            帧 = 0xAA 0x55 + 9 字节负载 + 1 字节校验，共 12 字节
  *            负载[2..10]，校验 = 负载各字节之和的低 8 位（存于第 12 字节）
  *            姿态位于负载[5..10]：yaw / pitch / roll 各 2 字节(小端)
  *            yaw 为无符号 0~36000(0.01°)，pitch/roll 为有符号(0.01°)
  *
  *          接线：USART1  PA9=TX  PA10=RX  波特率 115200
  ******************************************************************************
  */
#ifndef __IMU601_H
#define __IMU601_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

/** 姿态数据结构 */
typedef struct
{
    float yaw;    /* 航向角，已归一化到 -180 ~ +180 度 */
    float pitch;  /* 俯仰角(度) */
    float roll;   /* 横滚角(度) */
} IMU601_Attitude;

/** IMU 在线标志：收到第一帧校验通过的姿态后置 true（调试用，可断点查看） */
extern volatile bool g_imu_online;

/** 上电初始化：发送复位+校准命令，并启动单字节中断接收。
 *  需在任务上下文中调用（内部用到 osDelay）。 */
void IMU601_Init(void);

/** 由 HAL_UART_RxCpltCallback 转发调用（每收到 1 字节调用一次，内部自动重新武装） */
void IMU601_UART_RxCpltCallback(void);

/** 由 HAL_UART_ErrorCallback 转发调用（串口出错后恢复接收） */
void IMU601_UART_ErrorCallback(void);

/** 读取最新 yaw（-180 ~ +180 度）；尚未收到有效数据时返回 0 */
float IMU601_GetYaw(void);

/** 读取最新完整姿态；返回 true 表示已收到过有效数据 */
bool IMU601_GetAttitude(IMU601_Attitude *out);

#ifdef __cplusplus
}
#endif

#endif /* __IMU601_H */
