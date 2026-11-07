/**
  ******************************************************************************
  * @file    laser.h
  * @brief   单线红外激光测距模块驱动（MINIF / DMP0501C1，串口输出）
  *
  *          串口协议（参考 others/激光测距模块，波特率 115200，8N1）：
  *            模块主动按 ASCII 输出一行：
  *              <0x20 空格> <距离 mm 十进制字符串> <0x2C 逗号>
  *              <0x20 空格> <置信度 0~100> <0x0A 换行>
  *            例：" 1234, 45\n" 表示距离 1234mm、置信度 45。
  *            有效距离 20~4000mm；置信度 0~100。
  *
  *          接线：
  *            USART2  PA3=RX  -> 右侧激光
  *            USART3  PB11=RX -> 前侧激光
  *          仅接收，不发送。
  ******************************************************************************
  */
#ifndef __LASER_H
#define __LASER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usart.h"
#include <stdbool.h>

/* ============================================================================
 *                            类型定义
 * ========================================================================== */

/** 激光测距模块编号 */
typedef enum
{
    LASER_RIGHT = 0,   /* 右侧 -> USART2 */
    LASER_FRONT,       /* 前侧 -> USART3 */
    LASER_NUM
} LaserID;

/* ============================================================================
 *                            接口函数
 * ========================================================================== */

/** 初始化：为两个激光模块启动单字节中断接收。 */
void Laser_Init(void);

/** 由 HAL_UART_RxCpltCallback 转发调用（每收到 1 字节调用一次，内部自动重新武装） */
void Laser_FeedByte(UART_HandleTypeDef *huart);

/** 由 HAL_UART_ErrorCallback 转发调用（串口出错后恢复接收） */
void Laser_UART_Error(UART_HandleTypeDef *huart);

/** 读取最新距离(mm)；尚未收到有效数据时返回 0 */
uint16_t Laser_GetDistance(LaserID id);

/** 读取最新置信度(0~62) */
uint8_t Laser_GetConfidence(LaserID id);

/** 是否已收到过至少一帧有效数据 */
bool Laser_IsValid(LaserID id);

/** 已接收字节数（诊断用） */
uint32_t Laser_GetRxCount(LaserID id);

/** 已拼出的完整帧数（诊断用，含未通过距离/置信度校验的帧） */
uint32_t Laser_GetFrameCount(LaserID id);

#ifdef __cplusplus
}
#endif

#endif /* __LASER_H */
