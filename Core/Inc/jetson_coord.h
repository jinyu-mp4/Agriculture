/**
  ******************************************************************************
  * @file    jetson_coord.h
  * @brief   上位机 Jetson 物块二维坐标接收（UART4）
  *
  *          上位机通过 UART4 以 ASCII 发送物块二维坐标，格式固定为：
  *            $x,y
  *          例：$795,225
  *          x、y 为有符号十进制整数；支持负数（前导 '-'），坐标允许正负。
  *
  *          接收方式：UART4 单字节中断接收，在 HAL_UART_RxCpltCallback 里
  *          逐字节拼帧，不占用任务时间片。
  *
  *          接线（STM32F407）：PA0 = UART4_TX，PA1 = UART4_RX，波特率 115200 8N1。
  ******************************************************************************
  */
#ifndef __JETSON_COORD_H
#define __JETSON_COORD_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usart.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdint.h>

/* ============================================================================
 *                            类型定义
 * ========================================================================== */

/** 一个二维坐标点 */
typedef struct
{
    int32_t x;
    int32_t y;
    uint32_t frame;   /* 两帧坐标相同也有各自的帧序号 */
} CoordPoint;

/* ============================================================================
 *                            接口函数
 * ========================================================================== */

/** 初始化：为 UART4 启动单字节中断接收。 */
void Coord_Init(void);

/** 由 HAL_UART_RxCpltCallback 转发调用（每收到 1 字节调用一次，内部自动重新武装） */
void Coord_FeedByte(UART_HandleTypeDef *huart);

/** 由 HAL_UART_ErrorCallback 转发调用（串口出错后恢复接收） */
void Coord_UART_Error(UART_HandleTypeDef *huart);

/** 取最新坐标。有有效坐标时返回 true 并写入 *pt，否则返回 false 且不改动 *pt。 */
bool Coord_GetPoint(CoordPoint *pt);

/** 清零最新坐标并标记为无效。调用时须屏蔽 UART4 接收中断。 */
void Coord_ClearPoint(void);

/** 当前是否有有效坐标 */
bool Coord_IsValid(void);

/** 取最新坐标的 x（尚未收到有效帧时返回 0） */
int32_t Coord_GetX(void);

/** 取最新坐标的 y（尚未收到有效帧时返回 0） */
int32_t Coord_GetY(void);

/** 已接收字节数（诊断用） */
uint32_t Coord_GetRxCount(void);

/** 已拼出的完整帧数（诊断用） */
uint32_t Coord_GetFrameCount(void);

#ifdef __cplusplus
}
#endif

#endif /* __JETSON_COORD_H */
