/**
  ******************************************************************************
  * @file    claw.h
  * @brief   串口舵机夹爪控制（USART6）
  *
  *          通过 USART6 发送固定指令控制夹爪开合（串口总线舵机协议）。
  *            打开：#000P0660T0500!（P 为四位字段，T 单位 ms）
  *            闭合：#000P1000T0500!
  *
  *          发送接口返回 USART6 发送结果；另提供单字节中断接收，用于抓取夹爪回传的反馈帧，
  *          便于在屏幕上观察夹爪是否有应答、应答内容是否正常（故障检测）。
  ******************************************************************************
  */
#ifndef __CLAW_H
#define __CLAW_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usart.h"
#include <stdbool.h>

/* ============================================================================
 *                            接口函数
 * ========================================================================== */

/** 发送打开指令，返回串口是否成功发出。 */
bool Claw_Open(void);

/** 发送闭合指令，返回串口是否成功发出。 */
bool Claw_Close(void);

/* 放蛋后查询舵机位置，用于区分已打开、位置偏差和未收到有效回读。 */
typedef enum
{
    CLAW_OPEN_NO_REPLY,
    CLAW_OPEN_REACHED,
    CLAW_OPEN_MISMATCH
} ClawOpenStatus;

/** 通过 USART6 查询开爪位置，等待有效反馈最多 100 ms。 */
ClawOpenStatus Claw_CheckOpen(void);

/** 初始化：为 USART6 启动单字节中断接收（用于抓夹爪回传，诊断故障）。 */
void Claw_Init(void);

/** 由 HAL_UART_RxCpltCallback 转发调用（每收到 1 字节调用一次，内部自动重新武装） */
void Claw_FeedByte(UART_HandleTypeDef *huart);

/** 由 HAL_UART_ErrorCallback 转发调用（串口出错后恢复接收） */
void Claw_UART_Error(UART_HandleTypeDef *huart);

/** 已接收字节数（诊断用） */
uint32_t Claw_GetRxCount(void);

/** 已拼出的完整反馈帧数（诊断用） */
uint32_t Claw_GetFrameCount(void);

/** 最近一帧夹爪反馈文本（ASCII，以 '!' 或换行结尾）。尚未收到时返回空串 ""。 */
const char *Claw_GetLastReply(void);

#ifdef __cplusplus
}
#endif

#endif /* __CLAW_H */
