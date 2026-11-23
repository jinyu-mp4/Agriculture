/**
  ******************************************************************************
  * @file    claw.c
  * @brief   串口舵机夹爪控制（USART6，只发不收）
  ******************************************************************************
  */

#include "claw.h"
#include <string.h>

/* 串口发送超时(ms) */
#define CLAW_UART_TIMEOUT_MS   100

/* 固定指令：ID=000，P 后为位置(0~1000)，T 后为运动时间(ms)，! 结束 */
static const char CLAW_OPEN_CMD[]  = "#000P660T0500!";
static const char CLAW_CLOSE_CMD[] = "#000P1000T0500!";

static void claw_send(const char *cmd)
{
    HAL_UART_Transmit(&huart6, (uint8_t *)cmd, (uint16_t)strlen(cmd), CLAW_UART_TIMEOUT_MS);
}

void Claw_Open(void)
{
    claw_send(CLAW_OPEN_CMD);
}

void Claw_Close(void)
{
    claw_send(CLAW_CLOSE_CMD);
}
