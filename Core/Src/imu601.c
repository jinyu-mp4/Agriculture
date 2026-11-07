/**
  ******************************************************************************
  * @file    imu601.c
  * @brief   汇电籽-601 姿态模块驱动（M0 控制 ICM42688，串口输出姿态）
  *
  *          串口协议（参考 others/car_06_IMU601/imu601.c）：
  *            帧 = 0xAA 0x55 + 9 字节负载 + 1 字节校验，共 12 字节
  *            姿态位于负载[5..10]：yaw / pitch / roll 各 2 字节(小端)
  *
  *          接收方式：USART1 单字节中断接收(HAL_UART_Receive_IT)，
  *          在 HAL_UART_RxCpltCallback 里逐字节拼帧，不占用任务时间片。
  ******************************************************************************
  */

#include "imu601.h"
#include "main.h"
#include "usart.h"
#include "cmsis_os.h"

#define IMU601_FRAME_LEN   12

/* 单字节 IT 接收缓冲 */
static uint8_t s_rx_byte = 0;

/* 帧拼装状态 */
static uint8_t s_frame[IMU601_FRAME_LEN];
static uint8_t s_index = 0;
static uint8_t s_last  = 0;

/* 最新姿态 */
static IMU601_Attitude s_att = { 0.0f, 0.0f, 0.0f };
volatile bool g_imu_online = false;

/** 校验并解析一帧（s_frame 已装满 12 字节） */
static void imu601_parse_frame(void)
{
    uint8_t sum = 0;

    for (int i = 2; i < 11; i++)
    {
        sum += s_frame[i];
    }
    if (sum != s_frame[11])
    {
        return;   /* 校验错，整帧丢弃 */
    }

    uint16_t yaw_raw   = ((uint16_t)s_frame[6] << 8) | s_frame[5];
    int16_t  pitch_raw = (int16_t)(((uint16_t)s_frame[8] << 8) | s_frame[7]);
    int16_t  roll_raw  = (int16_t)(((uint16_t)s_frame[10] << 8) | s_frame[9]);

    float yaw = (float)yaw_raw / 100.0f;   /* 原始 0 ~ 360 度 */
    if (yaw > 180.0f)
    {
        yaw -= 360.0f;                     /* 归一化到 -180 ~ +180 度 */
    }

    s_att.yaw   = yaw;
    s_att.pitch = (float)pitch_raw / 100.0f;
    s_att.roll  = (float)roll_raw  / 100.0f;
    g_imu_online = true;
}

/** 每收到 1 字节调用一次：喂入拼帧状态机，并重新武装下一字节接收 */
void IMU601_UART_RxCpltCallback(void)
{
    uint8_t b = s_rx_byte;

    s_frame[s_index] = b;
    if (b == 0x55 && s_last == 0xAA)
    {
        /* 检测到帧头 0xAA 0x55，重新对齐 */
        s_frame[0] = 0xAA;
        s_frame[1] = 0x55;
        s_index = 2;
    }
    else
    {
        s_index++;
    }
    s_last = s_frame[s_index - 1];

    if (s_index >= IMU601_FRAME_LEN)
    {
        s_index = 0;
        imu601_parse_frame();
    }

    HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1);
}

/** 串口出错（溢出/帧错等）：恢复单字节接收，避免断流 */
void IMU601_UART_ErrorCallback(void)
{
    HAL_UART_AbortReceive_IT(&huart1);
    HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1);
}

/** 上电初始化：复位 + 校准，然后启动接收 */
void IMU601_Init(void)
{
    static const uint8_t reset_cmd[] = { 0xAA, 0x55, 0x60, 0x12, 0x00, 0x72 };
    static const uint8_t cali_cmd[]  = { 0xAA, 0x55, 0x60, 0x14, 0x04, 0xCD, 0x4C, 0xB4, 0x43, 0x88 };

    /* 复位 */
    HAL_UART_Transmit(&huart1, (uint8_t *)reset_cmd, sizeof(reset_cmd), 100);
    osDelay(500);
    /* 校准 */
    HAL_UART_Transmit(&huart1, (uint8_t *)cali_cmd, sizeof(cali_cmd), 100);

    /* 复位拼帧状态，开始单字节中断接收 */
    s_index = 0;
    s_last  = 0;
    HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1);
}

float IMU601_GetYaw(void)
{
    return s_att.yaw;
}

bool IMU601_GetAttitude(IMU601_Attitude *out)
{
    if (out == NULL)
    {
        return false;
    }

    out->yaw   = s_att.yaw;
    out->pitch = s_att.pitch;
    out->roll  = s_att.roll;

    return g_imu_online;
}
