/**
  ******************************************************************************
  * @file    claw.c
  * @brief   串口舵机夹爪控制（USART6）
  *
  *          发送：固定 ASCII 指令控制夹爪开合，检查 USART6 发送结果。
  *          接收：USART6 单字节中断接收，逐字节拼出夹爪回传的反馈帧（ASCII，
  *          以 '!' 或换行结尾），供屏幕显示、判断夹爪是否故障。
  ******************************************************************************
  */

#include "claw.h"
#include "cmsis_os.h"
#include <string.h>

/* 串口发送超时(ms) */
#define CLAW_UART_TIMEOUT_MS   100

/* 反馈帧缓冲最大长度 */
#define CLAW_REPLY_MAX         64

/* 开闭各用一条固定串口指令。舵机 ID 为 000，位置字段四位，T0500 为 500 ms。 */
static const char CLAW_OPEN_CMD[] = "#000P0660T0500!";
static const char CLAW_CLOSE_CMD[] = "#000P1000T0500!";

/* 接收解析状态 */
static uint8_t s_rx_byte;                       /* 单字节 IT 接收缓冲 */
static char    s_buf[CLAW_REPLY_MAX];           /* 正在拼的帧 */
static uint8_t s_len;                           /* 当前帧已收字节数 */
static char    s_last[CLAW_REPLY_MAX];          /* 最近一帧完整反馈 */
static volatile uint32_t s_rx_count;            /* 累计接收字节数 */
static volatile uint32_t s_frame_count;         /* 累计拼出的完整帧数 */

static bool claw_send(const char *cmd)
{
    /* 开闭都是固定位置，发送出错可重试同一指令，最多 3 次，间隔 20 ms。 */
    uint16_t len = (uint16_t)strlen(cmd);
    for (int attempt = 0; attempt < 3; attempt++)
    {
        if (HAL_UART_Transmit(&huart6, (uint8_t *)cmd, len, CLAW_UART_TIMEOUT_MS) == HAL_OK)
        {
            return true;
        }
        if (attempt < 2) osDelay(20);
    }
    return false;
}

bool Claw_Open(void)
{
    return claw_send(CLAW_OPEN_CMD);
}

bool Claw_Close(void)
{
    return claw_send(CLAW_CLOSE_CMD);
}

void Claw_Init(void)
{
    s_len = 0;
    s_buf[0] = '\0';
    s_last[0] = '\0';
    s_rx_count = 0;
    s_frame_count = 0;
    HAL_UART_Receive_IT(&huart6, &s_rx_byte, 1);
}

void Claw_FeedByte(UART_HandleTypeDef *huart)
{
    if (huart != &huart6)
    {
        return;
    }

    s_rx_count++;
    uint8_t b = s_rx_byte;

    /* 帧结束符：'!' 或换行。收到结束符时把当前缓冲固化成最近一帧。 */
    if (b == '!' || b == '\r' || b == '\n' || b == '\0')
    {
        if (s_len > 0)
        {
            s_buf[s_len] = '\0';
            strncpy(s_last, s_buf, CLAW_REPLY_MAX - 1);
            s_last[CLAW_REPLY_MAX - 1] = '\0';
            s_frame_count++;
        }
        s_len = 0;
        s_buf[0] = '\0';
    }
    else if (b >= 0x20 && b < 0x7F)   /* 只保留可见 ASCII，过滤不可见/噪声字节 */
    {
        if (s_len < CLAW_REPLY_MAX - 1)
        {
            s_buf[s_len++] = (char)b;
        }
        else
        {
            s_len = 0;   /* 溢出：丢弃并重新开始 */
            s_buf[0] = '\0';
        }
    }

    HAL_UART_Receive_IT(huart, &s_rx_byte, 1);
}

void Claw_UART_Error(UART_HandleTypeDef *huart)
{
    if (huart != &huart6)
    {
        return;
    }
    HAL_UART_AbortReceive_IT(huart);
    HAL_UART_Receive_IT(huart, &s_rx_byte, 1);
}

uint32_t Claw_GetRxCount(void)
{
    return s_rx_count;
}

uint32_t Claw_GetFrameCount(void)
{
    return s_frame_count;
}

const char *Claw_GetLastReply(void)
{
    return s_last;
}
