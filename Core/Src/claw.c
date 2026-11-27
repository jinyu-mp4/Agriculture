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
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include <string.h>

/* 串口发送超时(ms) */
#define CLAW_UART_TIMEOUT_MS   100

/* 反馈帧缓冲最大长度 */
#define CLAW_REPLY_MAX         64
#define CLAW_POSITION_WAIT_MS  100U   /* 发送查询后等待位置回读的时间（ms）。 */

/* 舵机 ID 为 000，位置字段单位为 µs，开到位回读允许 ±30 µs。 */
#define CLAW_OPEN_POSITION_US  660U
#define CLAW_CLOSE_POSITION_US 1000U
#define CLAW_OPEN_TOLERANCE_US 30U

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

static bool claw_move(unsigned position)
{
    /* ID 固定三位，位置和动作时间各四位。T0500 为 500 ms。 */
    char command[sizeof("#000P1000T0500!")];
    snprintf(command, sizeof(command), "#000P%04uT0500!", position);
    return claw_send(command);
}

bool Claw_Open(void)
{
    return claw_move(CLAW_OPEN_POSITION_US);
}

bool Claw_Close(void)
{
    return claw_move(CLAW_CLOSE_POSITION_US);
}

ClawOpenStatus Claw_CheckOpen(void)
{
    uint32_t frame = s_frame_count;
    if (!claw_send("#000PRAD!")) return CLAW_OPEN_NO_REPLY;

    uint32_t start = osKernelGetTickCount();
    while (osKernelGetTickCount() - start < CLAW_POSITION_WAIT_MS)
    {
        /* 中断保存的有效位置帧形如 #000P0660。查询回显和其他舵机应答不参与判断。 */
        char reply[sizeof("#000P0660")];
        taskENTER_CRITICAL();
        uint32_t current_frame = s_frame_count;
        if (current_frame != frame) memcpy(reply, s_last, sizeof(reply));
        taskEXIT_CRITICAL();

        if (current_frame != frame)
        {
            frame = current_frame;
            if (memcmp(reply, "#000P", 5) == 0)
            {
                unsigned position = 0;
                size_t digit = 5;
                while (digit < sizeof(reply) - 1 && reply[digit] >= '0' && reply[digit] <= '9')
                {
                    position = position * 10U + (unsigned)(reply[digit++] - '0');
                }
                if (digit >= 8 && reply[digit] == '\0' && position >= 500U && position <= 2500U)
                {
                    return (position >= CLAW_OPEN_POSITION_US - CLAW_OPEN_TOLERANCE_US &&
                            position <= CLAW_OPEN_POSITION_US + CLAW_OPEN_TOLERANCE_US)
                           ? CLAW_OPEN_REACHED : CLAW_OPEN_MISMATCH;
                }
            }
        }
        osDelay(5);
    }
    return CLAW_OPEN_NO_REPLY;
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
