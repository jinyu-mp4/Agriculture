/**
  ******************************************************************************
  * @file    jetson_coord.c
  * @brief   上位机 Jetson 物块二维坐标接收（UART4）
  *
  *          解析 "$x,y" 格式：x、y 为有符号十进制整数，支持负数。
  *          超范围或格式错误时丢弃该帧，重新等待 '$'。
  ******************************************************************************
  */

#include "jetson_coord.h"

/* 坐标数值上限：防止数字过长导致 int32 溢出 */
#define COORD_VALUE_MAX     100000000

/* 接收/解析状态 */
typedef struct
{
    UART_HandleTypeDef *huart;
    uint8_t  rx_byte;          /* 单字节 IT 接收缓冲 */

    /* 解析状态机：0 等 '$' / 1 解析 X / 2 解析 Y */
    uint8_t  state;
    int32_t  x, y;             /* 正在累积的坐标 */
    int8_t   sign;             /* 当前数字符号 */
    bool     num_started;      /* 当前数字是否已开始（用于区分空数字） */

    /* 最新有效结果 */
    volatile int32_t  coord_x;
    volatile int32_t  coord_y;
    volatile bool     valid;

    /* 诊断计数 */
    volatile uint32_t rx_count;      /* 累计接收字节数 */
    volatile uint32_t frame_count;   /* 累计拼出的完整帧数 */
} CoordDev;

static CoordDev s_coord = { .huart = &huart4 };

/** 把收到的 1 个字节喂给状态机 */
static void coord_parse(CoordDev *d, uint8_t b)
{
    switch (d->state)
    {
        case 0:   /* 等 '$' */
            if (b == '$')
            {
                d->state = 1;
                d->x = 0;
                d->y = 0;
                d->sign = 1;
                d->num_started = false;
            }
            break;

        case 1:   /* 解析 X，直到 ',' */
            if (b == '-')
            {
                d->sign = -1;
                d->num_started = true;
            }
            else if (b == '+')
            {
                d->sign = 1;
                d->num_started = true;
            }
            else if (b >= '0' && b <= '9')
            {
                if (d->x < COORD_VALUE_MAX) d->x = d->x * 10 + (b - '0');
                d->num_started = true;
            }
            else if (b == ',')
            {
                if (d->num_started)
                {
                    d->x *= d->sign;
                    d->state = 2;
                }
                else
                {
                    d->state = 0;   /* X 为空，格式错 */
                }
                d->sign = 1;
                d->num_started = false;
            }
            else
            {
                d->state = 0;   /* 格式错，复位 */
            }
            break;

        case 2:   /* 解析 Y，直到换行或下一个 '$' */
            if (b == '-')
            {
                d->sign = -1;
                d->num_started = true;
            }
            else if (b == '+')
            {
                d->sign = 1;
                d->num_started = true;
            }
            else if (b >= '0' && b <= '9')
            {
                if (d->y < COORD_VALUE_MAX) d->y = d->y * 10 + (b - '0');
                d->num_started = true;
            }
            else if (b == '\n' || b == '\r')
            {
                if (d->num_started)
                {
                    d->y *= d->sign;
                    d->coord_x = d->x;
                    d->coord_y = d->y;
                    d->valid = true;
                    d->frame_count++;
                }
                d->state = 0;
            }
            else if (b == '$')
            {
                /* 无换行的连续帧：先完成当前帧，再开始下一帧 */
                if (d->num_started)
                {
                    d->y *= d->sign;
                    d->coord_x = d->x;
                    d->coord_y = d->y;
                    d->valid = true;
                    d->frame_count++;
                }
                d->state = 1;
                d->x = 0;
                d->y = 0;
                d->sign = 1;
                d->num_started = false;
            }
            else
            {
                d->state = 0;   /* 格式错，复位 */
            }
            break;

        default:
            d->state = 0;
            break;
    }
}

void Coord_Init(void)
{
    HAL_UART_Receive_IT(&huart4, &s_coord.rx_byte, 1);
}

void Coord_FeedByte(UART_HandleTypeDef *huart)
{
    if (huart != &huart4) return;

    s_coord.rx_count++;
    coord_parse(&s_coord, s_coord.rx_byte);
    HAL_UART_Receive_IT(huart, &s_coord.rx_byte, 1);
}

void Coord_UART_Error(UART_HandleTypeDef *huart)
{
    if (huart != &huart4) return;

    HAL_UART_AbortReceive_IT(huart);
    HAL_UART_Receive_IT(huart, &s_coord.rx_byte, 1);
}

bool Coord_GetPoint(CoordPoint *pt)
{
    if (pt == NULL || !s_coord.valid) return false;

    do
    {
        pt->frame = s_coord.frame_count;
        pt->x = s_coord.coord_x;
        pt->y = s_coord.coord_y;
    } while (pt->frame != s_coord.frame_count);
    return true;
}

bool Coord_IsValid(void)
{
    return s_coord.valid;
}

int32_t Coord_GetX(void)
{
    return s_coord.coord_x;
}

int32_t Coord_GetY(void)
{
    return s_coord.coord_y;
}

uint32_t Coord_GetRxCount(void)
{
    return s_coord.rx_count;
}

uint32_t Coord_GetFrameCount(void)
{
    return s_coord.frame_count;
}
